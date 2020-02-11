/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Korey Sewell
 */

#include "cpu/o3/AliasCache.hh"

namespace X86ISA
{

#define ENABLE_ALIAS_CACHE_DEBUG 0

LRUAliasCache::LRUAliasCache(uint64_t _num_ways,
                            uint64_t _cache_block_size,
                            uint64_t _cache_size) :
            NumWays(_num_ways), CacheSize(_cache_size),
            CacheBlockSize(_cache_block_size),
            total_accesses(0), total_hits(0), total_misses(0),
            outstandingRead(0), outstandingWrite(0)
    {

                NumSets = (CacheSize / CacheBlockSize) / (NumWays);
                NumEntriesInCache = NumSets * NumWays;
                BitsPerSet = std::log2(NumSets);
                BitsPerBlock = std::log2(CacheBlockSize);
                ShiftAmount = BitsPerSet + BitsPerBlock;

                AliasCache = new CacheEntry*[NumSets];
                for (size_t i = 0; i < NumSets; i++) {
                  AliasCache[i]  = new CacheEntry[NumWays];
                }

                VictimCache = new LRUVictimCache(32);


                for (size_t set = 0; set < NumSets; set++) {
                    for (size_t way = 0; way < NumWays; way++) {
                      AliasCache[set][way].tag = 0;
                      AliasCache[set][way].valid = false;
                      AliasCache[set][way].dirty = false;
                      AliasCache[set][way].lruAge = 1;
                      AliasCache[set][way].pid = PointerID{0};
                      AliasCache[set][way].vaddr = 0;
                    }
                }

                stack_base = 0x7FFFFFFFF000ULL;
                max_stack_size = 32 * 1024 * 1024;
                next_thread_stack_base = stack_base - max_stack_size;
                RSPPrevValue = next_thread_stack_base;

    }

    LRUAliasCache::~LRUAliasCache(){
          delete [] AliasCache;
          delete VictimCache;
    }

        // this function is called when we want to write or read an alias
        // from the table. In the case of a write, we need to update the
        // sqn of the entry in the case of a squash
    bool LRUAliasCache::Access(Addr vaddr, ThreadContext* tc,PointerID* pid )
    {

            // first look into the SQ
            bool SQHit = AccessStoreQueue(vaddr,pid);
            if (SQHit){
              return true;
            }

            // if we are here it means a miss to SQ
            Addr thisIsTheTag = vaddr >> (ShiftAmount); //tag of the VA

            //Extract the set from the VA
            Addr thisIsTheSet =
                ((vaddr - (thisIsTheTag << ShiftAmount)) >> BitsPerBlock);


            assert(thisIsTheSet < NumSets);


            for (size_t wayNum = 0; wayNum < NumWays; wayNum++) {

                if (AliasCache[thisIsTheSet][wayNum].valid &&
                    AliasCache[thisIsTheSet][wayNum].tag == thisIsTheTag)
                {

                    assert(AliasCache[thisIsTheSet][wayNum].vaddr);
                    assert(AliasCache[thisIsTheSet][wayNum].vaddr == vaddr);
                    AliasCache[thisIsTheSet][wayNum].valid = true;
                    *pid = AliasCache[thisIsTheSet][wayNum].pid;

                    // increase the lru age
                    for (size_t i = 0; i < NumWays; i++) {
                      AliasCache[thisIsTheSet][i].lruAge++;
                    }
                    AliasCache[thisIsTheSet][wayNum].lruAge = 0;

                    return true;

                }

            }
            // if we are here then it means a miss
            // find the candiate for replamcement
            size_t candidateWay = 0;
            size_t candiateLruAge = 0;
            for (int i = 0; i < NumWays; i++) {
                if (!AliasCache[thisIsTheSet][i].valid)
                {
                  candidateWay = i;
                  break;
                }
                else if (AliasCache[thisIsTheSet][i].lruAge > candiateLruAge)
                {
                  candiateLruAge = AliasCache[thisIsTheSet][i].lruAge;
                  candidateWay = i;
                }
            }

            for (size_t i = 0; i < NumWays; i++) {
              AliasCache[thisIsTheSet][i].lruAge++;
            }


            // if the candidate entry is valid just write it to the
            // victim cache no matter it is dirty or not
            if (AliasCache[thisIsTheSet][candidateWay].valid)
            {
                VictimCache->VictimCacheWriteBack(
                              AliasCache[thisIsTheSet][candidateWay].vaddr);
            }
            // new read it from shadow_memory
            // if the page does not have any pid then it's defenitly a
            // PID(0). In this case just send it back and do not update
            // alias cache as it will just polute the cache and deacrese the
            // hit rate. If there is a page for it update the cache in any case
            Process *p = tc->getProcessPtr();
            Addr vpn = p->pTable->pageAlign(vaddr); // find the vpn

            auto it_lv1 = tc->ShadowMemory.find(vpn);
            if (it_lv1 != tc->ShadowMemory.end() && it_lv1->second.size() != 0)
            {
                // if the replamcement candidate is dirty we need to
                // writeback it before replacing it with new one
                if (AliasCache[thisIsTheSet][candidateWay].valid &&
                    AliasCache[thisIsTheSet][candidateWay].dirty)
                {

                    //send the wb_addr to WbBuffer;
                    uint64_t wb_addr =
                                AliasCache[thisIsTheSet][candidateWay].vaddr;
                    WriteBack(wb_addr);
                    Process *p = tc->getProcessPtr();
                    Addr wb_vpn = p->pTable->pageAlign(wb_addr);
                    // in this case writeback
                    PointerID wb_pid =
                                    AliasCache[thisIsTheSet][candidateWay].pid;

                    if (AliasCache[thisIsTheSet][candidateWay].pid.getPID()
                                                                        != 0){
                        tc->ShadowMemory[wb_vpn][wb_addr] = wb_pid;
                    }
                    else {
                      // if the pid == 0 we writeback if we can find the
                      // entry in the ShadowMemory
                      auto it_lv1 = tc->ShadowMemory.find(wb_vpn);
                      if (it_lv1 != tc->ShadowMemory.end())
                      {
                          auto it_lv2 = it_lv1->second.find(wb_addr);
                          if (it_lv2 != it_lv1->second.end())
                          {
                            tc->ShadowMemory[wb_vpn][wb_addr] = wb_pid;
                          }
                      }
                    }
                }

                // the page is there and not empty
                auto it_lv2 = it_lv1->second.find(vaddr);
                if (it_lv2 != it_lv1->second.end()){
                  AliasCache[thisIsTheSet][candidateWay].pid = it_lv2->second;
                  *pid = it_lv2->second;
                }
                else {
                  AliasCache[thisIsTheSet][candidateWay].pid = PointerID(0);
                  *pid = PointerID(0);
                }

                AliasCache[thisIsTheSet][candidateWay].valid = true;
                // here we read a fresh copy from ShadowMemory
                AliasCache[thisIsTheSet][candidateWay].dirty = false;
                AliasCache[thisIsTheSet][candidateWay].tag = thisIsTheTag;
                AliasCache[thisIsTheSet][candidateWay].lruAge = 0;
                AliasCache[thisIsTheSet][candidateWay].vaddr = vaddr;
            }
            else {
            // there is no alias in this page threfore just send back PID(0)
                *pid = PointerID(0);
            }

            return false;


    }

    // just initiates the access, in the case of miss
    // replacement hapeens after miss is handled
    // if it's a hit, there is no stall and InitiateAccess is complete
    bool LRUAliasCache::InitiateAccess(Addr vaddr, ThreadContext* tc){

        //TODO: stats
        total_accesses = total_accesses + 1;

        // first look into the SQ
        PointerID* pid = new PointerID(0);
        bool SQHit = AccessStoreQueue(vaddr, pid);
        if (SQHit){
            total_hits++;
            return true;
        }

        // if we are here it means a miss to SQ
        Addr thisIsTheTag = vaddr >> (ShiftAmount); //tag of the VA

        //Extract the set from the VA
        Addr thisIsTheSet =
                ((vaddr - (thisIsTheTag << ShiftAmount)) >> BitsPerBlock);


        assert(thisIsTheSet < NumSets);


        for (size_t wayNum = 0; wayNum < NumWays; wayNum++) {

            if (AliasCache[thisIsTheSet][wayNum].valid &&
                AliasCache[thisIsTheSet][wayNum].tag == thisIsTheTag)
            {

                total_hits++;
                assert(AliasCache[thisIsTheSet][wayNum].vaddr);
                assert(AliasCache[thisIsTheSet][wayNum].vaddr == vaddr);
                return true;

            }

        }

        if (VictimCache->VictimCacheInitiateRead(vaddr))
        {
          total_hits++;
          return true;
        }

        // if we are here then it means a miss
        // find the candiate for replamcement
        total_misses++;
        outstandingRead++;
        return false;


    }


    bool LRUAliasCache::CommitStore(Addr vaddr,
                                    uint64_t storeSeqNum, ThreadContext* tc){
      // here commit the youngest entry of the ExeAliasBuffer to shadow memory
      // which is actually the CommitAliasTable

      for (auto it = ExeAliasTableBuffer.cbegin(), next_it = it;
                    it != ExeAliasTableBuffer.cend();
                    it = next_it)
      {
            ++next_it;
            if (it->first.first == storeSeqNum &&
                it->first.second == vaddr)
            {

                // this is not a stack alias therfore commit it to
                // shadow memory and then erase it
                //writeback to alias cache
                // if this page has no alias and wb_pid is zero
                // do not send it for commit just remove it
                TheISA::PointerID writeback_pid = it->second;
                //Process *p = tc->getProcessPtr();
                //Addr vaddr_vpn = p->pTable->pageAlign(vaddr);
                //auto sm_it = tc->ShadowMemory.find(vaddr_vpn);
                //std::cout << writeback_pid;
                if (writeback_pid != TheISA::PointerID(0))
                {
                  //std::cout << "here!1";
                  Commit(it->first.second, tc, writeback_pid);
                }
                //delete from alias store buffer
                ExeAliasTableBuffer.erase(it);
                return true;
            }
            else {
              //std::cout << "here!";
              panic("Commiting a store which cannot be found!");
            }
      }
      return false;
    }

    bool LRUAliasCache::Commit(Addr vaddr,ThreadContext* tc, PointerID& pid)
    {
        // first tries to find the entry if it is a hit then updates
        // the entry and if it's not there first evicts an entry and
        // then overwrite it
        // if the dirty flags is set then we need an update to ShadowMemory
        //TODO: stats

        //TODO: stats
        Addr thisIsTheTag = vaddr >> (ShiftAmount); //tag of the VA

        //Extract the set from the VA
        Addr thisIsTheSet =
            ((vaddr - (thisIsTheTag << ShiftAmount)) >> BitsPerBlock);


        assert(thisIsTheSet < NumSets);


        for (size_t wayNum = 0; wayNum < NumWays; wayNum++) {

            if (AliasCache[thisIsTheSet][wayNum].valid &&
                AliasCache[thisIsTheSet][wayNum].tag == thisIsTheTag)
            {

                assert(AliasCache[thisIsTheSet][wayNum].vaddr);
                // just update the entry and return!
                AliasCache[thisIsTheSet][wayNum].tag   = thisIsTheTag;
                AliasCache[thisIsTheSet][wayNum].valid = true;
                AliasCache[thisIsTheSet][wayNum].dirty = true;
                AliasCache[thisIsTheSet][wayNum].pid   = pid;
                AliasCache[thisIsTheSet][wayNum].vaddr = vaddr;

                // increase the lru age
                for (size_t i = 0; i < NumWays; i++) {
                  AliasCache[thisIsTheSet][i].lruAge++;
                }

                AliasCache[thisIsTheSet][wayNum].lruAge = 0;

                return true;

            }

        }
        // if we are here then it means a miss
        // find the candiate for replamcement
        size_t candidateWay = 0;
        size_t candiateLruAge = 0;
        for (int i = 0; i < NumWays; i++) {
            if (!AliasCache[thisIsTheSet][i].valid){
              candidateWay = i;
              break;
            }
            else if (AliasCache[thisIsTheSet][i].lruAge > candiateLruAge){
              candiateLruAge = AliasCache[thisIsTheSet][i].lruAge;
              candidateWay = i;
            }
        }

        for (size_t i = 0; i < NumWays; i++) {
          AliasCache[thisIsTheSet][i].lruAge++;
        }

        // This entry is going to get evicted no matter it's dirty or not
        // just put it into the victim cache
        if (AliasCache[thisIsTheSet][candidateWay].valid)
        {
            VictimCache->VictimCacheWriteBack(
                          AliasCache[thisIsTheSet][candidateWay].vaddr);
        }

        // here we know that the entry is not in the cache
        // writeback to ShadowMemory if the entry that is going to get
        // overwirtten is dirty
        if (AliasCache[thisIsTheSet][candidateWay].valid &&
            AliasCache[thisIsTheSet][candidateWay].dirty)
        {

            uint64_t wb_addr = AliasCache[thisIsTheSet][candidateWay].vaddr;
            WriteBack(wb_addr);
            Process *p = tc->getProcessPtr();
            Addr wb_vpn = p->pTable->pageAlign(wb_addr);
            // in this case writeback
            TheISA::PointerID wb_pid =
                                  AliasCache[thisIsTheSet][candidateWay].pid;

            if (AliasCache[thisIsTheSet][candidateWay].pid.getPID() != 0){
                tc->ShadowMemory[wb_vpn][wb_addr] = wb_pid;
            }
            else {
              // if the pid == 0 we writeback if we can find the entry in
              // the ShadowMemory
              auto it_lv1 = tc->ShadowMemory.find(wb_vpn);
              if (it_lv1 != tc->ShadowMemory.end() &&
                  it_lv1->second.size() != 0)
              {
                  auto it_lv2 = it_lv1->second.find(wb_addr);
                  if (it_lv2 != it_lv1->second.end())
                  {
                    tc->ShadowMemory[wb_vpn][wb_addr] = wb_pid;
                  }
              }
            }
        }

        // now overwrite
        AliasCache[thisIsTheSet][candidateWay].tag =thisIsTheTag;
        AliasCache[thisIsTheSet][candidateWay].valid = true;
        AliasCache[thisIsTheSet][candidateWay].dirty = true;
        AliasCache[thisIsTheSet][candidateWay].pid   = pid;
        AliasCache[thisIsTheSet][candidateWay].vaddr = vaddr;
        AliasCache[thisIsTheSet][candidateWay].lruAge = 0;

        return false;

    }

    bool LRUAliasCache::Invalidate( ThreadContext* tc,PointerID& pid){

      // loop through the cache and invalid all freed entrys
      for (size_t setNum = 0; setNum < NumSets; setNum++) {
        for (size_t wayNum = 0; wayNum < NumWays; wayNum++) {
          if (AliasCache[setNum][wayNum].valid &&
              AliasCache[setNum][wayNum].pid == pid)
          {
            if (ENABLE_ALIAS_CACHE_DEBUG){
               std::cout << "Invalidate: " <<
                            std::hex << "EffAddr: " <<
                            AliasCache[setNum][wayNum].vaddr << " " <<
                            AliasCache[setNum][wayNum].pid <<
                            std::endl;
            }
            AliasCache[setNum][wayNum].valid = false;
          }
        }
      }

      // erase from ShadowMemory
      for (auto it_lv1 = tc->ShadowMemory.begin(),
                        next_it_lv1 = it_lv1;
                        it_lv1 != tc->ShadowMemory.end();
                        it_lv1 = next_it_lv1)
      {
              ++next_it_lv1;
              if (it_lv1->second.size() == 0)
              {
                  tc->ShadowMemory.erase(it_lv1);
              }
              else {
                  for (auto it_lv2 = it_lv1->second.cbegin(),
                           next_it_lv2 = it_lv2;
                           it_lv2 != it_lv1->second.cend();
                           it_lv2 = next_it_lv2)
                  {
                      ++next_it_lv2;
                      if (it_lv2->second.getPID() == pid.getPID())
                      {
                        it_lv1->second.erase(it_lv2);
                      }
                  }
              }
        }

        // delete all aliases that match this pid from exe alias table store
        // buffer
        // commit alias table will be updates in its own collector
        // this removal shoidl happen when returnin from Free function
        // but as we dont know the pid at the return and we are not tracking
        // anything during the free fucntion we can safelydo it here!
        for (auto it = ExeAliasTableBuffer.cbegin(), next_it = it;
                 it != ExeAliasTableBuffer.cend(); it = next_it)
        {
              ++next_it;
              if (it->second.getPID() == pid.getPID())
              {
                ExeAliasTableBuffer.erase(it);
              }
        }

        return true;


    }


    bool LRUAliasCache::RemoveStackAliases(Addr stack_addr, ThreadContext* tc){

        if (!(stack_addr <= stack_base &&
              stack_addr >= next_thread_stack_base))
            return false;

        Process *p = tc->getProcessPtr();
        Addr stack_vpn = p->pTable->pageAlign(stack_addr);

        if (stack_addr == RSPPrevValue) return false;

        auto it_lv1 = tc->ShadowMemory.find(stack_vpn);
        if (it_lv1 != tc->ShadowMemory.end() && it_lv1->second.size() != 0)
        {
           auto it_lv2 = it_lv1->second.lower_bound(stack_addr);
           if (it_lv2 != it_lv1->second.end())
           {
               it_lv1->second.erase(it_lv1->second.begin(), ++it_lv2);
           }
        }

        // loop through the cache and invalid all freed entrys
        for (size_t setNum = 0; setNum < NumSets; setNum++) {
          for (size_t wayNum = 0; wayNum < NumWays; wayNum++) {
            if (AliasCache[setNum][wayNum].valid)
            {
              assert(AliasCache[setNum][wayNum].vaddr);
              if (AliasCache[setNum][wayNum].vaddr >= next_thread_stack_base &&
                  AliasCache[setNum][wayNum].vaddr < stack_addr)
              {
                  if (ENABLE_ALIAS_CACHE_DEBUG){
                     std::cout << "RemoveStackAliases: " <<
                                  std::hex << "Stack Top: " << stack_addr <<
                                  " " << std::hex << "EffAddr: " <<
                                  AliasCache[setNum][wayNum].vaddr << " " <<
                                  AliasCache[setNum][wayNum].pid <<
                                  std::endl;
                  }
                  AliasCache[setNum][wayNum].valid = false;
              }
            }
          }
        }

        RSPPrevValue = stack_addr;

        return true;
    }


    bool LRUAliasCache::InsertStoreQueue(uint64_t seqNum, Addr effAddr,
                                         PointerID& pid)
    {
      if (ENABLE_ALIAS_CACHE_DEBUG){
         std::cout << "InsertStoreQueue: " << "SeqNum: " <<
                      std::dec << seqNum << " " <<
                      std::hex << "EffAddr: " << effAddr << " " <<
                      pid << std::endl;
      }
      ExeAliasTableBuffer[AliasTableKey(seqNum,effAddr)] = pid;

      return true;

    }

    bool LRUAliasCache::Squash(uint64_t squashed_num, bool include_inst){
      // new code
      for (auto exe_alias_buffer =
           ExeAliasTableBuffer.cbegin(), next_it = exe_alias_buffer;
           exe_alias_buffer != ExeAliasTableBuffer.cend();
           exe_alias_buffer = next_it)
      {
         ++next_it;
         if (include_inst &&
            (exe_alias_buffer->first.first >= squashed_num))
         {
           if (ENABLE_ALIAS_CACHE_DEBUG){
              std::cout << "Squash: " << "SeqNum: " << std::dec <<
                           exe_alias_buffer->first.first << " " <<
                           std::hex << "EffAddr: " <<
                           exe_alias_buffer->first.second << " " <<
                           exe_alias_buffer->second << std::endl;
           }
           ExeAliasTableBuffer.erase(exe_alias_buffer);
         }
         else if (!include_inst &&
                 (exe_alias_buffer->first.first > squashed_num))
         {
           if (ENABLE_ALIAS_CACHE_DEBUG){
              std::cout << "Squash: " << "SeqNum: " << std::dec <<
                           exe_alias_buffer->first.first << " " <<
                           std::hex << "EffAddr: " <<
                           exe_alias_buffer->first.second << " " <<
                           exe_alias_buffer->second << std::endl;
           }
           ExeAliasTableBuffer.erase(exe_alias_buffer);
         }
      }

      return true;
    }


    bool LRUAliasCache::AccessStoreQueue(Addr effAddr, PointerID* pid)
    {
      //first look in Execute Alias store buffer
      *pid = PointerID(0);
      for (auto exe_alias_buffer = ExeAliasTableBuffer.rbegin();
                  exe_alias_buffer != ExeAliasTableBuffer.rend();
                      ++exe_alias_buffer)
      {
          if (exe_alias_buffer->first.second == effAddr){
            if (ENABLE_ALIAS_CACHE_DEBUG){
               std::cout << "AccessStoreQueue: " <<
                            "SeqNum: " << std::dec <<
                            exe_alias_buffer->first.first << " " <<
                            std::hex << "EffAddr: " <<
                            exe_alias_buffer->first.second << " " <<
                            exe_alias_buffer->second << std::endl;
            }
            *pid = exe_alias_buffer->second;
            return true;  // found in SQ
          }

       }

       return false; // not in SQ
    }

    bool LRUAliasCache::SquashEntry(uint64_t squashed_num){

      for (auto exe_alias_table =
           ExeAliasTableBuffer.cbegin(), next_it = exe_alias_table;
          exe_alias_table != ExeAliasTableBuffer.cend();
          exe_alias_table = next_it)
      {
         ++next_it;
         if (exe_alias_table->first.first == squashed_num)
         {
           if (ENABLE_ALIAS_CACHE_DEBUG){
              std::cout << "SquashEntry: " << "SeqNum: " << std::dec <<
                           exe_alias_table->first.first << " " <<
                           std::hex << "EffAddr: " <<
                           exe_alias_table->first.second << " " <<
                           exe_alias_table->second << std::endl;
           }
           //remove and break we cant have two equal seqNum
           ExeAliasTableBuffer.erase(exe_alias_table);
           return true;
         }
      }

      return false;
    }

    void LRUAliasCache::print_stats() {
        printf("Alias Cache Stats: %lu, %lu, %lu, %f \n",
        total_accesses, total_hits, total_misses,
        (double)total_hits/total_accesses
        );
    }

    void LRUAliasCache::dump (){
      //dump for debugging
      for (auto& entry : ExeAliasTableBuffer) {
          std::cout << std::dec << "SeqNum: " << entry.first.first << " " <<
                    std::hex << "EffAddr: " <<
                    entry.first.second << " " << std::dec <<
                    entry.second << std::endl;
      }

    }

    void LRUAliasCache::WriteBack(Addr wb_addr){
        //first search in the qeue, if the address is there,
        // update it else just put it into the front
        if (std::find(WbBuffer.begin(), WbBuffer.end(), wb_addr) !=
                      WbBuffer.end())
        {
          return;
        }
        else {
          WbBuffer.push_front(wb_addr);
        }

        // if the number of writes is greater than threshold,
        // write back the store
        if (WbBuffer.size() > 24) // wb max threshold = 24
        {
          while (WbBuffer.size() > 8) // wb low threshold = 8
          {
            WbBuffer.pop_back();
            outstandingWrite++;
          }
        }
    }


  }
