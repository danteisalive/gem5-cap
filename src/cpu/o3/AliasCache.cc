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


LRUAliasCache::LRUAliasCache(uint64_t _num_ways,
                            uint64_t _cache_block_size,
                            uint64_t _cache_size) :
            NumWays(_num_ways), CacheSize(_cache_size),
            CacheBlockSize(_cache_block_size),
            total_accesses(0), total_hits(0), total_misses(0)
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


                for (size_t set = 0; set < NumSets; set++) {
                    for (size_t way = 0; way < NumWays; way++) {
                      AliasCache[set][way].tag = 0;
                      AliasCache[set][way].valid = false;
                      AliasCache[set][way].dirty = false;
                      AliasCache[set][way].lruAge = 1;
                      AliasCache[set][way].pid = PointerID{0};
                    }
                }

    }

    LRUAliasCache::~LRUAliasCache(){
          delete [] AliasCache;
    }

        // this function is called when we want to write or read an alias
        // from the table. In the case of a write, we need to update the
        // sqn of the entry in the case of a squash
    bool LRUAliasCache::Access(Addr vaddr,
                    ThreadContext* tc,
                    PointerID* pid )
    {

            //TODO: stats
            total_accesses = total_accesses + 1;

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
            total_misses++;
            size_t candidateWay = 0;
            size_t candiateLruAge = 0;
            for (int i = 0; i < NumWays; i++) {
                if (AliasCache[thisIsTheSet][i].lruAge > candiateLruAge){
                  candiateLruAge = AliasCache[thisIsTheSet][i].lruAge;
                  candidateWay = i;
                }
            }

            for (size_t i = 0; i < NumWays; i++) {
              AliasCache[thisIsTheSet][i].lruAge++;
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
                AliasCache[thisIsTheSet][candidateWay].tag = thisIsTheTag;
                AliasCache[thisIsTheSet][candidateWay].lruAge = 0;
            }
            else {
            // there is no alias in this page threfore just send back PID(0)
                total_misses--; // this was acutually a hit
                total_hits++;
                *pid = PointerID(0);
            }

            return false;


    }

    bool LRUAliasCache::initiateAccess(Addr vaddr, ThreadContext* tc){
        //TODO: stats
        total_accesses = total_accesses + 1;

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
                AliasCache[thisIsTheSet][wayNum].valid = true;

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
        total_misses++;
        size_t candidateWay = 0;
        size_t candiateLruAge = 0;
        for (int i = 0; i < NumWays; i++) {
            if (AliasCache[thisIsTheSet][i].lruAge > candiateLruAge){
              candiateLruAge = AliasCache[thisIsTheSet][i].lruAge;
              candidateWay = i;
            }
        }

        for (size_t i = 0; i < NumWays; i++) {
          AliasCache[thisIsTheSet][i].lruAge++;
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
            // the page is there and not empty
            auto it_lv2 = it_lv1->second.find(vaddr);
            if (it_lv2 != it_lv1->second.end()){
              AliasCache[thisIsTheSet][candidateWay].pid = it_lv2->second;
            }
            else {
              AliasCache[thisIsTheSet][candidateWay].pid = PointerID(0);
            }

            AliasCache[thisIsTheSet][candidateWay].valid = true;
            AliasCache[thisIsTheSet][candidateWay].tag = thisIsTheTag;
            AliasCache[thisIsTheSet][candidateWay].lruAge = 0;
        }
        else {
        // there is no alias in this page threfore just send back PID(0)
            total_misses--; // this was acutually a hit
            total_hits++;
        }

        return false;


    }

    void LRUAliasCache::print_stats() {
        printf("Alias Cache Stats: %lu, %lu, %lu, %f \n",
        total_accesses, total_hits, total_misses,
        (double)total_hits/total_accesses
        );
    }

  }
