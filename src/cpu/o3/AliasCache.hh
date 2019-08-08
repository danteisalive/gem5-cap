/*
 * Copyright (c) 2011-2012, 2014 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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

#ifndef __CPU_O3_ALIAS_CACHE_HH__
#define __CPU_O3_ALIAS_CACHE_HH__

#include <iostream>
#include <map>

#include "arch/x86/insts/static_inst.hh"
#include "arch/x86/types.hh"
#include "debug/Capability.hh"
#include "mem/page_table.hh"
#include "sim/process.hh"

namespace X86ISA
{

class LRUVictimCache
{
  private:
    std::deque<Addr>            VictimCache;
    int                          NumOfEntrys;

  public:
    LRUVictimCache(int _size): NumOfEntrys(_size)
    {}

    bool VictimCacheInitiateRead(Addr vaddr){

      bool hit = false;
      // First find the entry, if found bring it to the top of the queue
      for (auto it = VictimCache.begin(); it != VictimCache.end(); )
      {
          if (*it == vaddr) {
              it = VictimCache.erase(it);
              hit = true;
              break;
          }
          else {
            it++;
          }
      }

      // if true, just change the entry place as LRU policy
      if (hit){
        VictimCache.push_front(vaddr);
      }

      return hit;
    }

    // whenever there is an eviction from the alias cache, this function is
    //called to store the eviction address
    void VictimCacheWriteBack(Addr vaddr){
        // first look to see if we have the evicted address or not
        bool hit = false;
        // First find the entry, if found bring it to the top of the queue
        for (auto it = VictimCache.begin(); it != VictimCache.end(); )
        {
            if (*it == vaddr) {
                it = VictimCache.erase(it);
                hit = true;
                break;
            }
            else {
              it++;
            }
        }

        if (hit){
          VictimCache.push_front(vaddr);
          return;
        }

        // by here we dont have any entry with the same address
        if (VictimCache.size() <= NumOfEntrys){
          VictimCache.push_front(vaddr);
        }
        else {
          VictimCache.pop_back();
          VictimCache.push_front(vaddr);
        }

    }

};


class LRUAliasCache
{
    typedef std::pair<uint64_t, uint64_t> AliasTableKey;
    typedef std::map<AliasTableKey, TheISA::PointerID> ExeAliasBuffer;

    private:
    void WriteBack(Addr wb_addr);

    private:
        CacheEntry**                 AliasCache;
        ExeAliasBuffer               ExeAliasTableBuffer;
        LRUVictimCache*              VictimCache;
        std::deque<Addr>             WbBuffer;

        uint64_t                     NumWays;
        uint64_t                     NumSets;
        uint64_t                     CacheSize;
        uint64_t                     CacheBlockSize;
        uint64_t                     NumEntriesInCache;
        uint64_t                     BitsPerBlock;
        uint64_t                     BitsPerSet;
        uint64_t                     ShiftAmount;

        uint64_t                     RSPPrevValue;
        Addr                         stack_base ;
        Addr                         max_stack_size ;
        Addr                         next_thread_stack_base;

    public:

        //TODO: move these to gem5 stats
        uint64_t                     total_accesses;
        uint64_t                     total_hits;
        uint64_t                     total_misses;
        uint64_t                     outstandingRead;
        uint64_t                     outstandingWrite;

        LRUAliasCache(uint64_t _num_ways,
                            uint64_t _cache_block_size,
                            uint64_t _cache_size);

        ~LRUAliasCache();

        bool Access(Addr vaddr, ThreadContext* tc, PointerID* pid ) ;

        bool InitiateAccess(Addr vaddr,ThreadContext* tc);

        bool Commit(Addr vaddr, ThreadContext* tc, PointerID& pid);
        bool CommitStore(Addr vaddr,uint64_t storeSeqNum, ThreadContext* tc);
        bool Invalidate(ThreadContext* tc, PointerID& pid);

        bool RemoveStackAliases(Addr vaddr, ThreadContext* tc);

        bool InsertStoreQueue(uint64_t seqNum, Addr effAddr, PointerID& pid);

        bool Squash(uint64_t seqNum, bool include_inst);

        bool AccessStoreQueue(Addr effAddr, TheISA::PointerID* pid);

        bool SquashEntry(uint64_t squashed_num);

        void print_stats() ;
        void dump ();

        uint64_t GetSize(){
           return ExeAliasTableBuffer.size();
        }

};
}
#endif // __CPU_O3_ALIAS_CACHE_HH__
