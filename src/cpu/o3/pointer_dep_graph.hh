/*
 * Copyright (c) 2012 ARM Limited
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
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Kevin Lim
 */

#ifndef __CPU_O3_POINTER_DEP_GRAPH_HH__
#define __CPU_O3_POINTER_DEP_GRAPH_HH__

#include "cpu/o3/comm.hh"


/** Node in a linked list. */
template <class DynInstPtr>
class PointerDependencyEntry
{
  public:
    PointerDependencyEntry()
        : inst(NULL), pid(TheISA::PointerID(0))
    { }

    PointerDependencyEntry(DynInstPtr _inst, TheISA::PointerID _pid)
        : inst(_inst), pid(_pid)
    { }
    DynInstPtr inst;
    TheISA::PointerID pid{0};
};

/** Array of linked list that maintains the dependencies between
 * producing instructions and consuming instructions.  Each linked
 * list represents a single physical register, having the future
 * producer of the register's value, and all consumers waiting on that
 * value on the list.  The head node of each linked list represents
 * the producing instruction of that register.  Instructions are put
 * on the list upon reaching the IQ, and are removed from the list
 * either when the producer completes, or the instruction is squashed.
*/
template <class DynInstPtr>
class PointerDependencyGraph
{
  public:
    typedef PointerDependencyEntry<DynInstPtr> PointerDepEntry;

    /** Default construction.  Must call resize() prior to use. */
    PointerDependencyGraph()
        : numEntries(0), memAllocCounter(0), nodesTraversed(0), nodesRemoved(0)
    {
        for (int i = 0; i < TheISA::NumIntRegs; ++i) {
            dependGraph[i].clear();
            FetchArchRegsPid[i] = TheISA::PointerID(0);
            CommitArchRegsPid[i] = TheISA::PointerID(0);
        }
    }

    ~PointerDependencyGraph();

    /** Resize the dependency graph to have num_entries registers. */
    // void resize(int num_entries);

    /** Clears all of the linked lists. */
    void reset();

    /** Inserts an instruction to be dependent on the given index. */
    void insert(DynInstPtr &new_inst);


    void doSquash(uint64_t squashedSeqNum);

    void doUpdate(DynInstPtr& inst);
    void InternalUpdate(PointerDepEntry* it ,DynInstPtr &inst);
    /** Removes an instruction from a single linked list. */
    void doCommit(DynInstPtr inst);

    /** Debugging function to dump out the dependency graph.
     */
    void dump();

  private:
    /** Array of linked lists.  Each linked list is a list of all the
     *  instructions that depend upon a given register.  The actual
     *  register's index is used to index into the graph; ie all
     *  instructions in flight that are dependent upon r34 will be
     *  in the linked list of dependGraph[34].
     */

    std::array<std::deque<PointerDepEntry>, TheISA::NumIntRegs> dependGraph;
    std::array<TheISA::PointerID, TheISA::NumIntRegs> CommitArchRegsPid;
    std::array<TheISA::PointerID, TheISA::NumIntRegs> FetchArchRegsPid;

    /** Number of linked lists; identical to the number of registers. */
    int numEntries;

    // Debug variable, remove when done testing.
    unsigned memAllocCounter;

  public:
    // Debug variable, remove when done testing.
    uint64_t nodesTraversed;
    // Debug variable, remove when done testing.
    uint64_t nodesRemoved;

    std::array<TheISA::PointerID, TheISA::NumIntRegs>
     getFetchArchRegsPidArray(){
       return FetchArchRegsPid;
    }

    void setFetchArchRegsPidArray(int idx, TheISA::PointerID _pid){
        FetchArchRegsPid[idx] = _pid;
    }
    void setCommitArchRegsPidArray(int idx, TheISA::PointerID _pid){
        CommitArchRegsPid[idx] = _pid;
    }
};

template <class DynInstPtr>
PointerDependencyGraph<DynInstPtr>::~PointerDependencyGraph()
{}


template <class DynInstPtr>
void
PointerDependencyGraph<DynInstPtr>::reset()
{

    for (int i = 0; i < TheISA::NumIntRegs; ++i) {
        dependGraph[i].clear();
        FetchArchRegsPid[i] = TheISA::PointerID(0);
        CommitArchRegsPid[i] = TheISA::PointerID(0);
    }
}

template <class DynInstPtr>
void
PointerDependencyGraph<DynInstPtr>::insert(DynInstPtr &inst)
{

    #define ENABLE_DEP_GRAPH_INSERT 0

    if (inst->isBoundsCheckMicroop()) return;

    //transfer capabilities
    if (inst->isMallocBaseCollectorMicroop() ||
        inst->isCallocBaseCollectorMicroop() ||
        inst->isReallocBaseCollectorMicroop())
    {
        // here we generate a new PID and insert it into rax
        dependGraph[X86ISA::INTREG_RAX].push_front(
                                        PointerDepEntry(inst, inst->dyn_pid));
        FetchArchRegsPid[X86ISA::INTREG_RAX] = inst->dyn_pid;

        //dump();
    }
    else if (inst->isFreeCallMicroop() ||
            inst->isReallocSizeCollectorMicroop())
    {
      dependGraph[X86ISA::INTREG_RDI].push_front(
                                      PointerDepEntry(inst, inst->dyn_pid));
      FetchArchRegsPid[X86ISA::INTREG_RDI] = inst->dyn_pid;
    }
    // this can be a potential pointer refill
    else if (inst->isLoad() && inst->staticInst->getDataSize() == 8)
    {
        // get the predicted PID for this load
        TheISA::PointerID _pid = inst->macroop->getMacroopPid();
        // insert an entry for the destination reg
        for (size_t i = 0; i < inst->staticInst->numDestRegs(); i++) {
            if (inst->staticInst->destRegIdx(i).isIntReg() &&
                (inst->staticInst->destRegIdx(i).index() < TheISA::NumIntRegs))
            {
                int dest_reg_idx = inst->staticInst->destRegIdx(i).index();
                dependGraph[dest_reg_idx].push_front(
                                              PointerDepEntry(inst, _pid));
                FetchArchRegsPid[dest_reg_idx] = _pid;
            }
        }
    }
    else if (inst->isLoad() && inst->staticInst->getDataSize() != 8)
    {
        // this is defenitly not a pointer refill
        // set the dest regs as PID(0)
        TheISA::PointerID _pid = TheISA::PointerID(0);
        for (size_t i = 0; i < inst->staticInst->numDestRegs(); i++) {
            if (inst->staticInst->destRegIdx(i).isIntReg() &&
               (inst->staticInst->destRegIdx(i).index() < TheISA::NumIntRegs))
            {
                int dest_reg_idx = inst->staticInst->destRegIdx(i).index();
                dependGraph[dest_reg_idx].push_front(
                                              PointerDepEntry(inst, _pid));
                FetchArchRegsPid[dest_reg_idx] = _pid;
            }
        }
    }
    else if (inst->staticInst->getDataSize() == 8) {

        TheISA::PointerID _pid{0} ;
        for (size_t i = 0; i < inst->staticInst->numSrcRegs(); i++) {
          if (inst->staticInst->srcRegIdx(i).isIntReg() &&
              (inst->staticInst->srcRegIdx(i).index() < TheISA::NumIntRegs))
          {
              // if one of the sources is not pid(0), assign it to pid
              // and break;
              int src_reg_idx = inst->staticInst->srcRegIdx(i).index();
              if (FetchArchRegsPid[src_reg_idx] != TheISA::PointerID(0))
              {
                _pid = FetchArchRegsPid[src_reg_idx];
                break;
              }
          }
        }

        for (size_t i = 0; i < inst->staticInst->numDestRegs(); i++) {
          if (inst->staticInst->destRegIdx(i).isIntReg() &&
              (inst->staticInst->destRegIdx(i).index() < TheISA::NumIntRegs))
          {
             // assign pid to all of the dest regs
             int dest_reg_idx = inst->staticInst->destRegIdx(i).index();
             dependGraph[dest_reg_idx].push_front(
                                        PointerDepEntry(inst, _pid));
             FetchArchRegsPid[dest_reg_idx] = _pid;
          }
        }

    }
    // defenitly this is not a pointer transefer
    // zero out all the dest regs
    else if (inst->staticInst->getDataSize() != 8) {

        TheISA::PointerID _pid = TheISA::PointerID(0);
        for (size_t i = 0; i < inst->staticInst->numDestRegs(); i++) {
          if (inst->staticInst->destRegIdx(i).isIntReg() &&
              (inst->staticInst->destRegIdx(i).index() < TheISA::NumIntRegs))
          {
              // zero out all dest regs
              int dest_reg_idx = inst->staticInst->destRegIdx(i).index();
              dependGraph[dest_reg_idx].push_front(
                                        PointerDepEntry(inst, _pid));
              FetchArchRegsPid[dest_reg_idx] = _pid;
          }
        }
    }

    // zero out all interface regs for the next macroopp
    if (inst->isLastMicroop()){
      TheISA::PointerID _pid = TheISA::PointerID(0);
      for (size_t i = X86ISA::NUM_INTREGS; i < TheISA::NumIntRegs; i++) {
          //zero out all dest regs
          FetchArchRegsPid[i] = _pid;
      }
      //dump();
    }



      if (ENABLE_DEP_GRAPH_INSERT)
      {
        dump();
        std::cout << "-------------END--------------\n";
      }


}

template <class DynInstPtr>
void
PointerDependencyGraph<DynInstPtr>::doSquash(uint64_t squashedSeqNum){

    #define POINTER_DEP_GRAPH_SQUASH_DEBUG 0

    if (POINTER_DEP_GRAPH_SQUASH_DEBUG)
    {
        std::cout <<"--------START----------\n";
        std::cout << std::dec << "Squashing until sequence number " <<
                  squashedSeqNum << std::endl;
        std::cout << "Before Squashing:\n";
        dump();
    }

    // if the producer seqNum is greater than squashedSeqNum then
    // remove it and all of the consumers as they are all will be squashed

    for (size_t i = 0; i < TheISA::NumIntRegs; i++) {

        // Erase all even numbers (C++11 and later)
        for (auto it = dependGraph[i].begin(); it != dependGraph[i].end(); )
        {
            if (it->inst->seqNum > squashedSeqNum) {
                it->inst = NULL;
                it = dependGraph[i].erase(it);
            } else {
                ++it;
            }
        }

    } // for loop

    // now for each int reg update the FetchArchRegsPid with the front inst
    // if there is no inst in the queue then update it with the
    // CommitArchRegsPid
    for (size_t i = 0; i < TheISA::NumIntRegs; i++) {
        if (dependGraph[i].empty()){
          FetchArchRegsPid[i] = CommitArchRegsPid[i];
        }
        else {
          FetchArchRegsPid[i] = dependGraph[i].front().pid;
        }
    }

    if (POINTER_DEP_GRAPH_SQUASH_DEBUG)
    {
        std::cout << "After Squashing\n";
        dump();
        std::cout << "--------END----------\n";
    }
}

template <class DynInstPtr>
void
PointerDependencyGraph<DynInstPtr>::doCommit(DynInstPtr inst){

    #define POINTER_DEP_GRAPH_COMMIT_DEBUG 0

    if (inst->isBoundsCheckMicroop()) return; // we do not save these

    if (POINTER_DEP_GRAPH_COMMIT_DEBUG)
    {
        std::cout <<"--------START----------\n";
        std::cout << std::dec << "Commiting: [" <<
                  inst->seqNum << "]"<< std::endl;
        std::cout << "Before Commit:\n";
        dump();
    }

    // for all the dest regs for this inst, commit it
    // assert if the inst is not in the dependGraph
    for (size_t i = 0; i < inst->staticInst->numDestRegs(); i++) {
      if (inst->staticInst->destRegIdx(i).isIntReg() &&
          (inst->staticInst->destRegIdx(i).index() < TheISA::NumIntRegs))
      {
         // the inst should be at the end of the queue
         int dest_reg_idx = inst->staticInst->destRegIdx(i).index();
         panic_if(dependGraph[dest_reg_idx].back().inst->seqNum !=
                  inst->seqNum,
                  "Dangling inst in PointerDependGraph");
         CommitArchRegsPid[dest_reg_idx] =
                          dependGraph[dest_reg_idx].back().pid;
         dependGraph[dest_reg_idx].back().inst = NULL;
         dependGraph[dest_reg_idx].pop_back();

      }
    }

    if (POINTER_DEP_GRAPH_COMMIT_DEBUG)
    {
        std::cout << "After Commit\n";
        dump();
        std::cout << "--------END----------\n";
    }
}

template <class DynInstPtr>
void
PointerDependencyGraph<DynInstPtr>::dump()
{

    for (size_t i = 0; i < TheISA::NumIntRegs; i++) {

        std::cout << "PointerDependGraph[" << i << "][" <<
        FetchArchRegsPid[i] << "][" << CommitArchRegsPid[i] << "]: ";
        // Erase all even numbers (C++11 and later)
        for (auto it = dependGraph[i].begin();
             it != dependGraph[i].end(); it++)
        {
          assert(it->inst); // shouldnt be a null inst
          std::cout << it->inst->pcState() <<
                      " [sn:" <<  it->inst->seqNum << "]" << it->pid << " ";
        }

        std::cout << std::endl;

    } // for loop

}

template <class DynInstPtr>
void
PointerDependencyGraph<DynInstPtr>::doUpdate(DynInstPtr& inst)
{
    #define POINTER_DEP_GRAPH_UPDATE_DEBUG 0

    if (inst->isBoundsCheckMicroop()) return; // we do not save these
    if (!inst->isLoad()) {
        panic("doUpdate called with a non-load instruction!");
    }

    if (POINTER_DEP_GRAPH_UPDATE_DEBUG)
    {
        std::cout <<"--------START----------\n";
        std::cout << std::dec << "Updating: [" <<
                  inst->seqNum << "]"<< std::endl;
        std::cout << "Before Update:\n";
        dump();
    }

    //find the load uop
    // we should defenitly find it othersie it's a panic!
    // doUpdate happends before squash so we should find it in the DEP Graph
    bool found = false;
    for (size_t indx = 0; indx < TheISA::NumIntRegs; indx++) {
        // Erase  (C++11 and later)
        for (auto it = dependGraph[indx].begin();
            it != dependGraph[indx].end(); it++)
        {
            //if found: update and return
            if (it->inst->seqNum == inst->seqNum) {
                // get the actual PID for this load
                TheISA::PointerID _pid = inst->macroop->getMacroopPid();
                // insert an entry for the destination reg
                for (size_t i = 0; i < inst->staticInst->numDestRegs(); i++) {
                    if (inst->staticInst->destRegIdx(i).isIntReg() &&
                        (inst->staticInst->destRegIdx(i).index() <
                                                  TheISA::NumIntRegs))
                    {
                        int dest_reg_idx =
                            inst->staticInst->destRegIdx(i).index();
                        panic_if(dest_reg_idx != i,
                          "destination reg id does not \
                           match with updated load!");
                        it->pid = _pid;
                    }
                }

                found = true;
                break;

            }
        }

    } // for loop
    panic_if(!found, "Could not find the load uop\
                     when updating the DEP Graph!");

    // now one by one find all the instruction with seqNum greater
    // than load uop and update their uops
    for (size_t indx = 0; indx < TheISA::NumIntRegs; indx++) {
        // Erase  (C++11 and later)
        for (auto it = dependGraph[indx].begin();
          it != dependGraph[indx].end(); it++)
        {
            if (it->inst->seqNum > inst->seqNum) {
              //InternalUpdate(it, inst);
            }
        }
    }

    // now update FetchArchRegsPid with the latest values
    for (size_t i = 0; i < TheISA::NumIntRegs; i++) {
        FetchArchRegsPid[i] = dependGraph[i].front().pid;
    }


    if (POINTER_DEP_GRAPH_COMMIT_DEBUG)
    {
        std::cout << "After Update\n";
        dump();
        std::cout << "--------END----------\n";
    }



}

template <class DynInstPtr>
void
PointerDependencyGraph<DynInstPtr>::InternalUpdate(
                                  PointerDepEntry* it ,DynInstPtr &inst)
{

    #define ENABLE_DEP_GRAPH_INTERNAL_UPDATE 0

    if (inst->isBoundsCheckMicroop()) return;

    if (inst->isLoad() && inst->staticInst->getDataSize() == 8)
    {
        // get the predicted PID for this load
        TheISA::PointerID _pid = inst->macroop->getMacroopPid();
        // insert an entry for the destination reg
        for (size_t i = 0; i < inst->staticInst->numDestRegs(); i++) {
            if (inst->staticInst->destRegIdx(i).isIntReg() &&
                (inst->staticInst->destRegIdx(i).index() < TheISA::NumIntRegs))
            {
                int dest_reg_idx = inst->staticInst->destRegIdx(i).index();
                dependGraph[dest_reg_idx].push_front(
                                              PointerDepEntry(inst, _pid));
                FetchArchRegsPid[dest_reg_idx] = _pid;
            }
        }
    }
    else if (inst->isLoad() && inst->staticInst->getDataSize() != 8)
    {
        // this is defenitly not a pointer refill
        // set the dest regs as PID(0)
        TheISA::PointerID _pid = TheISA::PointerID(0);
        for (size_t i = 0; i < inst->staticInst->numDestRegs(); i++) {
            if (inst->staticInst->destRegIdx(i).isIntReg() &&
               (inst->staticInst->destRegIdx(i).index() < TheISA::NumIntRegs))
            {
                int dest_reg_idx = inst->staticInst->destRegIdx(i).index();
                dependGraph[dest_reg_idx].push_front(
                                              PointerDepEntry(inst, _pid));
                FetchArchRegsPid[dest_reg_idx] = _pid;
            }
        }
    }
    else if (inst->staticInst->getDataSize() == 8) {

        TheISA::PointerID _pid{0} ;
        for (size_t i = 0; i < inst->staticInst->numSrcRegs(); i++) {
          if (inst->staticInst->srcRegIdx(i).isIntReg() &&
              (inst->staticInst->srcRegIdx(i).index() < TheISA::NumIntRegs))
          {
              // if one of the sources is not pid(0), assign it to pid
              // and break;
              int src_reg_idx = inst->staticInst->srcRegIdx(i).index();
              if (FetchArchRegsPid[src_reg_idx] != TheISA::PointerID(0))
              {
                _pid = FetchArchRegsPid[src_reg_idx];
                break;
              }
          }
        }

        for (size_t i = 0; i < inst->staticInst->numDestRegs(); i++) {
          if (inst->staticInst->destRegIdx(i).isIntReg() &&
              (inst->staticInst->destRegIdx(i).index() < TheISA::NumIntRegs))
          {
             // assign pid to all of the dest regs
             int dest_reg_idx = inst->staticInst->destRegIdx(i).index();
             dependGraph[dest_reg_idx].push_front(
                                        PointerDepEntry(inst, _pid));
             FetchArchRegsPid[dest_reg_idx] = _pid;
          }
        }

    }
    // defenitly this is not a pointer transefer
    // zero out all the dest regs
    else if (inst->staticInst->getDataSize() != 8) {

        TheISA::PointerID _pid = TheISA::PointerID(0);
        for (size_t i = 0; i < inst->staticInst->numDestRegs(); i++) {
          if (inst->staticInst->destRegIdx(i).isIntReg() &&
              (inst->staticInst->destRegIdx(i).index() < TheISA::NumIntRegs))
          {
              // zero out all dest regs
              int dest_reg_idx = inst->staticInst->destRegIdx(i).index();
              dependGraph[dest_reg_idx].push_front(
                                        PointerDepEntry(inst, _pid));
              FetchArchRegsPid[dest_reg_idx] = _pid;
          }
        }
    }

    // zero out all interface regs for the next macroopp
    if (inst->isLastMicroop()){
      TheISA::PointerID _pid = TheISA::PointerID(0);
      for (size_t i = X86ISA::NUM_INTREGS; i < TheISA::NumIntRegs; i++) {
          //zero out all dest regs
          FetchArchRegsPid[i] = _pid;
      }
      //dump();
    }



      if (ENABLE_DEP_GRAPH_INTERNAL_UPDATE)
      {
        dump();
        std::cout << "-------------END--------------\n";
      }


}

#endif // __CPU_O3_DEP_GRAPH_HH__
