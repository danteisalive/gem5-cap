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
        : inst(NULL), next(NULL)
    { }

    DynInstPtr inst;
    //Might want to include data about what arch. register the
    //dependence is waiting on.
    PointerDependencyEntry<DynInstPtr> *next;
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
    { }

    ~PointerDependencyGraph();

    /** Resize the dependency graph to have num_entries registers. */
    void resize(int num_entries);

    /** Clears all of the linked lists. */
    void reset();

    /** Inserts an instruction to be dependent on the given index. */
    void insert(DynInstPtr &new_inst);

    /** Sets the producing instruction of a given register. */
    void setInst(PhysRegIndex idx, DynInstPtr &new_inst)
    { dependGraph[idx].inst = new_inst; }

    bool hasProducer(PhysRegIndex idx){
      if (dependGraph[idx].inst)
          return true;
      else
          return false;
    }

    /** Clears the producing instruction. */
    void clearInst(PhysRegIndex idx)
    {dependGraph[idx].inst = NULL;}

    void doSquash(uint64_t squashedSeqNum);
    /** Removes an instruction from a single linked list. */
    void remove(PhysRegIndex idx, DynInstPtr &inst_to_remove);

    /** Removes and returns the newest dependent of a specific register. */
    DynInstPtr pop(PhysRegIndex idx);

    /** Checks if the entire dependency graph is empty. */
    bool empty() const;

    /** Checks if there are any dependents on a specific register. */
    bool empty(PhysRegIndex idx) const { return !dependGraph[idx].next; }

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
    PointerDepEntry *dependGraph;

    /** Number of linked lists; identical to the number of registers. */
    int numEntries;

    // Debug variable, remove when done testing.
    unsigned memAllocCounter;

  public:
    // Debug variable, remove when done testing.
    uint64_t nodesTraversed;
    // Debug variable, remove when done testing.
    uint64_t nodesRemoved;
};

template <class DynInstPtr>
PointerDependencyGraph<DynInstPtr>::~PointerDependencyGraph()
{
    delete [] dependGraph;
}

template <class DynInstPtr>
void
PointerDependencyGraph<DynInstPtr>::resize(int num_entries)
{
    numEntries = num_entries;
    dependGraph = new PointerDepEntry[numEntries];
}

template <class DynInstPtr>
void
PointerDependencyGraph<DynInstPtr>::reset()
{
    // Clear the dependency graph
    PointerDepEntry *curr;
    PointerDepEntry *prev;

    for (int i = 0; i < numEntries; ++i) {
        curr = dependGraph[i].next;

        while (curr) {
            prev = curr;
            curr = prev->next;
            prev->inst = NULL;

            delete prev;
        }

        if (dependGraph[i].inst) {
            dependGraph[i].inst = NULL;
        }

        dependGraph[i].next = NULL;
    }
}

template <class DynInstPtr>
void
PointerDependencyGraph<DynInstPtr>::insert(DynInstPtr &inst)
{

    #define ENABLE_DEP_GRAPH_INSERT 0

      if (ENABLE_DEP_GRAPH_INSERT)
      {
          std::cout << "-------------START--------------\n";
          std::cout << inst->pcState() << " " <<
                      inst->staticInst->disassemble(inst->pcState().pc()) <<
                      " " << inst->seqNum << std::endl;
          for (size_t i = 0; i < inst->numSrcRegs(); i++) {
              std::cout << std::dec <<
                        "Arch Src: " << i << " :: " <<
                        inst->staticInst->srcRegIdx(i) <<
                        " Phy Src: "  << i << " :: " <<
                        inst->renamedSrcRegIdx(i)->flatIndex() <<
                           std::endl;
          }
          for (size_t i = 0; i < inst->numDestRegs(); i++) {
              std::cout << std::dec <<
                           "Arch Dest: " << i << " :: " <<
                           inst->staticInst->destRegIdx(i) <<
                           " Phy Dest: "  << i << " :: " <<
                           inst->renamedDestRegIdx(i)->flatIndex() <<
                           std::endl;
          }
          std::cout << std::endl;
      }


      if (inst->isMallocBaseCollectorMicroop()){
          // dest0 => dest
          int idx = inst->renamedDestRegIdx(0)->flatIndex();
          setInst(idx, inst);
      }
      else if (inst->isLoad()){
        // for one kind of load:
        // src0 => -----
        // src1 => Base
        // src2 => -----
        // Dest => dest
        // put the dest always as the producer if the datasize equals to 8
        // and it's a integer register
        if (inst->staticInst->getDataSize() == 8 &&
            inst->renamedDestRegIdx(0)->isIntPhysReg())
        {
          panic_if(inst->numDestRegs() > 1, "Load: numDestRegs > 1");
          int idx = inst->renamedDestRegIdx(0)->flatIndex();
          setInst(idx, inst);
        }

        for (size_t i = 0; i < inst->numSrcRegs(); i++) {
            if (inst->renamedSrcRegIdx(i)->isIntPhysReg()){
                int idx = inst->renamedSrcRegIdx(i)->flatIndex();
                if (hasProducer(idx)){
                // add this inst to consumer list of the respective producer
                    PointerDepEntry *new_entry = new PointerDepEntry;
                    new_entry->next = dependGraph[idx].next;
                    new_entry->inst = inst;
                    // Then actually add it to the chain.
                    dependGraph[idx].next = new_entry;
                }
            }
        }

      }
      else if (inst->isStore()){
        // for load:
        // src0 => -----
        // src1 => Base
        // src2 => src
        // src3 => -----
        // floating point stores and any kind of stores which are
        // integer type, they can be any type of write
        // for example they can write aliases, wrtie to heap or stack
        // but they never can be a producer as they dont have a dest reg
        for (size_t i = 0; i < inst->numSrcRegs(); i++) {
            if (inst->renamedSrcRegIdx(i)->isIntPhysReg()){
                int idx = inst->renamedSrcRegIdx(i)->flatIndex();
                if (hasProducer(idx)){
                // add this inst to consumer list of the respective producer
                    PointerDepEntry *new_entry = new PointerDepEntry;
                    new_entry->next = dependGraph[idx].next;
                    new_entry->inst = inst;
                    // Then actually add it to the chain.
                    dependGraph[idx].next = new_entry;
                }
            }
        }

      }
      else {
        // for all other integer instructions just go through all the
        // src regs and add them to the consumers if they have any producer
        for (size_t i = 0; i < inst->numSrcRegs(); i++) {
            if (inst->renamedSrcRegIdx(i)->isIntPhysReg()){
                int idx = inst->renamedSrcRegIdx(i)->flatIndex();
                if (hasProducer(idx)){
              // add this inst to consumer list of the respective producer
                    PointerDepEntry *new_entry = new PointerDepEntry;
                    new_entry->next = dependGraph[idx].next;
                    new_entry->inst = inst;
                    // Then actually add it to the chain.
                    dependGraph[idx].next = new_entry;
                }
            }
        }

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

    #define POINTER_DEP_GRAPH_DEBUG 0

    if (POINTER_DEP_GRAPH_DEBUG)
    {
        std::cout <<"--------START----------\n";
        std::cout << std::dec << "Squashing until sequence number " <<
                  squashedSeqNum << std::endl;
        std::cout << "Before Squashing:\n";
        dump();
    }
    // Clear the dependency graph
    PointerDepEntry *curr;
    PointerDepEntry *prev;
    // if the producer seqNum is greater than squashedSeqNum then
    // remove it and all of the consumers as they are all will be squashed
    for (size_t i = 0; i < numEntries; i++) {
        if (dependGraph[i].inst)
        {
            // if the producer should get squashed too
            if (dependGraph[i].inst->seqNum > squashedSeqNum)
            {
                // first remove all consumers
                curr = dependGraph[i].next;
                while (curr) {
                    prev = curr;
                    curr = prev->next;
                    if (POINTER_DEP_GRAPH_DEBUG)
                      cprintf("Squashed: %s [sn:%lli]\n",
                            prev->inst->pcState(), prev->inst->seqNum);
                    prev->inst = NULL;

                    delete prev;
                }

                if (POINTER_DEP_GRAPH_DEBUG)
                  cprintf("Squashed: %s [sn:%lli]\n",
                        dependGraph[i].inst->pcState(),
                        dependGraph[i].inst->seqNum);

                dependGraph[i].inst = NULL;
                dependGraph[i].next = NULL;
            }
            // if only the consumers should get squashed
            else {
                PointerDepEntry *c_node = dependGraph[i].next;
                while (c_node){
                    // find the first consumer that its seqNum is greater than
                    // squashedSeqNum
                    if (c_node->inst->seqNum > squashedSeqNum)
                    {
                        curr = c_node->next;
                        while (curr) {
                            prev = curr;
                            curr = prev->next;

                            if (POINTER_DEP_GRAPH_DEBUG)
                              cprintf("Squashed: %s [sn:%lli]\n",
                                    prev->inst->pcState(), prev->inst->seqNum);

                            prev->inst = NULL;

                            delete prev;
                        }

                        if (POINTER_DEP_GRAPH_DEBUG)
                          cprintf("Squashed: %s [sn:%lli]\n",
                                c_node->inst->pcState(), c_node->inst->seqNum);

                        c_node->inst = NULL;
                        c_node->next = NULL;
                        delete c_node;

                        break;
                    }
                    else {
                      c_node = c_node->next;
                    }
                }
            } // if the producer seqNum is less than squashedSeqNum

        }  // if there is a producer for this idx
    } // for loop

    if (POINTER_DEP_GRAPH_DEBUG)
    {
        std::cout << "After Squashing\n";
        dump();
        std::cout << "--------END----------\n";
    }
}

template <class DynInstPtr>
void
PointerDependencyGraph<DynInstPtr>::remove(PhysRegIndex idx,
                                    DynInstPtr &inst_to_remove)
{
    PointerDepEntry *prev = &dependGraph[idx];
    PointerDepEntry *curr = dependGraph[idx].next;

    // Make sure curr isn't NULL.  Because this instruction is being
    // removed from a dependency list, it must have been placed there at
    // an earlier time.  The dependency chain should not be empty,
    // unless the instruction dependent upon it is already ready.
    if (curr == NULL) {
        return;
    }

    nodesRemoved++;

    // Find the instruction to remove within the dependency linked list.
    while (curr->inst != inst_to_remove) {
        prev = curr;
        curr = curr->next;
        nodesTraversed++;
    }



    // Now remove this instruction from the list.
    prev->next = curr->next;


    // Could push this off to the destructor of DependencyEntry
    curr->inst = NULL;

    delete curr;
}

template <class DynInstPtr>
DynInstPtr
PointerDependencyGraph<DynInstPtr>::pop(PhysRegIndex idx)
{
    PointerDepEntry *node;
    node = dependGraph[idx].next;
    DynInstPtr inst = NULL;
    if (node) {
        inst = node->inst;
        dependGraph[idx].next = node->next;
        node->inst = NULL;
        memAllocCounter--;
        delete node;
    }
    return inst;
}

template <class DynInstPtr>
bool
PointerDependencyGraph<DynInstPtr>::empty() const
{
    for (int i = 0; i < numEntries; ++i) {
        if (!empty(i))
            return false;
    }
    return true;
}

template <class DynInstPtr>
void
PointerDependencyGraph<DynInstPtr>::dump()
{
    PointerDepEntry *curr;

    for (int i = 0; i < numEntries; ++i)
    {
        curr = &dependGraph[i];

        if (curr->inst) {
            cprintf("dependGraph[%i]: producer: %s [sn:%lli] consumer: ",
                    i, curr->inst->pcState(), curr->inst->seqNum);
            if (curr->next != NULL){
                while (curr->next != NULL) {
                    curr = curr->next;
                    cprintf("%s [sn:%lli] ",
                          curr->inst->pcState(), curr->inst->seqNum);
                }

            }
            cprintf("\n");

        }
        else {
            if (curr->next != NULL){
              cprintf("dependGraph[%i]: No producer. consumer: ", i);
              while (curr->next != NULL) {
                    curr = curr->next;

                    cprintf("%s [sn:%lli] ",
                            curr->inst->pcState(), curr->inst->seqNum);
              }
              cprintf("\n");
            }

       }


    }
    //cprintf("memAllocCounter: %i\n", memAllocCounter);
}

#endif // __CPU_O3_DEP_GRAPH_HH__
