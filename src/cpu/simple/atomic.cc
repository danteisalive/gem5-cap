/*
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2012-2013,2015,2017-2018 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Steve Reinhardt
 */

#include "cpu/simple/atomic.hh"

#include "arch/locked_mem.hh"
#include "arch/mmapped_ipr.hh"
#include "arch/utility.hh"
#include "base/output.hh"
#include "config/the_isa.hh"
#include "cpu/exetrace.hh"
#include "debug/Drain.hh"
#include "debug/ExecFaulting.hh"
#include "debug/SimpleCPU.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/physical.hh"
#include "params/AtomicSimpleCPU.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

void
AtomicSimpleCPU::init()
{
    BaseSimpleCPU::init();

    int cid = threadContexts[0]->contextId();
    ifetch_req->setContext(cid);
    data_read_req->setContext(cid);
    data_write_req->setContext(cid);
}

AtomicSimpleCPU::AtomicSimpleCPU(AtomicSimpleCPUParams *p)
    : BaseSimpleCPU(p),
      tickEvent([this]{ tick(); }, "AtomicSimpleCPU tick",
                false, Event::CPU_Tick_Pri),
      width(p->width), locked(false),
      simulate_data_stalls(p->simulate_data_stalls),
      simulate_inst_stalls(p->simulate_inst_stalls),
      icachePort(name() + ".icache_port", this),
      dcachePort(name() + ".dcache_port", this),
      dcache_access(false), dcache_latency(0),
      ppCommit(nullptr)
{
    _status = Idle;
    ifetch_req = std::make_shared<Request>();
    data_read_req = std::make_shared<Request>();
    data_write_req = std::make_shared<Request>();


    numOfMemRefs = 0;
    numOfHeapAccesses = 0;
    threadContexts[0]->enableCapability = p->enable_capability;
    threadContexts[0]->symbolsFile = p->symbol_file;
    threadContexts[0]->Collector_Status = ThreadContext::NONE;
    threadContexts[0]->AtomicPID = 0;
    threadContexts[0]->num_of_allocations = 0;

    max_insts_any_thread = p->max_insts_any_thread;
    if (p->symbol_file != ""){
        std::string line;
        std::ifstream myfile (threadContexts[0]->symbolsFile);
        if (myfile.is_open()){
            Addr pcAddr;
            uint64_t checkType;
            while ( std::getline (myfile,line) ){
                std::istringstream iss(line);
                iss >> std::hex >> pcAddr  >> checkType ;
                (threadContexts[0]->syms_cache).insert(
                          std::pair<Addr, TheISA::CheckType>
                          (pcAddr, (TheISA::CheckType)checkType));
            }
            myfile.close();
        }
        else fatal("Can't open symbols file");
    }

    std::cout << "Atomic CPU Initilization: " << std::endl;
    threadContexts[0]->interval_tree = VG_newFM(interval_tree_Cmp);
    threadContexts[0]->FunctionSymbols = VG_newFM(interval_tree_Cmp);
    threadContexts[0]->FunctionsToIgnore = VG_newFM(interval_tree_Cmp);
    threadContexts[0]->InSlice = false;

    for (size_t i = 0; i < TheISA::NumIntRegs; i++) {
        threadContexts[0]->PointerTracker[i] = 0;
    }
    //symtab
    Process *process = threadContexts[0]->getProcessPtr();
    std::stringstream test(process->progName());
    std::string segment;
    std::vector<std::string> seglist;

    while (std::getline(test, segment, '/'))
    {
       seglist.push_back(segment);
    }

    if (!readSymTab(seglist[seglist.size()-1].c_str(),threadContexts[0])){
      warn("cannot read symtab!");
    }

    // UWord keyW, valW;
    // VG_initIterFM(threadContexts[0]->FunctionSymbols);
    // while (
    //     VG_nextIterFM(threadContexts[0]->FunctionSymbols, &keyW, &valW )) {
    //    Block* bk = (Block*)keyW;
    //    assert(valW == 0);
    //    assert(bk);
    //    std::cout << std::hex << bk->payload << " " << bk->name << std::endl;
    // }
    // VG_doneIterFM( threadContexts[0]->FunctionSymbols );
}


AtomicSimpleCPU::~AtomicSimpleCPU()
{
    if (tickEvent.scheduled()) {
        deschedule(tickEvent);
    }

}

DrainState
AtomicSimpleCPU::drain()
{
    // Deschedule any power gating event (if any)
    deschedulePowerGatingEvent();

    if (switchedOut())
        return DrainState::Drained;

    if (!isDrained()) {
        DPRINTF(Drain, "Requesting drain.\n");
        return DrainState::Draining;
    } else {
        if (tickEvent.scheduled())
            deschedule(tickEvent);

        activeThreads.clear();
        DPRINTF(Drain, "Not executing microcode, no need to drain.\n");
        return DrainState::Drained;
    }
}

void
AtomicSimpleCPU::threadSnoop(PacketPtr pkt, ThreadID sender)
{
    DPRINTF(SimpleCPU, "received snoop pkt for addr:%#x %s\n", pkt->getAddr(),
            pkt->cmdString());

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (tid != sender) {
            if (getCpuAddrMonitor(tid)->doMonitor(pkt)) {
                wakeup(tid);
            }

            TheISA::handleLockedSnoop(threadInfo[tid]->thread,
                                      pkt, dcachePort.cacheBlockMask);
        }
    }
}

void
AtomicSimpleCPU::drainResume()
{
    assert(!tickEvent.scheduled());
    if (switchedOut())
        return;

    DPRINTF(SimpleCPU, "Resume\n");
    verifyMemoryMode();

    assert(!threadContexts.empty());

    _status = BaseSimpleCPU::Idle;

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (threadInfo[tid]->thread->status() == ThreadContext::Active) {
            threadInfo[tid]->notIdleFraction = 1;
            activeThreads.push_back(tid);
            _status = BaseSimpleCPU::Running;

            // Tick if any threads active
            if (!tickEvent.scheduled()) {
                schedule(tickEvent, nextCycle());
            }
        } else {
            threadInfo[tid]->notIdleFraction = 0;
        }
    }

    // Reschedule any power gating event (if any)
    schedulePowerGatingEvent();
}

bool
AtomicSimpleCPU::tryCompleteDrain()
{
    if (drainState() != DrainState::Draining)
        return false;

    DPRINTF(Drain, "tryCompleteDrain.\n");
    if (!isDrained())
        return false;

    DPRINTF(Drain, "CPU done draining, processing drain event\n");
    signalDrainDone();

    return true;
}


void
AtomicSimpleCPU::switchOut()
{
    BaseSimpleCPU::switchOut();

    assert(!tickEvent.scheduled());
    assert(_status == BaseSimpleCPU::Running || _status == Idle);
    assert(isDrained());
}


void
AtomicSimpleCPU::takeOverFrom(BaseCPU *oldCPU)
{
    BaseSimpleCPU::takeOverFrom(oldCPU);

    // The tick event should have been descheduled by drain()
    assert(!tickEvent.scheduled());
}

void
AtomicSimpleCPU::verifyMemoryMode() const
{
    if (!system->isAtomicMode()) {
        fatal("The atomic CPU requires the memory system to be in "
              "'atomic' mode.\n");
    }
}

void
AtomicSimpleCPU::activateContext(ThreadID thread_num)
{
    DPRINTF(SimpleCPU, "ActivateContext %d\n", thread_num);

    assert(thread_num < numThreads);

    threadInfo[thread_num]->notIdleFraction = 1;
    Cycles delta = ticksToCycles(threadInfo[thread_num]->thread->lastActivate -
                                 threadInfo[thread_num]->thread->lastSuspend);
    numCycles += delta;

    if (!tickEvent.scheduled()) {
        //Make sure ticks are still on multiples of cycles
        schedule(tickEvent, clockEdge(Cycles(0)));
    }
    _status = BaseSimpleCPU::Running;
    if (std::find(activeThreads.begin(), activeThreads.end(), thread_num)
        == activeThreads.end()) {
        activeThreads.push_back(thread_num);
    }

    BaseCPU::activateContext(thread_num);
}


void
AtomicSimpleCPU::suspendContext(ThreadID thread_num)
{
    DPRINTF(SimpleCPU, "SuspendContext %d\n", thread_num);

    assert(thread_num < numThreads);
    activeThreads.remove(thread_num);

    if (_status == Idle)
        return;

    assert(_status == BaseSimpleCPU::Running);

    threadInfo[thread_num]->notIdleFraction = 0;

    if (activeThreads.empty()) {
        _status = Idle;

        if (tickEvent.scheduled()) {
            deschedule(tickEvent);
        }
    }

    BaseCPU::suspendContext(thread_num);
}

Tick
AtomicSimpleCPU::sendPacket(MasterPort &port, const PacketPtr &pkt)
{
    return port.sendAtomic(pkt);
}

Tick
AtomicSimpleCPU::AtomicCPUDPort::recvAtomicSnoop(PacketPtr pkt)
{
    DPRINTF(SimpleCPU, "received snoop pkt for addr:%#x %s\n", pkt->getAddr(),
            pkt->cmdString());

    // X86 ISA: Snooping an invalidation for monitor/mwait
    AtomicSimpleCPU *cpu = (AtomicSimpleCPU *)(&owner);

    for (ThreadID tid = 0; tid < cpu->numThreads; tid++) {
        if (cpu->getCpuAddrMonitor(tid)->doMonitor(pkt)) {
            cpu->wakeup(tid);
        }
    }

    // if snoop invalidates, release any associated locks
    // When run without caches, Invalidation packets will not be received
    // hence we must check if the incoming packets are writes and wakeup
    // the processor accordingly
    if (pkt->isInvalidate() || pkt->isWrite()) {
        DPRINTF(SimpleCPU, "received invalidation for addr:%#x\n",
                pkt->getAddr());
        for (auto &t_info : cpu->threadInfo) {
            TheISA::handleLockedSnoop(t_info->thread, pkt, cacheBlockMask);
        }
    }

    return 0;
}

void
AtomicSimpleCPU::AtomicCPUDPort::recvFunctionalSnoop(PacketPtr pkt)
{
    DPRINTF(SimpleCPU, "received snoop pkt for addr:%#x %s\n", pkt->getAddr(),
            pkt->cmdString());

    // X86 ISA: Snooping an invalidation for monitor/mwait
    AtomicSimpleCPU *cpu = (AtomicSimpleCPU *)(&owner);
    for (ThreadID tid = 0; tid < cpu->numThreads; tid++) {
        if (cpu->getCpuAddrMonitor(tid)->doMonitor(pkt)) {
            cpu->wakeup(tid);
        }
    }

    // if snoop invalidates, release any associated locks
    if (pkt->isInvalidate()) {
        DPRINTF(SimpleCPU, "received invalidation for addr:%#x\n",
                pkt->getAddr());
        for (auto &t_info : cpu->threadInfo) {
            TheISA::handleLockedSnoop(t_info->thread, pkt, cacheBlockMask);
        }
    }
}

Fault
AtomicSimpleCPU::readMem(Addr addr, uint8_t * data, unsigned size,
                         Request::Flags flags)
{
    SimpleExecContext& t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    // use the CPU's statically allocated read request and packet objects
    const RequestPtr &req = data_read_req;

    if (traceData)
        traceData->setMem(addr, size, flags);

    //The size of the data we're trying to read.
    int fullSize = size;

    //The address of the second part of this access if it needs to be split
    //across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, cacheLineSize());

    if (secondAddr > addr)
        size = secondAddr - addr;

    dcache_latency = 0;

    req->taskId(taskId());
    curStaticInst->atomic_vaddr = addr;

    while (1) {
        req->setVirt(0, addr, size, flags, dataMasterId(), thread->pcState().instAddr());

        // translate to physical address
        Fault fault = thread->dtb->translateAtomic(req, thread->getTC(),
                                                          BaseTLB::Read);

        // Now do the access.
        if (fault == NoFault && !req->getFlags().isSet(Request::NO_ACCESS)) {
            Packet pkt(req, Packet::makeReadCmd(req));
            pkt.dataStatic(data);

            if (req->isMmappedIpr()) {
                dcache_latency += TheISA::handleIprRead(thread->getTC(), &pkt);
            } else {
                dcache_latency += sendPacket(dcachePort, &pkt);
            }
            dcache_access = true;

            assert(!pkt.isError());

            if (req->isLLSC()) {
                TheISA::handleLockedRead(thread, req);
            }
        }

        //If there's a fault, return it
        if (fault != NoFault) {
            if (req->isPrefetch()) {
                return NoFault;
            } else {
                return fault;
            }
        }

        //If we don't need to access a second cache line, stop now.
        if (secondAddr <= addr)
        {
            if (req->isLockedRMW() && fault == NoFault) {
                assert(!locked);
                locked = true;
            }

            return fault;
        }

        /*
         * Set up for accessing the second cache line.
         */

        //Move the pointer we're reading into to the correct location.
        data += size;
        //Adjust the size to get the remaining bytes.
        size = addr + fullSize - secondAddr;
        //And access the right address.
        addr = secondAddr;
    }
}

Fault
AtomicSimpleCPU::initiateMemRead(Addr addr, unsigned size,
                                 Request::Flags flags)
{
    panic("initiateMemRead() is for timing accesses, and should "
          "never be called on AtomicSimpleCPU.\n");
}

Fault
AtomicSimpleCPU::writeMem(uint8_t *data, unsigned size, Addr addr,
                          Request::Flags flags, uint64_t *res)
{
    SimpleExecContext& t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;
    static uint8_t zero_array[64] = {};

    if (data == NULL) {
        assert(size <= 64);
        assert(flags & Request::STORE_NO_DATA);
        // This must be a cache block cleaning request
        data = zero_array;
    }

    // use the CPU's statically allocated write request and packet objects
    const RequestPtr &req = data_write_req;

    if (traceData)
        traceData->setMem(addr, size, flags);

    //The size of the data we're trying to read.
    int fullSize = size;

    //The address of the second part of this access if it needs to be split
    //across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, cacheLineSize());

    if (secondAddr > addr)
        size = secondAddr - addr;

    dcache_latency = 0;

    req->taskId(taskId());
    while (1) {
        req->setVirt(0, addr, size, flags, dataMasterId(), thread->pcState().instAddr());

        // translate to physical address
        Fault fault = thread->dtb->translateAtomic(req, thread->getTC(), BaseTLB::Write);

        // Now do the access.
        if (fault == NoFault) {
            bool do_access = true;  // flag to suppress cache access
            curStaticInst->atomic_vaddr = addr;

            if (req->isLLSC()) {
                do_access = TheISA::handleLockedWrite(thread, req, dcachePort.cacheBlockMask);
            } else if (req->isSwap()) {
                if (req->isCondSwap()) {
                    assert(res);
                    req->setExtraData(*res);
                }
            }

            if (do_access && !req->getFlags().isSet(Request::NO_ACCESS)) {
                Packet pkt(req, Packet::makeWriteCmd(req));
                pkt.dataStatic(data);

                if (req->isMmappedIpr()) {
                    dcache_latency +=
                        TheISA::handleIprWrite(thread->getTC(), &pkt);
                } else {
                    dcache_latency += sendPacket(dcachePort, &pkt);

                    // Notify other threads on this CPU of write
                    threadSnoop(&pkt, curThread);
                }
                dcache_access = true;
                assert(!pkt.isError());

                if (req->isSwap()) {
                    assert(res);
                    memcpy(res, pkt.getConstPtr<uint8_t>(), fullSize);
                }
            }

            if (res && !req->isSwap()) {
                *res = req->getExtraData();
            }
        }

        //If there's a fault or we don't need to access a second cache line,
        //stop now.
        if (fault != NoFault || secondAddr <= addr)
        {
            if (req->isLockedRMW() && fault == NoFault) {
                assert(locked);
                locked = false;
            }


            if (fault != NoFault && req->isPrefetch()) {
                return NoFault;
            } else {
                return fault;
            }
        }

        /*
         * Set up for accessing the second cache line.
         */

        //Move the pointer we're reading into to the correct location.
        data += size;
        //Adjust the size to get the remaining bytes.
        size = addr + fullSize - secondAddr;
        //And access the right address.
        addr = secondAddr;
    }
}


void
AtomicSimpleCPU::tick()
{

    #define ENABLE_LOGGING 0
    DPRINTF(SimpleCPU, "Tick\n");

    // Change thread if multi-threaded
    swapActiveThread();

    // Set memroy request ids to current thread
    if (numThreads > 1) {
        ContextID cid = threadContexts[curThread]->contextId();

        ifetch_req->setContext(cid);
        data_read_req->setContext(cid);
        data_write_req->setContext(cid);
    }

    SimpleExecContext& t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    Tick latency = 0;

    for (int i = 0; i < width || locked; ++i) {
        numCycles++;
        updateCycleCounters(BaseCPU::CPU_STATE_ON);

        if (!curStaticInst || !curStaticInst->isDelayedCommit()) {
            checkForInterrupts();
            checkPcEventQueue();
        }

        // We must have just got suspended by a PC event
        if (_status == Idle) {
            tryCompleteDrain();
            return;
        }

        Fault fault = NoFault;

        TheISA::PCState pcState = thread->pcState();

        bool needToFetch = !isRomMicroPC(pcState.microPC()) &&
                           !curMacroStaticInst;
        if (needToFetch) {
            ifetch_req->taskId(taskId());
            setupFetchRequest(ifetch_req);
            fault = thread->itb->translateAtomic(ifetch_req, thread->getTC(),
                                                 BaseTLB::Execute);
        }

        if (fault == NoFault) {
            Tick icache_latency = 0;
            bool icache_access = false;
            dcache_access = false; // assume no dcache access

            if (needToFetch) {
                // This is commented out because the decoder would act like
                // a tiny cache otherwise. It wouldn't be flushed when needed
                // like the I cache. It should be flushed, and when that works
                // this code should be uncommented.
                //Fetch more instruction memory if necessary
                //if (decoder.needMoreBytes())
                //{
                    icache_access = true;
                    Packet ifetch_pkt = Packet(ifetch_req, MemCmd::ReadReq);
                    ifetch_pkt.dataStatic(&inst);

                    icache_latency = sendPacket(icachePort, &ifetch_pkt);

                    assert(!ifetch_pkt.isError());

                    // ifetch_req is initialized to read the instruction directly
                    // into the CPU object's inst field.
                //}
            }

            preExecute();

            Tick stall_ticks = 0;
            if (curStaticInst) {
                fault = curStaticInst->execute(&t_info, traceData);

                SimpleExecContext& t_info = *threadInfo[0];
                SimpleThread* thread = t_info.thread;

              if (threadContexts[0]->enableCapability && fault == NoFault){
                trackAlias(pcState);
              }

              if (threadContexts[0]->enableCapability && fault == NoFault){
                  if (curStaticInst->isFirstMicroop())
                  {

                    auto syms_it =
                      (threadContexts[0]->syms_cache).find(pcState.instAddr());
                    if (syms_it != (threadContexts[0]->syms_cache).end()){
                      collector(threadContexts[0], pcState, syms_it->second);
                    }
                  }
              }

              // if (threadContexts[0]->enableCapability && fault == NoFault){
              //     UpdatePointerTracker(threadContexts[0],pcState);
              // }

              // if (threadContexts[0]->enableCapability && fault == NoFault
              //     ){
              //   UpdatePointerTrackerSpeculative(threadContexts[0],pcState);
              //   ComparePointerTrackerSpeculative(threadContexts[0],pcState);
              // }

              // if (threadContexts[0]->enableCapability && fault == NoFault){
              //       if (curStaticInst->isStore() &&
              //           curStaticInst->getDataSize() == 8)
              //       {
              //         // if (ENABLE_LOGGING)
              //          updateAliasTableWithStack(threadContexts[0],pcState);
              //         // else
              //         //    updateAliasTable(threadContexts[0],pcState);
              //       }
              // }
              // if (threadContexts[0]->enableCapability &&
              //     fault == NoFault &&
              //     (curStaticInst->isLoad() || curStaticInst->isStore())
              //     )
              // {
              //     AccessCapabilityCache(threadContexts[0], pcState);
              // }

              // if (threadContexts[0]->enableCapability && fault == NoFault){
              //   Verify(threadContexts[0],pcState);
              // }

              if (threadContexts[0]->enableCapability && fault == NoFault){
                    if (curStaticInst->isLoad() &&
                        curStaticInst->getDataSize() == 8 &&
                        threadContexts[0]->InSlice)
                    {
                        WarmupAliasTable(threadContexts[0],pcState);
                    }
                }


                if (ENABLE_LOGGING)
                  if (threadContexts[0]->enableCapability && fault == NoFault){
                      if (curStaticInst->isLoad() &&
                          curStaticInst->getDataSize() == 8)
                      {
                         getLog(threadContexts[0],pcState);
                      }
                  }

                if (ENABLE_LOGGING){
                  if (((uint64_t)t_info.numInsts.value() ==
                      max_insts_any_thread - 1) &&
                      curStaticInst->isLastMicroop()){

                    for (auto& elem : debug_function_calls){
                        std::cout << elem.first << ": " <<
                                     elem.second << std::endl;
                    }
                  }
                }
                // dump stats

                if (threadContexts[0]->enableCapability &&
                    fault == NoFault && curStaticInst->isLastMicroop() &&
                    ((uint64_t)t_info.numInsts.value() % 1000000 == 0))
                {
                    std::cout << std::dec << t_info.numInsts.value() << " " <<
                              threadContexts[0]->num_of_allocations << " " <<
                              threadContexts[0]->ShadowMemory.size() << "\n";
                          // numOfMemRefs << " " << numOfHeapAccesses << " ";
                    // threadContexts[0]->LRUPidCache.LRUPIDCachePrintStats();
                     //numOfMemRefs = 0; numOfHeapAccesses = 0;

                     // int LV1Size = AliasPageTable.size();
                     //
                     // int LV2Size = 0, LV3Size = 0, LV4Size = 0,
                     //     LV5Size = 0, LV6Size = 0, LV7Size = 0;
                     //
                     // for (auto &lv1_elem: AliasPageTable){
                     //    LV2Size += lv1_elem.second.size();
                     //    for (auto &lv2_elem: lv1_elem.second){
                     //       LV3Size += lv2_elem.second.size();
                     //       for (auto &lv3_elem: lv2_elem.second){
                     //          LV4Size += lv3_elem.second.size();
                     //          for (auto &lv4_elem: lv3_elem.second){
                     //             LV5Size += lv4_elem.second.size();
                     //             for (auto &lv5_elem: lv4_elem.second){
                     //               LV6Size += lv5_elem.second.size();
                     //               for (auto &lv6_elem: lv5_elem.second){
                     //                 LV7Size += lv6_elem.second.size();
                     //               }
                     //             }
                     //          }
                     //       }
                     //    }
                     // }
                     //
                     // std::cout << " AliasPageTable: " << std::dec <<
                     //              "LV1: " << LV1Size << " " <<
                     //              "LV2: " << LV2Size << " " <<
                     //              "LV3: " << LV3Size << " " <<
                     //              "LV4: " << LV4Size << " " <<
                     //              "LV5: " << LV5Size << " " <<
                     //              "LV6: " << LV6Size << " " <<
                     //              "LV7: " << LV7Size << " " <<
                     //              std::endl;
                }

                if (fault == NoFault) {
                    countInst();
                    ppCommit->notify(std::make_pair(thread, curStaticInst));
                }
                else if (traceData && !DTRACE(ExecFaulting)) {
                    delete traceData;
                    traceData = NULL;
                }

                if (fault != NoFault &&
                    dynamic_pointer_cast<SyscallRetryFault>(fault)) {
                    // Retry execution of system calls after a delay.
                    // Prevents immediate re-execution since conditions which
                    // caused the retry are unlikely to change every tick.
                    stall_ticks += clockEdge(syscallRetryLatency) - curTick();
                }

                postExecute();
            }

            // @todo remove me after debugging with legion done
            if (curStaticInst && (!curStaticInst->isMicroop() ||
                        curStaticInst->isFirstMicroop()))
                instCnt++;

            if (simulate_inst_stalls && icache_access)
                stall_ticks += icache_latency;

            if (simulate_data_stalls && dcache_access)
                stall_ticks += dcache_latency;

            if (stall_ticks) {
                // the atomic cpu does its accounting in ticks, so
                // keep counting in ticks but round to the clock
                // period
                latency += divCeil(stall_ticks, clockPeriod()) *
                    clockPeriod();
            }

        }
        if (fault != NoFault || !t_info.stayAtPC)
            advancePC(fault);
    }

    if (tryCompleteDrain())
        return;

    // instruction takes at least one cycle
    if (latency < clockPeriod())
        latency = clockPeriod();

    if (_status != Idle)
        reschedule(tickEvent, curTick() + latency, true);
}

void
AtomicSimpleCPU::regProbePoints()
{
    BaseCPU::regProbePoints();

    ppCommit = new ProbePointArg<pair<SimpleThread*, const StaticInstPtr>>
                                (getProbeManager(), "Commit");
}

void
AtomicSimpleCPU::printAddr(Addr a)
{
    dcachePort.printAddr(a);
}

////////////////////////////////////////////////////////////////////////
//
//  AtomicSimpleCPU Simulation Object
//
AtomicSimpleCPU *
AtomicSimpleCPUParams::create()
{
    return new AtomicSimpleCPU(this);
}


void
AtomicSimpleCPU::collector(ThreadContext * _tc,
                         PCState &pcState,
                         TheISA::CheckType _sym){

    #define ATOMIC_CPU_COLLECTOR_DEBUG 0

    SimpleExecContext& t_info = *threadInfo[0];
    SimpleThread* thread = t_info.thread;

    if (_sym == TheISA::CheckType::AP_MALLOC_SIZE_COLLECT){

      if (threadContexts[0]->Collector_Status !=
          ThreadContext::COLLECTOR_STATUS::NONE)
      {
          std::cout << "PRE STATE: " << threadContexts[0]->Collector_Status <<
            " " << pcState <<
            std::endl;
         panic("AP_MALLOC_SIZE_COLLECT: Invalid Status!");
      }

      threadContexts[0]->ap_size = thread->readIntReg(X86ISA::INTREG_RDI);

      threadContexts[0]->Collector_Status =
                ThreadContext::COLLECTOR_STATUS::MALLOC_SIZE;
      // logs
      if (ATOMIC_CPU_COLLECTOR_DEBUG)
      { std::cout << "AP_MALLOC_SIZE_COLLECT: " << pcState <<
               " " << curStaticInst->disassemble(pcState.pc()) << " Size: " <<
               std::hex << threadContexts[0]->ap_size << std::endl;
      }

    }
    else if (_sym == TheISA::CheckType::AP_MALLOC_BASE_COLLECT){

      if (threadContexts[0]->Collector_Status !=
                            ThreadContext::COLLECTOR_STATUS::MALLOC_SIZE)
          panic("AP_MALLOC_BASE_COLLECT: Invalid Status!");

      threadContexts[0]->ap_base = thread->readIntReg(X86ISA::INTREG_RAX);

      threadContexts[0]->Collector_Status =
            ThreadContext::COLLECTOR_STATUS::NONE;
      threadContexts[0]->num_of_allocations++;

      Block* bk = static_cast<Block*>(malloc(sizeof(Block)));
      bk->payload   = (Addr)threadContexts[0]->ap_base;
      bk->req_szB   = (SizeT)threadContexts[0]->ap_size;
      bk->pid       = (Addr)++threadContexts[0]->AtomicPID;
      unsigned char present =
              VG_addToFM( _tc->interval_tree, (UWord)bk, (UWord)0);
      assert(!present);
      // logs
      if (ATOMIC_CPU_COLLECTOR_DEBUG)
      { std::cout << "AP_MALLOC_BASE_COLLECT: " << pcState <<
                  " " << curStaticInst->disassemble(pcState.pc()) <<
                  " Base: " << std::hex <<
                  threadContexts[0]->ap_base << std::endl;
      }
      _tc->PointerTracker[X86ISA::INTREG_RAX] = bk->pid;

    }
    else if (_sym == TheISA::CheckType::AP_CALLOC_SIZE_COLLECT){

      if (threadContexts[0]->Collector_Status !=
          ThreadContext::COLLECTOR_STATUS::NONE)
      {
         panic("AP_CALLOC_SIZE_COLLECT: Invalid Status!");
      }

      threadContexts[0]->ap_size = thread->readIntReg(X86ISA::INTREG_RDI) *
                        thread->readIntReg(X86ISA::INTREG_RSI);
      threadContexts[0]->Collector_Status =
              ThreadContext::COLLECTOR_STATUS::CALLOC_SIZE;
      // logs
      if (ATOMIC_CPU_COLLECTOR_DEBUG)
      { std::cout << "AP_CALLOC_SIZE_COLLECT: " << pcState <<
               " " << curStaticInst->disassemble(pcState.pc()) << " Size: " <<
               std::hex << threadContexts[0]->ap_size << std::endl;
      }

    }
    else if (_sym == TheISA::CheckType::AP_CALLOC_BASE_COLLECT){

       if (threadContexts[0]->Collector_Status !=
                        ThreadContext::COLLECTOR_STATUS::CALLOC_SIZE)
          panic("AP_CALLOC_BASE_COLLECT: Invalid Status!");

       threadContexts[0]->ap_base = thread->readIntReg(X86ISA::INTREG_RAX);

       threadContexts[0]->Collector_Status =
              ThreadContext::COLLECTOR_STATUS::NONE;
       threadContexts[0]->num_of_allocations++;
       // logs
       if (ATOMIC_CPU_COLLECTOR_DEBUG)
       { std::cout << "AP_CALLOC_BASE_COLLECT: " << pcState <<
                   " " << curStaticInst->disassemble(pcState.pc()) <<
                   " Base: " << std::hex <<
                   threadContexts[0]->ap_base << std::endl;
       }

       Block* bk = static_cast<Block*>(malloc(sizeof(Block)));
       bk->payload   = (Addr)threadContexts[0]->ap_base;
       bk->req_szB   = (SizeT)threadContexts[0]->ap_size;
       bk->pid       = (Addr)++threadContexts[0]->AtomicPID;
       unsigned char present =
                  VG_addToFM( _tc->interval_tree, (UWord)bk, (UWord)0);


       if (present){
         if (ATOMIC_CPU_COLLECTOR_DEBUG){
             UWord keyW, valW;
             VG_initIterFM( _tc->interval_tree );
             while (VG_nextIterFM( _tc->interval_tree, &keyW, &valW )) {
                Block* bk = (Block*)keyW;
                assert(valW == 0);
                assert(bk);
                std::cout << "INTERVAL_TREE: " <<"Base: " <<
                            std::hex << bk->payload << std::dec <<
                            " Size: " << bk->req_szB <<" PID: " <<
                            bk->pid << std::endl;
              }
             VG_doneIterFM( _tc->interval_tree );
         }
         assert(!present);
       }

    }
    else if (_sym == TheISA::CheckType::AP_FREE_CALL){

      uint64_t base_addr = thread->readIntReg(X86ISA::INTREG_RDI);
      // logs
      if (ATOMIC_CPU_COLLECTOR_DEBUG)
      { std::cout << "AP_FREE_CALL: " << pcState <<
                  " " << curStaticInst->disassemble(pcState.pc()) <<
                  " FOUND: " <<  std::hex << base_addr << std::endl;
      }

      Block fake;
      fake.payload = base_addr;
      fake.req_szB = 1;
      UWord foundkey = 1;
      UWord foundval = 1;
      unsigned char present = VG_lookupFM( _tc->interval_tree,
                                  &foundkey, &foundval, (UWord)&fake );

      Block* bk = NULL;
      if (present){
          Block fake;
          fake.payload = base_addr;
          fake.req_szB = 1;
          UWord oldKeyW;
          unsigned char found = VG_delFromFM( _tc->interval_tree,
                                   &oldKeyW, NULL, (Addr)&fake );
          bk = (Block*)oldKeyW;
          assert(bk);
          assert(bk->pid != 0);
          assert(found);
        //  _tc->freedPIDVector.push_back(bk->pid);

          if (ENABLE_LOGGING){ // delete all aliases related to this PID

              for (auto it_lv1 = threadContexts[0]->ShadowMemory.begin(),
                        next_it_lv1 = it_lv1;
                        it_lv1 != threadContexts[0]->ShadowMemory.end();
                        it_lv1 = next_it_lv1)
              {
                  ++next_it_lv1;
                  if (it_lv1->second.size() == 0)
                  {
                      threadContexts[0]->ShadowMemory.erase(it_lv1);
                  }
                  else {
                      for (auto it_lv2 = it_lv1->second.cbegin(),
                           next_it_lv2 = it_lv2;
                           it_lv2 != it_lv1->second.cend();
                           it_lv2 = next_it_lv2)
                      {
                        ++next_it_lv2;
                        if (it_lv2->second.getPID() == bk->pid)
                        {
                          it_lv1->second.erase(it_lv2);
                        }
                      }
                  }
              }

          }

          free(bk);
          threadContexts[0]->num_of_allocations--;
      }

    }
    else if (_sym == TheISA::CheckType::AP_FREE_RET){

      //threadContexts[0]->Collector_Status =
      //              ThreadContext::COLLECTOR_STATUS::NONE;

    }
    else if (_sym == TheISA::CheckType::AP_REALLOC_SIZE_COLLECT){

      if (threadContexts[0]->Collector_Status !=
          ThreadContext::COLLECTOR_STATUS::NONE)
          panic("AP_REALLOC_SIZE_COLLECT: Invalid Status!");

      threadContexts[0]->ap_size = thread->readIntReg(X86ISA::INTREG_RSI);
      threadContexts[0]->Collector_Status =
              ThreadContext::COLLECTOR_STATUS::REALLOC_SIZE;
      uint64_t old_base_addr = thread->readIntReg(X86ISA::INTREG_RDI);

      // logs
      if (ATOMIC_CPU_COLLECTOR_DEBUG)
      { std::cout << "AP_REALLOC_SIZE_COLLECT: " << pcState <<
              " " << curStaticInst->disassemble(pcState.pc()) << " Size: " <<
              std::hex << threadContexts[0]->ap_size << " Old Base Addr.: " <<
              old_base_addr << std::endl;
      }

      Block fake;
      fake.payload = old_base_addr;
      fake.req_szB = 1;
      UWord foundkey = 1;
      UWord foundval = 1;
      unsigned char present = VG_lookupFM( _tc->interval_tree,
                                  &foundkey, &foundval, (UWord)&fake );
      if (present){
          Block fake;
          fake.payload = old_base_addr;
          fake.req_szB = 1;
          UWord oldKeyW;
          unsigned char found = VG_delFromFM( _tc->interval_tree,
                                   &oldKeyW, NULL, (Addr)&fake );
          Block* bk = (Block*)oldKeyW;
          assert(bk);
          assert(found);
          //_tc->freedPIDVector.push_back(bk->pid);

          if (ENABLE_LOGGING){ // delete all aliases related to this PID

              for (auto it_lv1 = threadContexts[0]->ShadowMemory.begin(),
                        next_it_lv1 = it_lv1;
                        it_lv1 != threadContexts[0]->ShadowMemory.end();
                        it_lv1 = next_it_lv1)
              {
                  ++next_it_lv1;
                  if (it_lv1->second.size() == 0)
                  {
                      threadContexts[0]->ShadowMemory.erase(it_lv1);
                  }
                  else {
                      for (auto it_lv2 = it_lv1->second.cbegin(),
                           next_it_lv2 = it_lv2;
                           it_lv2 != it_lv1->second.cend();
                           it_lv2 = next_it_lv2)
                      {
                        ++next_it_lv2;
                        if (it_lv2->second.getPID() == bk->pid)
                        {
                          it_lv1->second.erase(it_lv2);
                        }
                      }
                  }
              }

          }
          free(bk);
          threadContexts[0]->num_of_allocations--;
      }

    }
    else if (_sym == TheISA::CheckType::AP_REALLOC_BASE_COLLECT){

      if (threadContexts[0]->Collector_Status !=
                        ThreadContext::COLLECTOR_STATUS::REALLOC_SIZE)
        panic("AP_REALLOC_BASE_COLLECT: Invalid Status!");

      threadContexts[0]->Collector_Status =
                  ThreadContext::COLLECTOR_STATUS::NONE;
      threadContexts[0]->ap_base = thread->readIntReg(X86ISA::INTREG_RAX);
      threadContexts[0]->num_of_allocations++;
      // logs
      if (ATOMIC_CPU_COLLECTOR_DEBUG)
      { std::cout << "AP_REALLOC_BASE_COLLECT: " << pcState <<
                  " " << curStaticInst->disassemble(pcState.pc()) <<
                  " Base: " << std::hex <<
                  threadContexts[0]->ap_base << std::endl;
      }

      Block* bk = static_cast<Block*>(malloc(sizeof(Block)));
      bk->payload   = (Addr)threadContexts[0]->ap_base;
      bk->req_szB   = (SizeT)threadContexts[0]->ap_size;
      bk->pid       = (Addr)++threadContexts[0]->AtomicPID;

      unsigned char present =
                VG_addToFM( _tc->interval_tree, (UWord)bk, (UWord)0);


      if (present){
        if (ATOMIC_CPU_COLLECTOR_DEBUG){
            UWord keyW, valW;
            VG_initIterFM( _tc->interval_tree );
            while (VG_nextIterFM( _tc->interval_tree, &keyW, &valW )) {
               Block* bk = (Block*)keyW;
               assert(valW == 0);
               assert(bk);
               std::cout << "INTERVAL_TREE: " <<"Base: " <<
                           std::hex << bk->payload << std::dec <<
                           " Size: " << bk->req_szB <<" PID: " <<
                           bk->pid << std::endl;
             }
            VG_doneIterFM( _tc->interval_tree );
        }
        assert(!present);
      }
    }

}


void AtomicSimpleCPU::AccessCapabilityCache(ThreadContext * tc,
                         PCState &pcState)
{
  SimpleExecContext& t_info = *threadInfo[0];
  SimpleThread* thread = t_info.thread;

  if (thread->stop_tracking) return;

  assert(curStaticInst->atomic_vaddr != 0);
  // first find the page vpn and store into the page cluster
  Block* bk = find_Block_containing(curStaticInst->atomic_vaddr);

  numOfMemRefs++;
  // if found this is a heap MemRef
  if (bk) {
    assert(bk->pid != 0);
    numOfHeapAccesses++;
    //access the capability cache
    tc->LRUPidCache.LRUPIDCache_Access(bk->pid);
  }


}

void AtomicSimpleCPU::updateAliasTable(ThreadContext * _tc,
                         PCState &pcState)
{
    #define ATOMIC_UPDATE_ALIAS_TABLE 0
    SimpleExecContext& t_info = *threadInfo[0];
    SimpleThread* thread = t_info.thread;

    if (thread->stop_tracking) return;

    // srcRegIdx(2) in store microops is the src
    // check whther it is an integer reg or not
    if (!curStaticInst->srcRegIdx(2).isIntReg()) return;

    // if segnemt reg is SS then return because we c dont care about stack
    // aliases in this mode. in fact stis microops are discarded
    if (curStaticInst->getSegment() == TheISA::SEGMENT_REG_SS) return;

    // ignore all stack aliases as they are temporary
    // we dont need to store them
    // eventually they will get removed from alias table
    // new code
    Addr stack_base = 0x7FFFFFFFF000ULL;
    Addr max_stack_size = 32 * 1024 * 1024;
    Addr next_thread_stack_base = stack_base - max_stack_size;
    // as we check for NoFault, then defenitly atomic_vaddr is valid
    if ((curStaticInst->atomic_vaddr >= next_thread_stack_base &&
          curStaticInst->atomic_vaddr <= stack_base))
          return; // this is a stack address, return

    // make sure the addr is between heap range


    Addr vaddr = thread->readIntReg(curStaticInst->getMemOpDataRegIndex());
    // first find the page vpn and store into the page cluster
    Block* bk = find_Block_containing(vaddr);


    Process* p = threadContexts[0]->getProcessPtr();
    assert(curStaticInst->atomic_vaddr != 0);
    Addr vpn = p->pTable->pageAlign(curStaticInst->atomic_vaddr);

    // if found: update the ShadowMemory
    if (bk) { // just the base addresses
      assert(bk->pid != 0);
      threadContexts[0]->ShadowMemory[vpn][curStaticInst->atomic_vaddr] =
                                                                      bk->pid;
      if (ATOMIC_UPDATE_ALIAS_TABLE) {
        std::cout << curStaticInst->disassemble(pcState.pc()) << " " <<
                   std::hex <<
                   thread->readIntReg(curStaticInst->getMemOpDataRegIndex()) <<
                   std::hex << " (" << bk->payload << "," <<
                   bk->payload + bk->req_szB << ") = " <<
                   std::dec << "PID: " << bk->pid <<
                   std::endl;
      }
    }
    else {
      // if not found in the capability cache, then check if the alias is
      // overwritten
      auto it_lv1 = threadContexts[0]->ShadowMemory.find(vpn);
      if (it_lv1 != threadContexts[0]->ShadowMemory.end()){
         auto it_lv2 = it_lv1->second.find(curStaticInst->atomic_vaddr);
         if (it_lv2 != it_lv1->second.end()){
            it_lv1->second.erase(it_lv2);
         }
         if (it_lv1->second.empty()){
           threadContexts[0]->ShadowMemory.erase(it_lv1);
         }
      }

    }


}

void AtomicSimpleCPU::WarmupAliasTable(ThreadContext * _tc,
                         PCState &pcState)
{
    #define ATOMIC_WARMUP_ALIAS_TABLE 0
    SimpleExecContext& t_info = *threadInfo[0];
    SimpleThread* thread = t_info.thread;

    if (thread->stop_tracking) return;
    // ignore all stack aliases as they are temporary
    // we dont need to store them
    // eventually they will get removed from alias table
    // new code
    assert(curStaticInst->atomic_vaddr != 0);

    // make sure the addr is between heap range

    if (!curStaticInst->destRegIdx(0).isIntReg()) return;

    Addr vaddr = thread->readIntReg(curStaticInst->destRegIdx(0).index());
    // first find the page vpn and store into the page cluster
    Block* bk = find_Block_containing(vaddr);


    Process* p = threadContexts[0]->getProcessPtr();

    Addr vpn = p->pTable->pageAlign(curStaticInst->atomic_vaddr);

    // if found: update the ShadowMemory
    if (bk) { // just the base addresses
      assert(bk->pid != 0);
      threadContexts[0]->ShadowMemory[vpn][curStaticInst->atomic_vaddr] =
                                                                      bk->pid;
      if (ATOMIC_WARMUP_ALIAS_TABLE) {
        std::cout << curStaticInst->disassemble(pcState.pc()) << " " <<
                   std::hex <<
                   thread->readIntReg(curStaticInst->getMemOpDataRegIndex()) <<
                   std::hex << " (" << bk->payload << "," <<
                   bk->payload + bk->req_szB << ") = " <<
                   std::dec << "PID: " << bk->pid <<
                   std::endl;
      }
    }
    else {
      // if not found in the capability cache, then check if the alias is
      // overwritten
      auto it_lv1 = threadContexts[0]->ShadowMemory.find(vpn);
      if (it_lv1 != threadContexts[0]->ShadowMemory.end()){
         auto it_lv2 = it_lv1->second.find(curStaticInst->atomic_vaddr);
         if (it_lv2 != it_lv1->second.end()){
            it_lv1->second.erase(it_lv2);
         }
         if (it_lv1->second.empty()){
           threadContexts[0]->ShadowMemory.erase(it_lv1);
         }
      }

    }


}
// this function is slow and gatters unnecesaary information and only should
// be used with getLog function in debuging mode
void AtomicSimpleCPU::updateAliasTableWithStack(ThreadContext * _tc,
                         PCState &pcState)
{
    #define ATOMIC_UPDATE_ALIAS_TABLE_WITH_STACK 0
    SimpleExecContext& t_info = *threadInfo[0];
    SimpleThread* thread = t_info.thread;

    if (thread->stop_tracking) return;

    // srcRegIdx(2) in store microops is the src
    // check whther it is an integer reg or not
    if (!curStaticInst->srcRegIdx(2).isIntReg()) return;

    // if (!(curStaticInst->getSegment() == TheISA::SEGMENT_REG_DS ||
    //     curStaticInst->getSegment() == TheISA::SEGMENT_REG_SS)) return;

    // first delete all the aliases with the updated rsp
    uint64_t RSPValue = thread->readIntReg(X86ISA::INTREG_RSP);
    Addr stack_base = 0x7FFFFFFFF000ULL;
    Addr max_stack_size = 32 * 1024 * 1024;
    Addr next_thread_stack_base = stack_base - max_stack_size;

    // rsp val is between program stack
    if ((RSPValue >= next_thread_stack_base && RSPValue <= stack_base)){
      // two level removal of stack aliases between RSPValue and
      // next_thread_stack_base
      Process *p = threadContexts[0]->getProcessPtr();
      Addr vpn = p->pTable->pageAlign(RSPValue);
      auto it_lv1 = threadContexts[0]->ShadowMemory.find(vpn);
      if (it_lv1 != threadContexts[0]->ShadowMemory.end()){
          if (it_lv1->second.size() == 0){
            threadContexts[0]->ShadowMemory.erase(it_lv1);
          }
          else {
              for (auto it = it_lv1->second.cbegin(), next_it = it;
                    it != it_lv1->second.cend(); it = next_it)
              {
                   ++next_it;
                   if (it->first >= next_thread_stack_base &&
                      it->first <= RSPValue)
                   {
                     it_lv1->second.erase(it);
                   }
              }
          }
      }

    } // end of stack update


    Addr vaddr = thread->readIntReg(curStaticInst->getMemOpDataRegIndex());
    // first find the page vpn and store into the page cluster
    Block* bk = find_Block_containing(vaddr);

    Process* p = threadContexts[0]->getProcessPtr();
    assert(curStaticInst->atomic_vaddr != 0);
    Addr vpn = p->pTable->pageAlign(curStaticInst->atomic_vaddr);

    // if found: update the ShadowMemory
    if (bk) {
      assert(bk->pid != 0);
      threadContexts[0]->ShadowMemory[vpn][curStaticInst->atomic_vaddr] =
                                                                      bk->pid;
      if (ATOMIC_UPDATE_ALIAS_TABLE_WITH_STACK) {
        std::cout << curStaticInst->disassemble(pcState.pc()) << " " <<
                   std::hex <<
                   thread->readIntReg(curStaticInst->getMemOpDataRegIndex()) <<
                   std::hex << " (" << bk->payload << "," <<
                   bk->payload + bk->req_szB << ") = " <<
                   std::dec << "PID: " << bk->pid <<
                   std::endl;
      }

      uint64_t lv1 = curStaticInst->atomic_vaddr & LV1_MASK;
      uint64_t lv2 = curStaticInst->atomic_vaddr & LV2_MASK;
      uint64_t lv3 = curStaticInst->atomic_vaddr & LV3_MASK;
      uint64_t lv4 = curStaticInst->atomic_vaddr & LV4_MASK;
      uint64_t lv5 = curStaticInst->atomic_vaddr & LV5_MASK;
      uint64_t lv6 = curStaticInst->atomic_vaddr & LV6_MASK;
      uint64_t lv7 = curStaticInst->atomic_vaddr & LV7_MASK;

      AliasPageTable[lv1][lv2][lv3][lv4][lv5][lv6][lv7] = bk->pid;
    }
    else {
      // if not found in the capability cache, then check if the alias is
      // overwritten
      auto it_lv1 = threadContexts[0]->ShadowMemory.find(vpn);
      if (it_lv1 != threadContexts[0]->ShadowMemory.end()){
         auto it_lv2 = it_lv1->second.find(curStaticInst->atomic_vaddr);
         if (it_lv2 != it_lv1->second.end()){
            it_lv1->second.erase(it_lv2);
         }
         if (it_lv1->second.empty()){
           threadContexts[0]->ShadowMemory.erase(it_lv1);
         }
      }

    }


}

void AtomicSimpleCPU::getLog(ThreadContext * _tc,
                         PCState &pcState)
{
  SimpleExecContext& t_info = *threadInfo[0];
  SimpleThread* thread = t_info.thread;

  if (thread->stop_tracking) return;

  if (curStaticInst->destRegIdx(0).isIntReg()){

      int  dest = curStaticInst->getMemOpDataRegIndex();
      if (dest > X86ISA::NUM_INTREGS + 15)
          return;

      assert(curStaticInst->atomic_vaddr != 0);

      Process *p = threadContexts[0]->getProcessPtr();
      Addr vpn = p->pTable->pageAlign(curStaticInst->atomic_vaddr);
      auto it_lv1 = threadContexts[0]->ShadowMemory.find(vpn);
      if (it_lv1 != threadContexts[0]->ShadowMemory.end()){
          auto it_lv2 = it_lv1->second.find(curStaticInst->atomic_vaddr);
          if (it_lv2 != it_lv1->second.end()){
             Block bk;
             bk.pid = it_lv2->second.getPID();
             PIDLogs[pcState.pc()].push_back(bk);

             // find the function which loaded a pointer
             Block fake;
             fake.payload = (Addr)pcState.pc();
             fake.req_szB = 1;
             UWord foundkey = 1;
             UWord foundval = 1;
             unsigned char found =
                    VG_lookupFM(threadContexts[0]->FunctionSymbols,
                                         &foundkey, &foundval, (UWord)&fake );
            if (found) {
                Block* bk = (Block*)foundkey;
                debug_function_calls[bk->name]++;
            }
          }
      }

  }

}

void AtomicSimpleCPU::trackAlias(PCState &pcState){

    uint64_t pc = pcState.pc();
    // to use when we have function names
    SimpleExecContext& t_info = *threadInfo[0];
    SimpleThread* thread = t_info.thread;

    Block fake;
    fake.payload = (Addr)pc;
    fake.req_szB = 1;
    UWord foundkey = 1;
    UWord foundval = 1;
    unsigned char found = VG_lookupFM( threadContexts[0]->FunctionSymbols,
                                    &foundkey, &foundval, (UWord)&fake );
    if (found)
    {
      thread->stop_tracking = false;
    }
    else
    {
      thread->stop_tracking = true;
    }

}


Block* AtomicSimpleCPU::find_Block_containing ( Addr vaddr ){

    if (likely(fbc_cache0
                  && fbc_cache0->payload <= vaddr
                  && vaddr < fbc_cache0->payload + fbc_cache0->req_szB))
    {
        return fbc_cache0;
    }

    if (likely(fbc_cache1
                  && fbc_cache1->payload <= vaddr
                  && vaddr < fbc_cache1->payload + fbc_cache1->req_szB))
    {
        // found at 1; swap 0 and 1
        Block* tmp = fbc_cache0;
        fbc_cache0 = fbc_cache1;
        fbc_cache1 = tmp;
        return fbc_cache0;
    }

   Block fake;
   fake.payload = vaddr;
   fake.req_szB = 1;
   UWord foundkey = 1;
   UWord foundval = 1;
   unsigned char found = VG_lookupFM( threadContexts[0]->interval_tree,
                               &foundkey, &foundval, (UWord)&fake );
   if (!found) {
      return NULL;
   }

   assert(foundval == 0); // we don't store vals in the interval tree
   assert(foundkey != 1);
   Block* res = (Block*)foundkey;
   assert(res != &fake);

   return res;
}


void AtomicSimpleCPU::UpdatePointerTracker(ThreadContext * tc,
                         PCState &pcState)
{
    SimpleExecContext& t_info = *threadInfo[0];
    SimpleThread* thread = t_info.thread;

    if (curStaticInst->isLoad() && curStaticInst->getDataSize() == 8)
    {
        // pointer refill
        for (size_t i = 0; i < curStaticInst->numDestRegs(); i++) {
            if (curStaticInst->destRegIdx(i).isIntReg())
            {
                uint64_t dataRegContent =
                    thread->readIntReg(curStaticInst->destRegIdx(i).index());
                Block* dest_bk = find_Block_containing(dataRegContent);
                if (dest_bk){

                    std::cout << pcState;
                    std::cout <<
                      curStaticInst->disassemble(pcState.pc()) << std::endl;
                    std::cout <<
                      "Pointer Refill:" << curStaticInst->destRegIdx(i) <<
                      " {PID(" << dest_bk->pid << ")}" << std::endl;
                }
            }
        }

    }
    else if (curStaticInst->isStore() && curStaticInst->getDataSize() == 8)
    {
        // pointer spill
        if (curStaticInst->getDataSize() == 8) {
            for (size_t i = 0; i < curStaticInst->numSrcRegs(); i++) {
                if (curStaticInst->srcRegIdx(i).isIntReg() &&
                    curStaticInst->srcRegIdx(i).index() ==
                                curStaticInst->getMemOpDataRegIndex()
                    )
                {
                    uint64_t dataRegContent =
                      thread->readIntReg(curStaticInst->srcRegIdx(i).index());
                    Block* src_bk = find_Block_containing(dataRegContent);
                    if (src_bk){
                        std::cout << pcState;
                        std::cout <<
                        curStaticInst->disassemble(pcState.pc()) << std::endl;
                        std::cout <<
                        "Pointer Spill:" << curStaticInst->srcRegIdx(i) <<
                        " {PID(" << src_bk->pid << ")}" << std::endl;
                    }
                }

            }
        }

    }
    else if (curStaticInst->getDataSize() == 8) {

        for (size_t i = 0; i < curStaticInst->numDestRegs(); i++) {
          if (curStaticInst->destRegIdx(i).isIntReg())
          {
              uint64_t dataRegContent =
                    thread->readIntReg(curStaticInst->destRegIdx(i).index());
              Block* dest_bk = find_Block_containing(dataRegContent);
              if (dest_bk){

                  std::cout << pcState;
                  std::cout <<
                      curStaticInst->disassemble(pcState.pc()) << std::endl;

                  for (size_t i = 0; i < curStaticInst->numSrcRegs(); i++) {
                    if (curStaticInst->srcRegIdx(i).isIntReg())
                    {
                      Block* bk_src = find_Block_containing(dataRegContent);
                      if (bk_src){
                        std::cout <<
                                "Src: " << curStaticInst->srcRegIdx(i) <<
                                " {PID(" << dest_bk->pid << ")}" << std::endl;
                      }
                      else
                      {
                        std::cout << "Src: " << curStaticInst->srcRegIdx(i) <<
                                      " {PID(0)}" << std::endl;
                      }
                    }
                  }

                  std::cout << "Dest:" << curStaticInst->destRegIdx(i) <<
                                " {PID(" << dest_bk->pid << ")}" << std::endl;

              }
          }
        }

    }


    if (curStaticInst->isMemRef()){
        // heap access
        Block* bk = find_Block_containing(curStaticInst->atomic_vaddr);
        // if found: update the ShadowMemory
        if (bk) { // just the base addresses
          assert(bk->pid != 0);
          if (bk){
              std::cout << pcState;
              std::cout <<
                curStaticInst->disassemble(pcState.pc()) << std::endl;
              std::cout << "Heap Access:" << std::hex <<
                            curStaticInst->atomic_vaddr << std::dec <<
                            " {PID(" << bk->pid << ")}" << std::endl;
          }
        }
    }

}

//then call this
void AtomicSimpleCPU::ComparePointerTrackerSpeculative(ThreadContext * tc,
                         PCState &pcState)
{
  SimpleExecContext& t_info = *threadInfo[0];
  SimpleThread* thread = t_info.thread;

  if (thread->stop_tracking) return;

  for (size_t i = 0; i < curStaticInst->numDestRegs(); i++) {
      if (curStaticInst->destRegIdx(i).isIntReg() &&
          curStaticInst->destRegIdx(i).index() < TheISA::NumIntArchRegs)
      {
          uint64_t dataRegContent =
                  thread->readIntReg(curStaticInst->destRegIdx(i).index());
          Block* dest_bk = find_Block_containing(dataRegContent);
          if (dest_bk){
              if (tc->PointerTracker[curStaticInst->destRegIdx(i).index()] !=
                  dest_bk->pid)
              {
                std::cout << "warning! misspath found!\n";
                std::cout << pcState;
                std::cout <<
                    curStaticInst->disassemble(pcState.pc()) << std::endl;
              }

          }
          else {
            if (tc->PointerTracker[curStaticInst->destRegIdx(i).index()] != 0)
            {
              std::cout << "warning! misspath found!\n";
              std::cout << pcState;
              std::cout <<
                  curStaticInst->disassemble(pcState.pc()) << std::endl;
            }
          }
      }
  }

}

// first call this
void AtomicSimpleCPU::UpdatePointerTrackerSpeculative(ThreadContext * tc,
                         PCState &pcState)
{
  SimpleExecContext& t_info = *threadInfo[0];
  SimpleThread* thread = t_info.thread;

  if (thread->stop_tracking) return;
  //transfer capabilities
  if (curStaticInst->isMallocBaseCollectorMicroop()){
      uint64_t dataRegContent =
              thread->readIntReg(curStaticInst->destRegIdx(0).index());
      Block* dest_bk = find_Block_containing(dataRegContent);
      if (dest_bk){
          tc->PointerTracker[X86ISA::INTREG_RAX] = dest_bk->pid;
      }
  }
  else if (curStaticInst->isLoad() && curStaticInst->getDataSize() == 8)
  {
      // pointer refill
      for (size_t i = 0; i < curStaticInst->numDestRegs(); i++) {
          if (curStaticInst->destRegIdx(i).isIntReg())
          {
              uint64_t dataRegContent =
                    thread->readIntReg(curStaticInst->destRegIdx(i).index());
              Block* dest_bk = find_Block_containing(dataRegContent);
              if (dest_bk){
                tc->PointerTracker[curStaticInst->destRegIdx(i).index()] =
                  dest_bk->pid;
              }
          }
      }

  }
  else if (curStaticInst->isLoad() && curStaticInst->getDataSize() != 8)
  {
      // pointer refill
      for (size_t i = 0; i < curStaticInst->numDestRegs(); i++) {
          if (curStaticInst->destRegIdx(i).isIntReg())
          {
              if (curStaticInst->destRegIdx(i).index() < TheISA::NumIntRegs){
                tc->PointerTracker[curStaticInst->destRegIdx(i).index()] = 0;
              }
          }
      }

  }
  else if (curStaticInst->getDataSize() == 8) {

      uint64_t pid = 0;
      for (size_t i = 0; i < curStaticInst->numSrcRegs(); i++) {
        if (curStaticInst->srcRegIdx(i).isIntReg())
        {
          if (curStaticInst->srcRegIdx(i).index() < TheISA::NumIntRegs){
            if (tc->PointerTracker[curStaticInst->srcRegIdx(i).index()] != 0){
              pid = tc->PointerTracker[curStaticInst->srcRegIdx(i).index()];
               break;
            }
          }
        }
      }

      for (size_t i = 0; i < curStaticInst->numDestRegs(); i++) {
        if (curStaticInst->destRegIdx(i).isIntReg())
        {
          if (curStaticInst->destRegIdx(i).index() < TheISA::NumIntRegs){
            tc->PointerTracker[curStaticInst->destRegIdx(i).index()] = pid;
          }
        }
      }

  }
  else if (curStaticInst->getDataSize() != 8) {
      for (size_t i = 0; i < curStaticInst->numDestRegs(); i++) {
        if (curStaticInst->destRegIdx(i).isIntReg())
        {
          if (curStaticInst->destRegIdx(i).index() < TheISA::NumIntRegs){
            tc->PointerTracker[curStaticInst->destRegIdx(i).index()] = 0;
          }
        }
      }

  }

  // zero out all interface regs for the next macroopp
  if (curStaticInst->isLastMicroop()){
    for (size_t i = X86ISA::NUM_INTREGS; i < TheISA::NumIntRegs; i++) {
      tc->PointerTracker[i] = 0;
    }
  }

  // bool dumped = false;
  // for (size_t i = 0; i < X86ISA::NUM_INTREGS; i++) {
  //   if (tc->PointerTracker[i]){
  //     dumped = true;
  //     break;
  //   }
  // }

  // if (dumped)
  // {
  //   std::cout << "***************************************************\n";
  //   std::cout << pcState;
  //   std::cout << curStaticInst->disassemble(pcState.pc()) << std::endl;
  //   std::cout << "***************************************************\n";
  //   for (size_t i = 0; i < X86ISA::NUM_INTREGS; i++) {
  //     if (tc->PointerTracker[i]){
  //       std::cout << IntRegIndexStr(i) << " = " <<
  //                      tc->PointerTracker[i] << std::endl;
  //     }
  //   }
  //   std::cout << "****************************************************\n";
  // }

}


// first call this
void AtomicSimpleCPU::Verify(ThreadContext * tc,
                         PCState &pcState)
{

  SimpleExecContext& t_info = *threadInfo[0];
  SimpleThread* thread = t_info.thread;
  if (thread->stop_tracking) return;
  if (!curStaticInst->isInteger()) return;
  if (curStaticInst->isStore()) return;

  // src and reg PID determination
  for (size_t i = 0; i < curStaticInst->numSrcRegs(); i++) {
    if (curStaticInst->srcRegIdx(i).isIntReg())
    {
        uint64_t srcDataRegContent =
            thread->readIntReg(curStaticInst->srcRegIdx(i).index());
        Block* src_bk = find_Block_containing(srcDataRegContent);
        if (src_bk){
            curStaticInst->setSrcRegPid(i, src_bk->pid);
        }
        else {
            curStaticInst->setSrcRegPid(i, 0);
        }
    }
    else {
      curStaticInst->setSrcRegPid(i, 0);
    }
  }

  for (size_t i = 0; i < curStaticInst->numDestRegs(); i++) {
    if (curStaticInst->destRegIdx(i).isIntReg())
    {
        uint64_t destDataRegContent =
            thread->readIntReg(curStaticInst->destRegIdx(i).index());
        Block* dest_bk = find_Block_containing(destDataRegContent);
        if (dest_bk){
            curStaticInst->setDestRegPid(i, dest_bk->pid);
        }
        else {
            curStaticInst->setDestRegPid(i, 0);
        }
    }
    else {
      curStaticInst->setDestRegPid(i, 0);
    }
  }

  //sanitization
  for (size_t i = 0; i < curStaticInst->numSrcRegs(); i++) {
      for (size_t j = i+1; j < curStaticInst->numSrcRegs(); j++) {
        if (curStaticInst->getSrcRegPid(i) != 0 &&
            curStaticInst->getSrcRegPid(j) != 0 &&
            curStaticInst->getSrcRegPid(i) != curStaticInst->getSrcRegPid(j))
        {
          std::cout << pcState;
          std::cout << curStaticInst->disassemble(pcState.pc()) << std::endl;
          for (size_t k = 0; k < curStaticInst->numSrcRegs(); k++) {
            std::cout << "SRC " << k << ": " <<
                      curStaticInst->getSrcRegPid(k) << std::endl;
          }
          for (size_t k = 0; k < curStaticInst->numDestRegs(); k++) {
            std::cout << "DEST " << k << ": " <<
                      curStaticInst->getDestRegPid(k) << std::endl;
          }
          panic("What kind of sorcery is this!");
        }
      }

  }

  for (size_t i = 0; i < curStaticInst->numDestRegs(); i++) {
      for (size_t j = i+1; j < curStaticInst->numDestRegs(); j++) {
        if (curStaticInst->getDestRegPid(i) != 0 &&
            curStaticInst->getDestRegPid(j) != 0 &&
            curStaticInst->getDestRegPid(i) != curStaticInst->getDestRegPid(j))
        {
          std::cout << pcState;
          std::cout << curStaticInst->disassemble(pcState.pc()) << std::endl;
          for (size_t k = 0; k < curStaticInst->numSrcRegs(); k++) {
            std::cout << "SRC " << k << ": " <<
                      curStaticInst->getSrcRegPid(k) << std::endl;
          }
          for (size_t k = 0; k < curStaticInst->numDestRegs(); k++) {
            std::cout << "DEST " << k << ": " <<
                      curStaticInst->getDestRegPid(k) << std::endl;
          }
          panic("What kind of sorcery is this!");
        }
      }

  }

  // rules
  if (curStaticInst->isLoad() && curStaticInst->getDataSize() == 8)
  {
      // pointer refill
      for (size_t i = 0; i < curStaticInst->numDestRegs(); i++) {

      }

  }
  else if (curStaticInst->isLoad() && curStaticInst->getDataSize() != 8)
  {

      for (size_t i = 0; i < curStaticInst->numDestRegs(); i++) {
          if (curStaticInst->getDestRegPid(i))
          {
                std::cout << "Warning: Load.datasize != 8 : " << std::endl;
                std::cout << pcState;
                std::cout << curStaticInst->disassemble(pcState.pc()) <<
                            std::endl;
          }
      }

  }
  else if (curStaticInst->getDataSize() == 8) {

      for (size_t i = 0; i < curStaticInst->numSrcRegs(); i++) {
        if (curStaticInst->getSrcRegPid(i))
        {


        }
      }

  }
  else if (curStaticInst->getDataSize() != 8) {


  }


}
