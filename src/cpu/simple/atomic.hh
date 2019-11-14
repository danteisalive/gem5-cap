/*
 * Copyright (c) 2012-2013, 2015, 2018 ARM Limited
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
#ifndef __CPU_SIMPLE_ATOMIC_HH__
#define __CPU_SIMPLE_ATOMIC_HH__

#include <fstream>

#include "cpu/simple/WordFM.hh"
#include "cpu/simple/base.hh"
#include "cpu/simple/exec_context.hh"
#include "mem/request.hh"
#include "params/AtomicSimpleCPU.hh"
#include "sim/probe/probe.hh"

#define likely(x)       __builtin_expect((x),1)

class AtomicSimpleCPU : public BaseSimpleCPU
{
  public:

    AtomicSimpleCPU(AtomicSimpleCPUParams *params);
    virtual ~AtomicSimpleCPU();

    void init() override;

  protected:

    EventFunctionWrapper tickEvent;

    const int width;
    bool locked;
    const bool simulate_data_stalls;
    const bool simulate_inst_stalls;

    // main simulation loop (one cycle)
    void tick();

    void collector(ThreadContext * _tc,
                TheISA::PCState &pcState,
                TheISA::CheckType _sym);

    void WarmupAliasTable(ThreadContext * _tc, TheISA::PCState &pcState);
    void updateAliasTable(ThreadContext * _tc, TheISA::PCState &pcState);
    void updateAliasTableWithStack(ThreadContext * _tc,
                                   TheISA::PCState &pcState);
    void ComparePointerTrackerSpeculative(ThreadContext * tc,
                                    TheISA::PCState &pcState);
    void UpdatePointerTrackerSpeculative(ThreadContext * tc,
                                    TheISA::PCState &pcState);
    void UpdatePointerTracker(ThreadContext * _tc,
                                   TheISA::PCState &pcState);

    void AccessCapabilityCache(ThreadContext * tc,
                             TheISA::PCState &pcState);

    void Verify(ThreadContext * tc,
                              TheISA::PCState &pcState);

    void trackAlias(TheISA::PCState &pcState);
    Block* find_Block_containing ( Addr a );
    void getLog(ThreadContext * _tc, TheISA::PCState &pcState);

    WordFM* interval_tree = NULL;  /* WordFM* Block* void */
    std::map<Addr, std::vector<Block>> PIDLogs;
            // Function Name       // PC            //PIDs
    std::map<std::string, std::map<uint64_t, std::vector<uint64_t>>>
                                                debug_function_calls;
    uint64_t max_insts_any_thread;
    uint64_t numOfMemRefs;
    uint64_t numOfHeapAccesses;

    typedef std::map<uint64_t, uint64_t> PTL6;
    typedef std::map<uint64_t, PTL6> PTL5;
    typedef std::map<uint64_t, PTL5> PTL4;
    typedef std::map<uint64_t, PTL4> PTL3;
    typedef std::map<uint64_t, PTL3> PTL2;
    typedef std::map<uint64_t, PTL2> PTL1;

    #define LV6_MASK   uint64_t(0b1111111111111)
    #define LV5_MASK   uint64_t(0b111111111) << (13)
    #define LV4_MASK   uint64_t(0b111111111) << (13 + 9)
    #define LV3_MASK   uint64_t(0b111111111) << (13 + 9 + 9)
    #define LV2_MASK   uint64_t(0b111111111) << (13 + 9 + 9 + 9)
    #define LV1_MASK   uint64_t(0b11111111)  << (13 + 9 + 9 + 9 + 9)

    PTL1 AliasPageTable;

    // 2-entry cache for find_Block_containing
    Block* fbc_cache0 = NULL;
    Block* fbc_cache1 = NULL;
    /**
     * Check if a system is in a drained state.
     *
     * We need to drain if:
     * <ul>
     * <li>We are in the middle of a microcode sequence as some CPUs
     *     (e.g., HW accelerated CPUs) can't be started in the middle
     *     of a gem5 microcode sequence.
     *
     * <li>The CPU is in a LLSC region. This shouldn't normally happen
     *     as these are executed atomically within a single tick()
     *     call. The only way this can happen at the moment is if
     *     there is an event in the PC event queue that affects the
     *     CPU state while it is in an LLSC region.
     *
     * <li>Stay at PC is true.
     * </ul>
     */
    bool isDrained() {
        SimpleExecContext &t_info = *threadInfo[curThread];

        return t_info.thread->microPC() == 0 &&
            !locked &&
            !t_info.stayAtPC;
    }

    /**
     * Try to complete a drain request.
     *
     * @returns true if the CPU is drained, false otherwise.
     */
    bool tryCompleteDrain();

    virtual Tick sendPacket(MasterPort &port, const PacketPtr &pkt);

    /**
     * An AtomicCPUPort overrides the default behaviour of the
     * recvAtomicSnoop and ignores the packet instead of panicking. It
     * also provides an implementation for the purely virtual timing
     * functions and panics on either of these.
     */
    class AtomicCPUPort : public MasterPort
    {

      public:

        AtomicCPUPort(const std::string &_name, BaseSimpleCPU* _cpu)
            : MasterPort(_name, _cpu)
        { }

      protected:

        bool recvTimingResp(PacketPtr pkt)
        {
            panic("Atomic CPU doesn't expect recvTimingResp!\n");
            return true;
        }

        void recvReqRetry()
        {
            panic("Atomic CPU doesn't expect recvRetry!\n");
        }

    };

    class AtomicCPUDPort : public AtomicCPUPort
    {

      public:
        AtomicCPUDPort(const std::string &_name, BaseSimpleCPU* _cpu)
            : AtomicCPUPort(_name, _cpu), cpu(_cpu)
        {
            cacheBlockMask = ~(cpu->cacheLineSize() - 1);
        }

        bool isSnooping() const { return true; }

        Addr cacheBlockMask;
      protected:
        BaseSimpleCPU *cpu;

        virtual Tick recvAtomicSnoop(PacketPtr pkt);
        virtual void recvFunctionalSnoop(PacketPtr pkt);
    };


    AtomicCPUPort icachePort;
    AtomicCPUDPort dcachePort;


    RequestPtr ifetch_req;
    RequestPtr data_read_req;
    RequestPtr data_write_req;

    bool dcache_access;
    Tick dcache_latency;

    /** Probe Points. */
    ProbePointArg<std::pair<SimpleThread*, const StaticInstPtr>> *ppCommit;

  protected:

    /** Return a reference to the data port. */
    MasterPort &getDataPort() override { return dcachePort; }

    /** Return a reference to the instruction port. */
    MasterPort &getInstPort() override { return icachePort; }

    /** Perform snoop for other cpu-local thread contexts. */
    void threadSnoop(PacketPtr pkt, ThreadID sender);

  public:

    DrainState drain() override;
    void drainResume() override;

    void switchOut() override;
    void takeOverFrom(BaseCPU *oldCPU) override;

    void verifyMemoryMode() const override;

    void activateContext(ThreadID thread_num) override;
    void suspendContext(ThreadID thread_num) override;

    Fault readMem(Addr addr, uint8_t *data, unsigned size,
                  Request::Flags flags) override;

    Fault initiateMemRead(Addr addr, unsigned size,
                          Request::Flags flags) override;

    Fault writeMem(uint8_t *data, unsigned size,
                   Addr addr, Request::Flags flags, uint64_t *res) override;

    void regProbePoints() override;

    /**
     * Print state of address in memory system via PrintReq (for
     * debugging).
     */
    void printAddr(Addr a);
};

#endif // __CPU_SIMPLE_ATOMIC_HH__
