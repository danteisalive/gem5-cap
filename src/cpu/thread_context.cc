/*
 * Copyright (c) 2012, 2016 ARM Limited
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

#include "cpu/thread_context.hh"

#include "arch/kernel_stats.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/quiesce_event.hh"
#include "debug/Context.hh"
#include "debug/Quiesce.hh"
#include "params/BaseCPU.hh"
#include "sim/full_system.hh"

void
ThreadContext::compare(ThreadContext *one, ThreadContext *two)
{
    DPRINTF(Context, "Comparing thread contexts\n");

    // First loop through the integer registers.
    for (int i = 0; i < TheISA::NumIntRegs; ++i) {
        TheISA::IntReg t1 = one->readIntReg(i);
        TheISA::IntReg t2 = two->readIntReg(i);
        if (t1 != t2)
            panic("Int reg idx %d doesn't match, one: %#x, two: %#x",
                  i, t1, t2);
    }

    // Then loop through the floating point registers.
    for (int i = 0; i < TheISA::NumFloatRegs; ++i) {
        TheISA::FloatRegBits t1 = one->readFloatRegBits(i);
        TheISA::FloatRegBits t2 = two->readFloatRegBits(i);
        if (t1 != t2)
            panic("Float reg idx %d doesn't match, one: %#x, two: %#x",
                  i, t1, t2);
    }

    // Then loop through the vector registers.
    for (int i = 0; i < TheISA::NumVecRegs; ++i) {
        RegId rid(VecRegClass, i);
        const TheISA::VecRegContainer& t1 = one->readVecReg(rid);
        const TheISA::VecRegContainer& t2 = two->readVecReg(rid);
        if (t1 != t2)
            panic("Vec reg idx %d doesn't match, one: %#x, two: %#x",
                  i, t1, t2);
    }
    for (int i = 0; i < TheISA::NumMiscRegs; ++i) {
        TheISA::MiscReg t1 = one->readMiscRegNoEffect(i);
        TheISA::MiscReg t2 = two->readMiscRegNoEffect(i);
        if (t1 != t2)
            panic("Misc reg idx %d doesn't match, one: %#x, two: %#x",
                  i, t1, t2);
    }

    // loop through the Condition Code registers.
    for (int i = 0; i < TheISA::NumCCRegs; ++i) {
        TheISA::CCReg t1 = one->readCCReg(i);
        TheISA::CCReg t2 = two->readCCReg(i);
        if (t1 != t2)
            panic("CC reg idx %d doesn't match, one: %#x, two: %#x",
                  i, t1, t2);
    }
    if (!(one->pcState() == two->pcState()))
        panic("PC state doesn't match.");
    int id1 = one->cpuId();
    int id2 = two->cpuId();
    if (id1 != id2)
        panic("CPU ids don't match, one: %d, two: %d", id1, id2);

    const ContextID cid1 = one->contextId();
    const ContextID cid2 = two->contextId();
    if (cid1 != cid2)
        panic("Context ids don't match, one: %d, two: %d", id1, id2);


}

void
ThreadContext::quiesce()
{
    if (!getCpuPtr()->params()->do_quiesce)
        return;

    DPRINTF(Quiesce, "%s: quiesce()\n", getCpuPtr()->name());

    suspend();
    if (getKernelStats())
       getKernelStats()->quiesce();
}


void
ThreadContext::quiesceTick(Tick resume)
{
    BaseCPU *cpu = getCpuPtr();

    if (!cpu->params()->do_quiesce)
        return;

    EndQuiesceEvent *quiesceEvent = getQuiesceEvent();

    cpu->reschedule(quiesceEvent, resume, true);

    DPRINTF(Quiesce, "%s: quiesceTick until %lu\n", cpu->name(), resume);

    suspend();
    if (getKernelStats())
        getKernelStats()->quiesce();
}

void
serialize(ThreadContext &tc, CheckpointOut &cp)
{


    using namespace TheISA;

    FloatRegBits floatRegs[NumFloatRegs];
    for (int i = 0; i < NumFloatRegs; ++i)
        floatRegs[i] = tc.readFloatRegBitsFlat(i);
    // This is a bit ugly, but needed to maintain backwards
    // compatibility.
    arrayParamOut(cp, "floatRegs.i", floatRegs, NumFloatRegs);

    std::vector<TheISA::VecRegContainer> vecRegs(NumVecRegs);
    for (int i = 0; i < NumVecRegs; ++i) {
        vecRegs[i] = tc.readVecRegFlat(i);
    }
    SERIALIZE_CONTAINER(vecRegs);

    IntReg intRegs[NumIntRegs];
    for (int i = 0; i < NumIntRegs; ++i)
        intRegs[i] = tc.readIntRegFlat(i);
    SERIALIZE_ARRAY(intRegs, NumIntRegs);

#ifdef ISA_HAS_CC_REGS
    CCReg ccRegs[NumCCRegs];
    for (int i = 0; i < NumCCRegs; ++i)
        ccRegs[i] = tc.readCCRegFlat(i);
    SERIALIZE_ARRAY(ccRegs, NumCCRegs);
#endif

    tc.pcState().serialize(cp);

    // thread_num and cpu_id are deterministic from the config

    int num_of_alias_entrys = 0;
    // serialize alias table
    if (tc.enableCapability){
        std::string data = "";
        int pass_size = 0;
        std::string filename = "system.alias.physmem.smem";
        std::string filepath = CheckpointIn::dir() + filename.c_str();
        gzFile compressed_alias = gzopen(filepath.c_str(), "wb");
        if (compressed_alias == NULL)
            fatal("Can't open alias table checkpoint file '%s'\n",
                  filename);
        std::cout << filepath << std::endl;
        // gzwrite fails if (int)len < 0 (gzwrite returns int)
        pass_size = 0;
        for (auto& entry1: tc.ShadowMemory){
          if (pass_size == 10){
              if (data.size() > 0){
                  std::ostringstream buffer(data);
                  if (gzwrite(compressed_alias, buffer.str().c_str(),
                                                buffer.str().size()) !=
                                                (int) buffer.str().size()) {
                  fatal("Write failed on alias table checkpoint file '%s'\n",
                                          filename);
                  }
                  //std::cout << buffer.str() << std::endl;
              }

              data = ""; // zero out
              pass_size = 0;
          }
          else {
              for (auto& entry2 : entry1.second){
                // check to see if this pid is free or not in AtomicSimpleCPU

                auto it = std::find(tc.freedPIDVector.begin(),
                                    tc.freedPIDVector.end(),
                                    entry2.second.getPID());
                //cant find this pid in freedPIDVector threfore write it
                if (it == tc.freedPIDVector.end())
                {
                  num_of_alias_entrys++;
                  std::ostringstream temp1, temp2;
                  temp1 << std::hex << std::setw(16) <<
                           std::setfill('0') << entry2.first;
                  temp2 << std::hex << std::setw(16) <<
                           std::setfill('0') << entry2.second.getPID();
                  data += temp1.str() + std::string(" ") +
                          temp2.str() + std::string(" ");
                }
              }
              pass_size++;
          }
        }
        // writing last bytes
        if (data.size() > 0){
            std::ostringstream buffer(data);
            if (gzwrite(compressed_alias, buffer.str().c_str(),
                                          buffer.str().size()) !=
                                          (int) buffer.str().size()) {
                fatal("Write failed on alias table checkpoint file '%s'\n",
                                    filename);
            }
            //std::cout << buffer.str() << std::endl;
        }

        // close the compressed stream and check that the exit status
        // is zero
        if (gzclose(compressed_alias))
              fatal("Close failed on alias table checkpoint file '%s'\n",
                    filename);
    }
    //sanity check for gziped alias table
    if (tc.enableCapability){

      std::string data;
      std::string filename = "system.alias.physmem.smem";
      std::string filepath = CheckpointIn::dir() + filename.c_str();
      uint32_t bytes_read;
      // mmap memoryfile
      gzFile compressed_mem = gzopen(filepath.c_str(), "rb");
      if (compressed_mem == NULL)
          fatal("Can't open alias table checkpoint file '%s'", filename);

      data.resize(34*1000);
      while (1){
          bytes_read = gzread(compressed_mem, (void*) data.data(), 34*1000);
          if (bytes_read > 0){
            std::stringstream buffer;
            std::string effAddr,pid;
            buffer << data;

            if (bytes_read % 34 == 0){
                for (size_t i = 0; i < bytes_read/34; i++) {
                  num_of_alias_entrys--;
                  buffer >> effAddr; buffer >> pid;
                  // std::cout << std::hex <<
                  //           std::strtoull(effAddr.c_str(),0,16) << " " <<
                  //           std::dec << std::strtoull(pid.c_str(),0,16) <<
                  //           std::endl;
                }
            }
            else{
              panic("Invalid number of entrys in alias table Checkpoint file");
            }

          }
          else if (bytes_read == 0){
            break;
          }
          else {
            panic("Alias table Checkpoint file (bytes_read < 0)");
          }
      }

    }

    panic_if(num_of_alias_entrys != 0, "Sanity check for alias table failed!");

    int num_of_cap_entrys = 0;
    //serialize interval_tree
    if (tc.enableCapability){
        std::string data = "";
        int pass_size = 0;
        std::string filename = "system.capability.physmem.smem";
        std::string filepath = CheckpointIn::dir() + filename.c_str();
        gzFile compressed_alias = gzopen(filepath.c_str(), "wb");
        if (compressed_alias == NULL)
            fatal("Can't open capability checkpoint file '%s'\n",
                  filename);
        pass_size = 0;
        UWord keyW, valW;

        VG_initIterFM(tc.interval_tree);
        while (VG_nextIterFM(tc.interval_tree, &keyW, &valW )) {
           Block* bk = (Block*)keyW;
           assert(valW == 0);
           assert(bk);
           //dump
           if (pass_size == 1000){
               if (data.size() > 0){
                   std::ostringstream buffer(data);
                   if (gzwrite(compressed_alias, buffer.str().c_str(),
                                                 buffer.str().size()) !=
                                                 (int) buffer.str().size())
                   {
                     fatal("Write failed on capability checkpoint file '%s'\n",
                                           filename);
                   }
                   //std::cout << buffer.str() << std::endl;
               }

               data = ""; // zero out
               pass_size = 0;
           }
           //collect
           else {
               num_of_cap_entrys++;
               std::ostringstream temp1, temp2, temp3;
               temp1 << std::hex << std::setw(16) <<
                        std::setfill('0') << bk->payload;
               temp2 << std::hex << std::setw(16) <<
                        std::setfill('0') << bk->req_szB;
               temp3 << std::hex << std::setw(16) <<
                        std::setfill('0') << bk->pid;
               data +=  temp1.str() + std::string(" ") +
                        temp2.str() + std::string(" ") +
                        temp3.str() + std::string(" ");
               pass_size++;
           }
        }
        VG_doneIterFM(tc.interval_tree);

        // dump remaining pids
        // writing last bytes
        if (data.size() > 0){
            std::ostringstream buffer(data);
            if (gzwrite(compressed_alias, buffer.str().c_str(),
                                          buffer.str().size()) !=
                                          (int) buffer.str().size()) {
                fatal("Write failed on capability checkpoint file '%s'\n",
                                    filename);
            }
            //std::cout << buffer.str() << std::endl;
        }

        // close the compressed stream and check that the exit status
        // is zero
        if (gzclose(compressed_alias))
              fatal("Close failed on capability checkpoint file '%s'\n",
                    filename);
    }
    //sanity check for gziped interval tree
    if (tc.enableCapability){

      std::string data;
      std::string filename = "system.capability.physmem.smem";
      std::string filepath = CheckpointIn::dir() + filename.c_str();
      uint32_t bytes_read;
      // mmap memoryfile
      gzFile compressed_mem = gzopen(filepath.c_str(), "rb");
      if (compressed_mem == NULL)
          fatal("Can't open capability checkpoint file '%s'", filename);

      data.resize(51*1000);  // 3 * 16 + 3
      while (1){
          bytes_read = gzread(compressed_mem, (void*) data.data(), 51*1000);
          if (bytes_read > 0){
              std::stringstream buffer;
              std::string payload,pid,size;
              buffer << data;
              if (bytes_read % 51 == 0){
                  for (size_t i = 0; i < bytes_read/51; i++) {
                    num_of_cap_entrys--;
                    buffer >> payload; buffer >> size; buffer >> pid;
                    // std::cout << std::hex <<
                    //        std::strtoull(payload.c_str(),0,16) << " " <<
                    //        std::dec << std::strtoull(size.c_str(),0,16) <<
                    //        " " <<
                    //        std::dec << std::strtoull(pid.c_str(),0,16) <<
                    //        " " <<
                    //        std::endl;
                  }
              }
              else{
              panic("Invalid number of entrys in capability Checkpoint file");
              }
          }
          else if (bytes_read == 0){
            break;
          }
          else {
            panic("Capability Checkpoint file (bytes_read < 0)");
          }
      }

    }

    panic_if(num_of_cap_entrys != 0, "Sanity check for interval_tree failed!");


    if (tc.InSlice)
      tc.ShadowMemory.clear();

    tc.InSlice = !tc.InSlice;

}

void
unserialize(ThreadContext &tc, CheckpointIn &cp)
{
    using namespace TheISA;

    FloatRegBits floatRegs[NumFloatRegs];
    // This is a bit ugly, but needed to maintain backwards
    // compatibility.
    arrayParamIn(cp, "floatRegs.i", floatRegs, NumFloatRegs);
    for (int i = 0; i < NumFloatRegs; ++i)
        tc.setFloatRegBitsFlat(i, floatRegs[i]);

    std::vector<TheISA::VecRegContainer> vecRegs(NumVecRegs);
    UNSERIALIZE_CONTAINER(vecRegs);
    for (int i = 0; i < NumVecRegs; ++i) {
        tc.setVecRegFlat(i, vecRegs[i]);
    }

    IntReg intRegs[NumIntRegs];
    UNSERIALIZE_ARRAY(intRegs, NumIntRegs);
    for (int i = 0; i < NumIntRegs; ++i)
        tc.setIntRegFlat(i, intRegs[i]);

#ifdef ISA_HAS_CC_REGS
    CCReg ccRegs[NumCCRegs];
    UNSERIALIZE_ARRAY(ccRegs, NumCCRegs);
    for (int i = 0; i < NumCCRegs; ++i)
        tc.setCCRegFlat(i, ccRegs[i]);
#endif

    PCState pcState;
    pcState.unserialize(cp);
    tc.pcState(pcState);

    // thread_num and cpu_id are deterministic from the config
    //deserlizing the alias table
    if (tc.enableCapability){
      int alias_read = 0;
      Process* p = tc.getProcessPtr();
      std::string data;
      std::string filename = "system.alias.physmem.smem";
      std::string filepath = CheckpointIn::dir() + filename.c_str();
      std::cout << filepath << std::endl;
      uint32_t bytes_read;
      // mmap memoryfile
      gzFile compressed_mem = gzopen(filepath.c_str(), "rb");
      if (compressed_mem == NULL)
          fatal("Can't open alias table checkpoint file '%s'", filename);

      data.resize(34*1000);
      while (1){
          bytes_read = gzread(compressed_mem, (void*) data.data(), 34*1000);
          if (bytes_read > 0){
            std::stringstream buffer;
            std::string effAddr,pid;
            buffer << data;

            if (bytes_read % 34 == 0){
                for (size_t i = 0; i < bytes_read/34; i++) {
                  buffer >> effAddr; buffer >> pid;
                  uint64_t effAddr_val = std::strtoull(effAddr.c_str(),0,16);
                  uint64_t pid_val = std::strtoull(pid.c_str(),0,16);
                  Addr vpn = p->pTable->pageAlign(effAddr_val);
                  tc.ShadowMemory[vpn][effAddr_val] =
                                              TheISA::PointerID(pid_val);
                  alias_read++;
                }
            }
            else{
              panic("Invalid number of entrys in alias table Checkpoint file");
            }

          }
          else if (bytes_read == 0){
            break;
          }
          else {
            panic("Alias table Checkpoint file (bytes_read < 0)");
          }
      }
      std::cout << "In total read " << alias_read << " aliases!" <<
                std::endl;
    }

    if (tc.enableCapability){

      int capabilities_read = 0;
      std::string data;
      std::string filename = "system.capability.physmem.smem";
      std::string filepath = CheckpointIn::dir() + filename.c_str();
      uint32_t bytes_read;
      // mmap memoryfile
      gzFile compressed_mem = gzopen(filepath.c_str(), "rb");
      if (compressed_mem == NULL)
          fatal("Can't open capability checkpoint file '%s'", filename);

      data.resize(51*1000);  // 3 * 16 + 3
      while (1){
          bytes_read = gzread(compressed_mem, (void*) data.data(), 51*1000);
          if (bytes_read > 0){
              std::stringstream buffer;
              std::string payload,pid,size;
              buffer << data;
              if (bytes_read % 51 == 0){
                  for (size_t i = 0; i < bytes_read/51; i++) {
                    buffer >> payload; buffer >> size; buffer >> pid;
                    uint64_t payload_val = std::strtoull(payload.c_str(),0,16);
                    uint64_t size_val = std::strtoull(size.c_str(),0,16);
                    uint64_t pid_val = std::strtoull(pid.c_str(),0,16);
                    Block* bk = new Block();
                    bk->payload   = (Addr)payload_val;
                    bk->req_szB   = (SizeT)size_val;
                    bk->pid       = (Addr)pid_val;
                    unsigned char present =
                            VG_addToFM(tc.interval_tree, (UWord)bk, (UWord)0);
                    assert(!present);
                    capabilities_read++;
                  }
              }
              else{
              panic("Invalid number of entrys in capability Checkpoint file");
              }
          }
          else if (bytes_read == 0){
            break;
          }
          else {
            panic("Capability Checkpoint file (bytes_read < 0)");
          }
      }

      std::cout << "In total read " << capabilities_read << " capabilities!" <<
                std::endl;
    }

    for (int i = 0; i < TheISA::NumIntRegs; i++)
    {
       uint64_t data = intRegs[i];
       Block fake;
       fake.payload = data;
       fake.req_szB = 1;
       UWord foundkey = 1;
       UWord foundval = 1;
       unsigned char found = VG_lookupFM( tc.interval_tree,
                                   &foundkey, &foundval, (UWord)&fake );
       if (found)
       {
          Block* res = (Block*)foundkey;
          tc.PointerTrackerTable[i] = TheISA::PointerID(res->pid);
          std::cout << TheISA::PointerID(res->pid) << std::endl;
       }
       else
       {
          tc.PointerTrackerTable[i] = TheISA::PointerID(0);
          std::cout << TheISA::PointerID(0) << std::endl;
       }
    }
}

void
takeOverFrom(ThreadContext &ntc, ThreadContext &otc)
{
    assert(ntc.getProcessPtr() == otc.getProcessPtr());

    ntc.setStatus(otc.status());
    ntc.copyArchRegs(&otc);
    ntc.setContextId(otc.contextId());
    ntc.setThreadId(otc.threadId());

    if (FullSystem) {
        assert(ntc.getSystemPtr() == otc.getSystemPtr());

        BaseCPU *ncpu(ntc.getCpuPtr());
        assert(ncpu);
        EndQuiesceEvent *oqe(otc.getQuiesceEvent());
        assert(oqe);
        assert(oqe->tc == &otc);

        BaseCPU *ocpu(otc.getCpuPtr());
        assert(ocpu);
        EndQuiesceEvent *nqe(ntc.getQuiesceEvent());
        assert(nqe);
        assert(nqe->tc == &ntc);

        if (oqe->scheduled()) {
            ncpu->schedule(nqe, oqe->when());
            ocpu->deschedule(oqe);
        }
    }

    otc.setStatus(ThreadContext::Halted);
}
