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
 * Authors: Kevin Lim
 */

#include "cpu/pred/lvpt.hh"

#include "base/intmath.hh"
#include "base/trace.hh"
#include "debug/Capability.hh"

DefaultLVPT::DefaultLVPT(unsigned _numEntries,
                       unsigned _tagBits,
                       unsigned _instShiftAmt,
                       unsigned _num_threads)
    : predHist(_num_threads),
      numEntries(_numEntries),
      tagBits(_tagBits),
      instShiftAmt(_instShiftAmt),
      log2NumThreads(floorLog2(_num_threads))
{
    DPRINTF(Capability, "LVPT: Creating LVPT object.\n");

    if (!isPowerOf2(numEntries)) {
        fatal("LVPT entries is not a power of 2!");
    }

    lvpt.resize(numEntries);

    for (unsigned i = 0; i < numEntries; ++i) {
        lvpt[i].valid = false;
    }

    idxMask = numEntries - 1;

    tagMask = (1 << tagBits) - 1;

    tagShiftAmt = instShiftAmt + floorLog2(numEntries); // 12 + 2

    //Setup the array of counters for the local predictor
    localBiases.resize(numEntries);

    for (int i = 0; i < numEntries; ++i){
        localBiases[i] = 0;
    }

    localCtrs.resize(numEntries);
    for (size_t i = 0; i < numEntries; i++) {
        localCtrs[i].setBits(2);
        localCtrs[i].setInitial(0x3); // initial value is 0x11
        localCtrs[i].reset();
    }

    confLevel.resize(numEntries);
    for (size_t i = 0; i < numEntries; i++) {
        confLevel[i].setBits(4);
        confLevel[i].setInitial(0); // initial value is 0
        confLevel[i].reset();
    }

    localPointerPredictor.resize(numEntries);
    for (size_t i = 0; i < numEntries; i++) {
        localPointerPredictor[i].setBits(4);
        localPointerPredictor[i].setInitial(0); // initial value is 0
        localPointerPredictor[i].reset();
    }

    predictorMissHistory.resize(numEntries);
    predictorMissCount.resize(numEntries);
    for (size_t i = 0; i < numEntries; i++) {
      predictorMissCount[i] = 0;
    }



}

void
DefaultLVPT::reset()
{
    for (unsigned i = 0; i < numEntries; ++i) {
        lvpt[i].valid = false;
    }
}

inline
unsigned
DefaultLVPT::getIndex(Addr instPC, ThreadID tid)
{
    // Need to shift PC over by the word offset.
    return ((instPC >> instShiftAmt)
            ^ (tid << (tagShiftAmt - instShiftAmt - log2NumThreads)))
            & idxMask;
}

inline
Addr
DefaultLVPT::getTag(Addr instPC)
{
    return (instPC >> tagShiftAmt) & tagMask;
}

bool
DefaultLVPT::valid(Addr instPC, ThreadID tid)
{
    unsigned lvpt_idx = getIndex(instPC, tid);

    Addr inst_tag = getTag(instPC);

    assert(lvpt_idx < numEntries);

    if (lvpt[lvpt_idx].valid
        && inst_tag == lvpt[lvpt_idx].tag
        && lvpt[lvpt_idx].tid == tid) {
        return true;
    } else {
        return false;
    }
}

// @todo Create some sort of return struct that has both whether or not the
// address is valid, and also the address.  For now will just use addr = 0 to
// represent invalid entry.
TheISA::PointerID
DefaultLVPT::lookup(StaticInstPtr inst, Addr instPC, ThreadID tid)
{
    unsigned lvpt_idx = getIndex(instPC, tid);

    //Addr inst_tag = getTag(instPC);

    assert(lvpt_idx < numEntries);

    if (lvpt[lvpt_idx].valid)
    {

      TheISA::PointerID pred_pid = TheISA::PointerID(0);
      if (1/*inst_tag == lvpt[lvpt_idx].tag
          && lvpt[lvpt_idx].tid == tid*/)
      {
          switch (localCtrs[lvpt_idx].read()) {
            case 0x0:
              pred_pid = TheISA::PointerID(lvpt[lvpt_idx].target.getPID() +
                                       localBiases[lvpt_idx]);
              break;
            case 0x1:
              pred_pid = TheISA::PointerID(lvpt[lvpt_idx].target.getPID() +
                                       localBiases[lvpt_idx]);
              break;
            case 0x2:
            case 0x3:
              pred_pid = lvpt[lvpt_idx].target;
              break;

            default:
              assert("Invalud localCtrs value!");
              return TheISA::PointerID(0);
          }
      }
      // else {
      //     return TheISA::PointerID(0);
      // }
      // if (localPointerPredictor[lvpt_idx].read() > 0 &&
      //     pred_pid == TheISA::PointerID(0) &&
      //     confLevel[lvpt_idx].read() > 1)
      // {
      //     return TheISA::PointerID(0x1000000000000-1);
      // }

      // set the confidence level of this prediction

      inst->PredictionConfidenceLevel = (int)confLevel[lvpt_idx].read();
      inst->PredictionPointerRefillConfidence =
                                (int)localPointerPredictor[lvpt_idx].read();
      return pred_pid;

    }
    else
    {
        inst->PredictionConfidenceLevel = 0;
        inst->PredictionPointerRefillConfidence = 0;
        return TheISA::PointerID(0);
    }
}

void
DefaultLVPT::updateAndSnapshot(TheISA::PCState pc,
                    const InstSeqNum &seqNum,
                    Addr instPC,
                    const TheISA::PointerID &target,
                    const TheISA::PointerID &predicted_pid,
                    ThreadID tid, bool predict,
                    ThreadContext* tc
                   )
{

    if (predict){
      panic_if(predicted_pid.getPID() != target.getPID(),
              "Inequal PID when Prediction is True!");
    }
    unsigned lvpt_idx = getIndex(instPC, tid);

    assert(lvpt_idx < numEntries);


    // make a history for the entry which we are going to update
    // no matter it's predected right or not
    LVPTHistory *history = new LVPTHistory;


    history->localCtrEntry.write(localCtrs[lvpt_idx].read());
    history->localBiasEntry = localBiases[lvpt_idx];
    history->lvptEntry.tag = lvpt[lvpt_idx].tag;
    history->lvptEntry.tid = lvpt[lvpt_idx].tid;
    history->lvptEntry.valid = lvpt[lvpt_idx].valid;
    history->lvptEntry.target = lvpt[lvpt_idx].target;

    void* lvptHistory = static_cast<void*>(history);



    PIDPredictorHistory predict_record(seqNum, pc.instAddr(),
                                       predict, lvptHistory,
                                       tid, lvpt_idx
                                     );

    predict_record.targetPID = target;

    //predHist[tid].push_front(predict_record);
    predHist[tid].insert(std::pair<InstSeqNum,PIDPredictorHistory>
                        (seqNum, predict_record)
                        );

    DPRINTF(Capability, "[tid:%i]: [sn:%i]: History entry added."
            "predHist.size(): %i\n", tid, seqNum, predHist[tid].size());

    //Capture prediction Miss History
    if (!predict){
      predictorMissCount[lvpt_idx]++;
      //add it to the debug_function_calls
      // find the function which loaded a pointer
      Block fake;
      fake.payload = instPC;
      fake.req_szB = 1;
      UWord foundkey = 1;
      UWord foundval = 1;
      unsigned char found =
             VG_lookupFM(tc->FunctionSymbols,
                                  &foundkey, &foundval, (UWord)&fake );
     if (found) {
         Block* bk = (Block*)foundkey;
         std::string str =
            std::to_string(target.getPID()) +
                    "(" + std::to_string(predicted_pid.getPID()) + ")";
         predictorMissHistory[lvpt_idx][bk->name][instPC].push_back(str);
     }
    }

    update(instPC, target, tid, predict);

}

void
DefaultLVPT::update(Addr instPC,
                    const TheISA::PointerID &target,
                    ThreadID tid, bool predict
                   )
{

    unsigned lvpt_idx = getIndex(instPC, tid);

    assert(lvpt_idx < numEntries);

    // if prediction is true, we just update the localCtrs
      // std::cout << std::hex << "Update localCtrs: " <<
      // localCtrs[lvpt_idx].read() << std::endl;
      switch (localCtrs[lvpt_idx].read())
      {
        case 0x0:
          if (predict){ localCtrs[lvpt_idx].decrement(); }
          else        { localCtrs[lvpt_idx].increment(); }
          break;
        case 0x1:
          if (predict){ localCtrs[lvpt_idx].decrement(); }
          else        { localCtrs[lvpt_idx].increment(); }
          break;
        case 0x2:
          if (predict){ localCtrs[lvpt_idx].increment(); }
          else        { localCtrs[lvpt_idx].decrement(); }
          break;
        case 0x3:
          if (predict){ localCtrs[lvpt_idx].increment(); }
          else        { localCtrs[lvpt_idx].decrement(); }
          break;
        default:
          assert("localCtrs ivanlid value!");
      }



    // std::cout << std::hex << "Inst. " << instPC << " updated the LVPT." <<
    // " Before: " <<   lvpt[lvpt_idx].target << " After: " << target << "\n";

    localBiases[lvpt_idx] = target.getPID() - lvpt[lvpt_idx].target.getPID();
    lvpt[lvpt_idx].tid = tid;
    lvpt[lvpt_idx].valid = true;
    lvpt[lvpt_idx].target = target;
    lvpt[lvpt_idx].tag = getTag(instPC);

}


void
DefaultLVPT::squashAndUpdate(const InstSeqNum &squashed_sn,
                             const TheISA::PCState &pc,
                            TheISA::PointerID& corr_pid,
                            ThreadID tid
                            )
{

  DPRINTF(Capability, "[tid:%i]: Squashing all histories unitl: [sn:%i]"
          "\n", tid, squashed_sn);

    unsigned lvpt_idx = getIndex(pc.instAddr(), tid);

    History &pred_hist = predHist[tid];

    assert(lvpt_idx < numEntries);
    // first squash all histories afte the squashed_sn e.g squashed_sn = 604
    // then all histories > 604 wil be removed
    // this brings back the LVPT to the point right before the midpredicted PID

    squash(squashed_sn, tid);

    assert(!pred_hist.empty());
    // then we can update the LVPT with the right history

    auto pred_hist_it = pred_hist.find(squashed_sn);
    DPRINTF(Capability, "[tid:%i]: SEQ: [sn:%i] [sn:%i]"
            "\n", tid, pred_hist_it->second.seqNum ,squashed_sn);
    assert(pred_hist_it->second.seqNum == squashed_sn);

    LVPTHistory *history =
          static_cast<LVPTHistory*>(pred_hist_it->second.lvptEntryHistory);

    assert(pred_hist_it->second.lvptIdx == lvpt_idx);

    lvpt[lvpt_idx].tag = history->lvptEntry.tag;
    lvpt[lvpt_idx].tid = history->lvptEntry.tid;
    lvpt[lvpt_idx].valid = history->lvptEntry.valid;
    lvpt[lvpt_idx].target = history->lvptEntry.target;
    localCtrs[lvpt_idx].write(history->localCtrEntry.read());
    localBiases[lvpt_idx] = history->localBiasEntry;

    delete history;

    pred_hist.erase(pred_hist_it);
    //now update with correct pid
    update(pc.instAddr(), corr_pid, tid, true);

}


void
DefaultLVPT::squash(const InstSeqNum &squashed_sn, ThreadID tid)
{
    History &pred_hist = predHist[tid];

    // bring back the LVPT to a state right before the squash

    for (auto pred_hist_it = pred_hist.cbegin(),
         next_it = pred_hist_it;
        pred_hist_it != pred_hist.cend();
        pred_hist_it = next_it)
    {
       ++next_it;
       if (pred_hist_it->second.seqNum > squashed_sn)
       {
          LVPTHistory *history =
             static_cast<LVPTHistory*>(pred_hist_it->second.lvptEntryHistory);

          uint64_t lvpt_idx = pred_hist_it->second.lvptIdx;

          lvpt[lvpt_idx].tag = history->lvptEntry.tag;
          lvpt[lvpt_idx].tid = history->lvptEntry.tid;
          lvpt[lvpt_idx].valid = history->lvptEntry.valid;
          lvpt[lvpt_idx].target = history->lvptEntry.target;
          localCtrs[lvpt_idx].write(history->localCtrEntry.read());
          localBiases[lvpt_idx] = history->localBiasEntry;

          delete history;

          DPRINTF(Capability, "[tid:%i]: Removing history for [sn:%i] "
                 "PC %#x.\n", tid, pred_hist_it->second.seqNum,
                 pred_hist_it->second.pc);

          pred_hist.erase(pred_hist_it);

       }

    }


}

void
DefaultLVPT::updatePIDHistory(const InstSeqNum &done_sn, ThreadID tid)
{

    History &pred_hist = predHist[tid];

    DPRINTF(Capability, "[tid:%i]: Committing predictions until "
            "[sn:%lli].\n", tid, done_sn);



    for (auto pred_hist_it = pred_hist.cbegin(),
         next_it = pred_hist_it;
        pred_hist_it != pred_hist.cend();
        pred_hist_it = next_it)
    {
       ++next_it;
       if (pred_hist_it->second.seqNum <= done_sn)
       {
         LVPTHistory *history =
             static_cast<LVPTHistory*>(pred_hist_it->second.lvptEntryHistory);

        //update these two counters with real predictions
        // if the prediction is true increase confidence level otw. decrease

        if (pred_hist_it->second.predResult){
            confLevel[pred_hist_it->second.lvptIdx].increment();
        }
        else{
            confLevel[pred_hist_it->second.lvptIdx].decrement();
        }

        if (pred_hist_it->second.targetPID.getPID() != 0){
            localPointerPredictor[pred_hist_it->second.lvptIdx].increment();
        }
        else {
            localPointerPredictor[pred_hist_it->second.lvptIdx].decrement();
        }

         delete history;

         DPRINTF(Capability, "[tid:%i]: Removing history for [sn:%i] "
                 "PC %#x.\n", tid, pred_hist_it->second.seqNum,
                 pred_hist_it->second.pc);

         pred_hist.erase(pred_hist_it);

       }

    }
}
