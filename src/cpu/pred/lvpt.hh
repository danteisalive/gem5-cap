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

#ifndef __CPU_PRED_LVPT_HH__
#define __CPU_PRED_LVPT_HH__

#include <algorithm>
#include <numeric>

#include "arch/types.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/inst_seq.hh"
#include "cpu/pred/sat_counter.hh"
#include "cpu/static_inst.hh"

class DefaultLVPT
{
  private:
    struct LVPTEntry
    {
        LVPTEntry()
        {
          this->tag = 0;
          this->target = TheISA::PointerID(0);
          this->valid = false;
        }

        /** The entry's tag. */
        Addr tag;

        /** The entry's target. */
        TheISA::PointerID target{0};

        /** The entry's thread id. */
        ThreadID tid;

        /** Whether or not the entry is valid. */
        bool valid;
    };

  public:
    /** Creates a LVPT with the given number of entries, number of bits per
     *  tag, and instruction offset amount.
     *  @param numEntries Number of entries for the LVPT.
     *  @param tagBits Number of bits for each tag in the LVPT.
     *  @param instShiftAmt Offset amount for instructions to ignore alignment.
     */
    DefaultLVPT(unsigned numEntries, unsigned tagBits,
               unsigned instShiftAmt, unsigned numThreads);

    void reset();

/** Looks up an address in the LVPT. Must call valid() first on the address.
     *  @param inst_PC The address of the branch to look up.
     *  @param tid The thread id.
     *  @return Returns the target of the branch.
     */
    TheISA::PointerID lookup(StaticInstPtr inst, Addr instPC, ThreadID tid);

    /** Checks if a branch is in the LVPT.
     *  @param inst_PC The address of the branch to look up.
     *  @param tid The thread id.
     *  @return Whether or not the branch exists in the LVPT.
     */
    bool valid(Addr instPC, ThreadID tid);

    /** Updates the LVPT with the target of a branch.
     *  @param inst_PC The address of the branch being updated.
     *  @param target_PC The target address of the branch.
     *  @param tid The thread id.
     */
    void updateAndSnapshot(
                        TheISA::PCState pc,
                        const InstSeqNum &seqNum,
                        Addr instPC,
                        const TheISA::PointerID &target,
                        const TheISA::PointerID &predicted_pid,
                        ThreadID tid, bool predict,
                        ThreadContext* tc
                      );

    void update(Addr instPC,
                const TheISA::PointerID &target,
                ThreadID tid, bool predict
               );

    void squashAndUpdate(const InstSeqNum &squashed_sn,
                const TheISA::PCState &pc,
                TheISA::PointerID& corr_pid, ThreadID tid);

    void squash(const InstSeqNum &squashed_sn, ThreadID tid);

    void updatePIDHistory(const InstSeqNum &done_sn, ThreadID tid);

    float getAverageConfidenceLevel() {

        float avg = 0.0;

        for (size_t i = 0; i < numEntries; i++) {
          avg += float(confLevel[i].read());
        }

        return avg/numEntries;
    }

    void dumpStat(){
      auto v = predictorMissCount;
      std::vector<std::size_t> result(v.size());
      std::iota(std::begin(result), std::end(result), 0);
      std::sort(std::begin(result), std::end(result),
              [&v](const uint64_t & lhs, const uint64_t & rhs)
              {
                  return v[lhs] > v[rhs];
              }
      );

      for (auto &idx : result) {
            std::cout << std::dec << "INDEX[" << idx << "]: " << std::endl;
            for (auto& elem1 : predictorMissHistory[idx]){
                std::cout << "FUNCTION[" <<elem1.first << "]: " << std::endl;
                for (auto& elem2 : elem1.second) {
                    std::cout << std::hex <<"["<< elem2.first << "] => ";
                    for (auto& elem3 : elem2.second)
                        std::cout << std::dec << elem3 << ",";
                    std::cout << std::endl;
                }
            }
      }

    }

  private:
    struct PIDPredictorHistory {
        /**
         * Makes a predictor history struct that contains any
         * information needed to update the predictor, BTB, and RAS.
         */
        PIDPredictorHistory(const InstSeqNum &seq_num, Addr instPC,
                         bool pred_result, void *history,
                         ThreadID _tid, uint64_t lvpt_idx)
            : seqNum(seq_num), pc(instPC), lvptEntryHistory(history),
              targetPID(TheISA::PointerID(0)),
              tid(_tid), predResult(pred_result), lvptIdx(lvpt_idx)
        {}

        bool operator==(const PIDPredictorHistory &entry) const {
            return this->seqNum == entry.seqNum;
        }

        /** The sequence number for the predictor history entry. */
        InstSeqNum seqNum;

        /** The PC associated with the sequence number. */
        Addr pc;

        /** Pointer to the history object passed back from the branch
         * predictor.  It is used to update or restore state of the
         * branch predictor.
         */
        void *lvptEntryHistory;

        TheISA::PointerID targetPID{0};

        /** The thread id. */
        ThreadID tid;

        /** Whether or not it was predicted taken. */
        bool predResult;

        uint64_t lvptIdx;


    };

    struct LVPTHistory {
        LVPTEntry   lvptEntry;
        int         localBiasEntry;
        SatCounter  localCtrEntry;
    };

    typedef std::map<InstSeqNum,PIDPredictorHistory> History;

    typedef std::map<std::string, std::map<uint64_t, std::vector<std::string>>>
                                                EntryMissHistoty;

    std::vector<History> predHist;
    /** Returns the index into the LVPT, based on the branch's PC.
     *  @param inst_PC The branch to look up.
     *  @return Returns the index into the LVPT.
     */
    inline unsigned getIndex(Addr instPC, ThreadID tid);

    /** Returns the tag bits of a given address.
     *  @param inst_PC The branch's address.
     *  @return Returns the tag bits.
     */
    inline Addr getTag(Addr instPC);

    /** The actual LVPT. */
    std::vector<LVPTEntry>  lvpt;
    std::vector<int>        localBiases;
    std::vector<SatCounter> localCtrs;
    std::vector<SatCounter> confLevel;
    std::vector<SatCounter> localPointerPredictor;
    std::vector<EntryMissHistoty> predictorMissHistory;
    std::vector<uint64_t> predictorMissCount;;

    /** The number of entries in the LVPT. */
    unsigned numEntries;

    /** The index mask. */
    unsigned idxMask;

    /** The number of tag bits per entry. */
    unsigned tagBits;

    /** The tag mask. */
    unsigned tagMask;

    /** Number of bits to shift PC when calculating index. */
    unsigned instShiftAmt;

    /** Number of bits to shift PC when calculating tag. */
    unsigned tagShiftAmt;

    /** Log2 NumThreads used for hashing threadid */
    unsigned log2NumThreads;
};

#endif // __CPU_PRED_LVPT_HH__
