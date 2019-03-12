/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
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
 * Authors: Gabe Black
 */

#ifndef __ARCH_X86_TYPES_HH__
#define __ARCH_X86_TYPES_HH__

#include <bitset>
#include <iostream>
#include <cmath>
#include "arch/generic/types.hh"
#include "base/bitunion.hh"
#include "base/cprintf.hh"
#include "base/types.hh"
#include "sim/serialize.hh"
#include "debug/Capability.hh"

namespace X86ISA
{


    class Range
    {
        private:
            uint64_t mLow;
            uint64_t mHigh;

        public:

            explicit Range(uint64_t item):
            mLow(item),mHigh(item)  // [item,item]
            {

            }

            Range(uint64_t low, uint64_t high):  // [low,high]
            mLow(low), mHigh(high)
            {

            }

            bool operator < (const Range& rhs) const
            {
                if (mLow <= rhs.mLow && mHigh >= rhs.mLow)
                {
                  return true;
                }

                return false;
            } // operator<

            Range(const Range& _range)
            {
                this->mLow  = _range.low();
                this->mHigh = _range.high();
            }

            uint64_t low() const { return mLow; }
            uint64_t high() const { return mHigh; }

    }; // class Range

    class CacheEntry {

        public:
            int tag;
            int valid;
            int dirty;
            uint64_t lruAge;
            uint64_t t_access_nums;
            uint64_t t_num_replaced;

        public:
            CacheEntry():
                tag(-1),valid(1), dirty(0),
                lruAge(1),t_access_nums(0), t_num_replaced(0)
            {}

    } ;

    class LRUCapabilityCache
    {

    private:
        CacheEntry*   TheCache;

        uint64_t                     NumWays;
        uint64_t                     NumSets;
        uint64_t                     CacheSize;
        uint64_t                     CacheBlockSize;
        uint64_t                     NumEntriesInCache;
        uint64_t                     BitsPerBlock;
        uint64_t                     BitsPerSet;
        uint64_t                     ShiftAmount;

        //TODO: move these to gem5 stats
        uint64_t                     total_accesses;
        uint64_t                     total_hits;
        uint64_t                     total_misses;

    public:
        LRUCapabilityCache(uint64_t _num_ways,
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

                TheCache = new CacheEntry[NumEntriesInCache];

            }

        ~LRUCapabilityCache(){
            delete [] TheCache;
        }

        void LRUCache_Access(Addr v_addr) {
            //TODO: stats
            total_accesses = total_accesses + 1;

            Addr thisIsTheTag = v_addr >> (ShiftAmount); //tag of the VA

            //Extract the set from the VA
            Addr thisIsTheSet =
                    ((v_addr -
                          ( thisIsTheTag <<
                            (BitsPerBlock + BitsPerSet))) >> BitsPerBlock
                    );
            int setLoopLow = thisIsTheSet * NumWays;
            int setLoopHigh = setLoopLow + (NumWays - 1);
            int j;
            int hit = 0;
            for(j = setLoopLow; j <= setLoopHigh; j++){

                if(TheCache[j].tag == thisIsTheTag){

                    //We have a hit!
                    hit = 1;
                    total_hits = total_hits + 1;
                    int k;
                    //Increase the age of everything
                    for( k = setLoopLow; k <= setLoopHigh; k++){
                        TheCache[k].lruAge++;
                    }

                    TheCache[j].lruAge = 0;
                    TheCache[j].t_access_nums++;
                    break;

                }
            }

            if(hit == 0){

                total_misses = total_misses + 1;

                int m;
                int highestAge = 0;
                int highestSpot = 0;
                //Loop through the set and find the oldest element
                for(m = setLoopLow; m <= setLoopHigh; m++){

                    if(TheCache[m].lruAge>highestAge){
                        highestAge = TheCache[m].lruAge;
                        highestSpot = m;
                    }

                }

                //Replace the oldest element with the new tag
                TheCache[highestSpot].tag = thisIsTheTag;

                //Increase the age of each element
                for(m=setLoopLow;m<=setLoopHigh;m++){
                    TheCache[m].lruAge++;
                }

                TheCache[highestSpot].lruAge = 0;
                TheCache[highestSpot].t_access_nums++;
                TheCache[highestSpot].t_num_replaced++;
                //tl_assert(n_guest_instrs < g_guest_instrs_executed);
                // ULong miss_inst_diff = n_guest_instrs - n_guest_instrs_prev;
                // n_guest_instrs_prev = n_guest_instrs;


                // uint index = miss_inst_diff/100;

                //  if (index < CACHE_MISS_BINS_N)
                //     CacheMissHist[index]++;


            }

        }

      void LRUCachePrintStats() {
        DPRINTF(Capability, "Capability Cache Stats: %llu, %llu, %llu, %f \n",
        total_accesses, total_hits, total_misses,
        (double)total_hits/total_accesses
        );
      }

    };


    class PointerID
    {
    private:
        uint64_t id;
    public:

        PointerID(){

        }

        PointerID(uint64_t _id){

            id = _id;
        }

        ~PointerID()
        {

        }

        bool operator != (const PointerID& _pid){
            return (id != _pid.id);
        }

        bool operator == (const PointerID& _pid){
            return (id == _pid.id);
        }

            // A better implementation of operator=
        PointerID& operator = (const PointerID& _pid)
            {
                // self-assignment guard
                if (this == &_pid)
                    return *this;

                // do the copy
                this->id = _pid.id;

                // return the existing object so we can chain this operator
                return *this;
        }

        bool operator < (const PointerID& rhs) const {

            return (this->id < rhs.id);
        }

        PointerID operator + (uint64_t _val)
        {
            this->id += _val;
            PointerID temp(*this);
            //temp.id += _val;

            return temp;

        }

        PointerID& operator += (const uint64_t& _rhs)
        {
            this->id += _rhs;
            return *this;
        }

        PointerID(const PointerID& _PID)
        {
            this->id = _PID.id;
        }

        void        setPID(uint64_t _id) { id = _id; }
        uint64_t    getPID() const { return id; }


    };

    inline static std::ostream &
            operator << (std::ostream & os, const PointerID & _pid)
    {
        ccprintf(os, "PID(%llu)",
                _pid.getPID()
                );
        return os;
    }

    class EmitterID: public PointerID
    {
    public:
        EmitterID(uint64_t _emitter_ip): PointerID(_emitter_ip){}
        EmitterID(): PointerID(){}
        ~EmitterID(){}
        uint64_t getEID() {return getPID();}

    };

    class Capability
    {
        private:
            uint64_t begin;
            uint64_t end;
            uint64_t size;
            std::bitset<16> CSR;
            uint64_t  stack_addr;


        public:
            Capability (){
                CSR.reset();
            }

            Capability(uint64_t _size)
            {
                size = _size;
                CSR.reset();
                //CSR.set(0, 1);
            }
            ~Capability(){

            }

            // A better implementation of operator=
            Capability& operator = (const Capability &cap)
            {
                // self-assignment guard
                if (this == &cap)
                    return *this;

                // do the copy
                begin = cap.begin;
                end = cap.end;
                size = cap.size;
                stack_addr = cap.stack_addr;
                CSR = cap.CSR;

                // return the existing object so we can chain this operator
                return *this;
            }

            Capability(const Capability& _cap)
            {
                begin = _cap.begin;
                end = _cap.end;
                size = _cap.size;
                stack_addr = _cap.stack_addr;
                CSR = _cap.CSR;
            }


            void            setBaseAddr(uint64_t _addr){  begin = _addr;}
            void            setSize (uint64_t _size){ size = _size;}
            uint64_t        getSize(){ return size; }
            uint64_t        getEndAddr(){ return begin + size;}
            uint64_t        getBaseAddr(){ return begin;}
            void            setCSRBit(int bit_num){ CSR.set(bit_num, 1);}
            void            clearCSRBit(int bit_num){ CSR.set(bit_num, 0);}
            bool            getCSRBit(int bit_num){ return CSR.test(bit_num);}
            std::bitset<16> getCSR(){return CSR;}
            void            setStackAddr(uint64_t _stack_addr){
                                                  stack_addr = _stack_addr; }
            uint64_t        getStackAddr(){ return stack_addr; }
            void            reset(){ CSR.reset(); };

            bool contains(Addr _addr){
                end = begin + size;
                if (_addr >= begin && _addr <= end)
                    return true;
                else
                    return false;
            }

    };

    enum StackType{
        NONE,
        AP_MALLOC_STACK,
        AP_CALLOC_STACK,
        DP_FREE_STACK,
        STACK_W_RTT,
        STACK_WO_RTT
    };

    class StackID : public PointerID
    {

    public:
        typedef std::map<RegIndex,
                    std::tuple<EmitterID, PointerID, Capability>>
                                                      RegisterTrackingTable;
        typedef std::tuple<EmitterID, PointerID, Capability> RTTEntry;
    private:
        RegisterTrackingTable RTT ;
        std::vector<Addr> stack_pointers_addr ;
        StackType stack_type;
        Addr stack_addr;
    public:
        StackID(): PointerID() {}
        StackID(uint64_t _sid): PointerID(_sid){}
        StackID(Addr _stack_addr, StackType _stack_type):
            PointerID(), stack_type(_stack_type), stack_addr(_stack_addr)
        {}

        ~StackID(){}

        uint64_t    getSID() const { return getPID();}
        void        setStackType(StackType _stack_type){
                                              stack_type = _stack_type;}
        void        setStackAddr(Addr _stack_addr){ stack_addr = _stack_addr; }
        StackType   getStackType() const {return stack_type;}
        Addr        getStackAddr() const {return stack_addr;}



        StackID& operator = (const StackID& _sid)
            {
                // self-assignment guard
                if (this == &_sid)
                    return *this;

                // do the copy
                this->stack_addr = _sid.stack_addr;
                this->stack_type = _sid.stack_type;
                this->RTT                   = _sid.RTT;
                this->stack_pointers_addr   = _sid.stack_pointers_addr;
                this->setPID(_sid.getPID());

                // return the existing object so we can chain this operator
                return *this;
        }

        StackID operator + (uint64_t _val)
        {
            setPID(getPID() + _val);
            StackID temp(*this);
            //temp.id += _val;

            return temp;

        }

        StackID(const StackID& _SID) : PointerID(_SID.getPID())
        {
            this->stack_addr            = _SID.stack_addr;
            this->stack_type            = _SID.stack_type;
            this->RTT                   = _SID.RTT;
            this->stack_pointers_addr   = _SID.stack_pointers_addr;
        }




    };

    inline static std::ostream &
            operator << (std::ostream & os, const StackID & _sid)
    {
        ccprintf(os, "STACK CREATED FOR STACK ADDR: %#lx  TYPE: %d ID: %s\n",
                _sid.getStackAddr(),
                _sid.getStackType(),
                _sid.getSID()
                );
        return os;
    }

    enum CheckType {
        BOUNDS = 0,
        READ = 1,
        WRITE = 2,
        EXECUTE = 3,
        FREE = 4,
        ALLOC_SIZE = 5,
        ALLOC_ADDR = 6,
        AP_MALLOC = 7,
        STACK = 8,
        DP = 9,
        AP_BOUNDS_INJECT        = 0xd,
        AP_MALLOC_BASE_COLLECT  = 0xb,
        AP_MALLOC_SIZE_COLLECT  = 0xc,
        AP_FREE_CALL            = 0xe,
        AP_FREE_RET             = 0xf,
        AP_CALLOC_BASE_COLLECT  = 0x10,
        AP_CALLOC_SIZE_COLLECT  = 0x11,
        AP_REALLOC_BASE_COLLECT = 0x12,
        AP_REALLOC_SIZE_COLLECT  = 0x13
    };


    static inline std::string
    CheckTypeToStr(CheckType type)
    {
        switch (type) {
          case BOUNDS:
            return "Bounds Check";
          case READ:
            return "Read Permission Check";
          case WRITE:
            return "Write Permission Check";
          case EXECUTE:
            return "Execute Permission Check";
          case FREE:
            return "Free Check";
          case ALLOC_ADDR:
            return "ALLOC ADDR PICK";
          case ALLOC_SIZE:
            return "ALLOC SIZE PICK";
          case AP_MALLOC:
            return "AP_MALLOC";
          case STACK:
            return "STACK";
          case DP:
            return "DP";
          case AP_MALLOC_BASE_COLLECT:
            return "AP_MALLOC_BASE_COLLECT";
          case AP_MALLOC_SIZE_COLLECT:
            return "AP_MALLOC_SIZE_COLLECT";
          case AP_BOUNDS_INJECT:
            return "AP_BOUNDS_INJECT";
          case AP_FREE_CALL:
            return "AP_FREE_CALL";
          case AP_FREE_RET:
            return "AP_FREE_RET";
          case AP_CALLOC_BASE_COLLECT:
            return "AP_CALLOC_BASE_COLLECT";
          case AP_CALLOC_SIZE_COLLECT:
            return "AP_CALLOC_SIZE_COLLECT";
          case AP_REALLOC_BASE_COLLECT:
            return "AP_REALLOC_BASE_COLLECT";
          case AP_REALLOC_SIZE_COLLECT:
            return "AP_REALLOC_SIZE_COLLECT";
          default:
          {
            assert(0);
            return "Unrecognized Check!";
          }
        }
    }

    //This really determines how many bytes are passed to the decoder.
    typedef uint64_t MachInst;

    enum Prefixes {
        NoOverride,
        ESOverride,
        CSOverride,
        SSOverride,
        DSOverride,
        FSOverride,
        GSOverride,
        RexPrefix,
        OperandSizeOverride,
        AddressSizeOverride,
        Lock,
        Rep,
        Repne,
        Vex2Prefix,
        Vex3Prefix,
        XopPrefix,
    };

    BitUnion8(LegacyPrefixVector)
        Bitfield<7, 4> decodeVal;
        Bitfield<7> repne;
        Bitfield<6> rep;
        Bitfield<5> lock;
        Bitfield<4> op;
        Bitfield<3> addr;
        //There can be only one segment override, so they share the
        //first 3 bits in the legacyPrefixes bitfield.
        Bitfield<2,0> seg;
    EndBitUnion(LegacyPrefixVector)

    BitUnion8(ModRM)
        Bitfield<7,6> mod;
        Bitfield<5,3> reg;
        Bitfield<2,0> rm;
    EndBitUnion(ModRM)

    BitUnion8(Sib)
        Bitfield<7,6> scale;
        Bitfield<5,3> index;
        Bitfield<2,0> base;
    EndBitUnion(Sib)

    BitUnion8(Rex)
        //This bit doesn't mean anything according to the ISA, but in
        //this implementation, it being set means an REX prefix was present.
        Bitfield<6> present;
        Bitfield<3> w;
        Bitfield<2> r;
        Bitfield<1> x;
        Bitfield<0> b;
    EndBitUnion(Rex)

    BitUnion8(Vex2Of3)
        // Inverted bits from the REX prefix.
        Bitfield<7> r;
        Bitfield<6> x;
        Bitfield<5> b;
        // Selector for what would be two or three byte opcode types.
        Bitfield<4, 0> m;
    EndBitUnion(Vex2Of3)

    BitUnion8(Vex3Of3)
        // Bit from the REX prefix.
        Bitfield<7> w;
        // Inverted extra register index.
        Bitfield<6, 3>  v;
        // Vector length specifier.
        Bitfield<2> l;
        // Implied 66, F2, or F3 opcode prefix.
        Bitfield<1, 0> p;
    EndBitUnion(Vex3Of3)

    BitUnion8(Vex2Of2)
        // Inverted bit from the REX prefix.
        Bitfield<7> r;
        // Inverted extra register index.
        Bitfield<6, 3>  v;
        // Vector length specifier
        Bitfield<2> l;
        // Implied 66, F2, or F3 opcode prefix.
        Bitfield<1, 0> p;
    EndBitUnion(Vex2Of2)

    BitUnion8(VexInfo)
        // Extra register index.
        Bitfield<6, 3> v;
        // Vector length specifier.
        Bitfield<2> l;
        // Whether the VEX prefix was used.
        Bitfield<0> present;
    EndBitUnion(VexInfo)

    enum OpcodeType {
        BadOpcode,
        OneByteOpcode,
        TwoByteOpcode,
        ThreeByte0F38Opcode,
        ThreeByte0F3AOpcode,
    };

    static inline const char *
    opcodeTypeToStr(OpcodeType type)
    {
        switch (type) {
          case BadOpcode:
            return "bad";
          case OneByteOpcode:
            return "one byte";
          case TwoByteOpcode:
            return "two byte";
          case ThreeByte0F38Opcode:
            return "three byte 0f38";
          case ThreeByte0F3AOpcode:
            return "three byte 0f3a";
          default:
            return "unrecognized!";
        }
    }

    BitUnion8(Opcode)
        Bitfield<7,3> top5;
        Bitfield<2,0> bottom3;
    EndBitUnion(Opcode)

    BitUnion8(OperatingMode)
        Bitfield<3> mode;
        Bitfield<2,0> submode;
    EndBitUnion(OperatingMode)

    enum X86Mode {
        LongMode,
        LegacyMode
    };

    enum X86SubMode {
        SixtyFourBitMode,
        CompatabilityMode,
        ProtectedMode,
        Virtual8086Mode,
        RealMode
    };

    //The intermediate structure used by the x86 decoder.
    struct ExtMachInst
    {
        void reset() {
            memset(static_cast<void *>(this), 0, sizeof(*this));
        }

        //Prefixes
        LegacyPrefixVector legacy;
        Rex rex;
        VexInfo vex;

        //This holds all of the bytes of the opcode
        struct
        {
            OpcodeType type;
            //The main opcode byte. The highest addressed byte in the opcode.
            Opcode op;
        } opcode;
        //Modifier bytes
        ModRM modRM;
        Sib sib;
        //Immediate fields
        uint64_t immediate;
        uint64_t displacement;

        //The effective operand size.
        uint8_t opSize;
        //The effective address size.
        uint8_t addrSize;
        //The effective stack size.
        uint8_t stackSize;
        //The size of the displacement
        uint8_t dispSize;

        //Mode information
        OperatingMode mode;
    };

    inline static std::ostream &
        operator << (std::ostream & os, const ExtMachInst & emi)
    {
        ccprintf(os, "\n{\n\tleg = %#x,\n\trex = %#x,\n\t"
                     "vex/xop = %#x,\n\t"
                     "op = {\n\t\ttype = %s,\n\t\top = %#x,\n\t\t},\n\t"
                     "modRM = %#x,\n\tsib = %#x,\n\t"
                     "immediate = %#x,\n\tdisplacement = %#x\n\t"
                     "dispSize = %d}\n",
                     (uint8_t)emi.legacy, (uint8_t)emi.rex,
                     (uint8_t)emi.vex,
                     opcodeTypeToStr(emi.opcode.type), (uint8_t)emi.opcode.op,
                     (uint8_t)emi.modRM, (uint8_t)emi.sib,
                     emi.immediate, emi.displacement, emi.dispSize);
        return os;
    }

    inline static bool
        operator == (const ExtMachInst &emi1, const ExtMachInst &emi2)
    {
        if (emi1.legacy != emi2.legacy)
            return false;
        if (emi1.rex != emi2.rex)
            return false;
        if (emi1.vex != emi2.vex)
            return false;
        if (emi1.opcode.type != emi2.opcode.type)
            return false;
        if (emi1.opcode.op != emi2.opcode.op)
            return false;
        if (emi1.modRM != emi2.modRM)
            return false;
        if (emi1.sib != emi2.sib)
            return false;
        if (emi1.immediate != emi2.immediate)
            return false;
        if (emi1.displacement != emi2.displacement)
            return false;
        if (emi1.mode != emi2.mode)
            return false;
        if (emi1.opSize != emi2.opSize)
            return false;
        if (emi1.addrSize != emi2.addrSize)
            return false;
        if (emi1.stackSize != emi2.stackSize)
            return false;
        if (emi1.dispSize != emi2.dispSize)
            return false;
        return true;
    }

    class PCState : public GenericISA::UPCState<MachInst>
    {
      protected:
        typedef GenericISA::UPCState<MachInst> Base;

        uint8_t _size;

      public:
        void
        set(Addr val)
        {
            Base::set(val);
            _size = 0;
        }

        PCState() {}
        PCState(Addr val) { set(val); }

        void
        setNPC(Addr val)
        {
            Base::setNPC(val);
            _size = 0;
        }

        uint8_t size() const { return _size; }
        void size(uint8_t newSize) { _size = newSize; }

        bool
        branching() const
        {
            return (this->npc() != this->pc() + size()) ||
                   (this->nupc() != this->upc() + 1);
        }

        void
        advance()
        {
            Base::advance();
            _size = 0;
        }

        void
        uEnd()
        {
            Base::uEnd();
            _size = 0;
        }

        void
        serialize(CheckpointOut &cp) const
        {
            Base::serialize(cp);
            SERIALIZE_SCALAR(_size);
        }

        void
        unserialize(CheckpointIn &cp)
        {
            Base::unserialize(cp);
            UNSERIALIZE_SCALAR(_size);
        }
    };

}

namespace std {
    template<>
    struct hash<X86ISA::ExtMachInst> {
        size_t operator()(const X86ISA::ExtMachInst &emi) const {
            return (((uint64_t)emi.legacy << 48) |
                    ((uint64_t)emi.rex << 40) |
                    ((uint64_t)emi.vex << 32) |
                    ((uint64_t)emi.modRM << 24) |
                    ((uint64_t)emi.sib << 16) |
                    ((uint64_t)emi.opcode.type << 8) |
                    ((uint64_t)emi.opcode.op)) ^
                    emi.immediate ^ emi.displacement ^
                    emi.mode ^
                    emi.opSize ^ emi.addrSize ^
                    emi.stackSize ^ emi.dispSize;
        };
    };
}

// These two functions allow ExtMachInst to be used with SERIALIZE_SCALAR
// and UNSERIALIZE_SCALAR.
template <>
void
paramOut(CheckpointOut &cp, const std::string &name,
         const X86ISA::ExtMachInst &machInst);
template <>
void
paramIn(CheckpointIn &cp, const std::string &name,
        X86ISA::ExtMachInst &machInst);

#endif // __ARCH_X86_TYPES_HH__
