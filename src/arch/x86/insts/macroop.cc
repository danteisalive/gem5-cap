/*
 * macroop.cc
 *
 * Created on: May 11, 2018
 * Author: rasool
 */

 #include "macroop.hh"

 #include <iostream>

 #include "arch/x86/faults.hh"
 #include "arch/x86/generated/decoder.hh"
 #include "arch/x86/isa_traits.hh"
 #include "cpu/reg_class.hh"
 #include "enums/OpClass.hh"
 #include "debug/X86.hh"



namespace X86ISA
{

bool MacroopBase::filterInst(ThreadContext * tc) {

    return false;
}

void MacroopBase::updatePointerTracker(ThreadContext * tc, PCState &nextPC)
{
      #define ENABLE_POINTER_TRACKER_DEBUG 0
      if (_isInjected) panic("tracking an injected macroop!");

      // this is probably a a little late but still can be effective
      if (tc->ExeStopTracking) return;
      // its like we are executing microps here
      for (size_t i = 0; i < numMicroops; i++) {
          const StaticInstPtr si = microops[i];


          // //let's see whether this is a heap access or not
          // This should be done before chaning pointer track table state
          //  to our knowledge:
          // (base >=0 && base < 16) and base == 32 could be used for
          //  addresing.
          // if the base reg is RSP then it's not a heap access
           if (si->isLoad() && !si->checked)
           {
                int base = si->getBase();
                if ((base != INTREG_RSP) &&
                    ((base < X86ISA::NUM_INTREGS) ||
                     (base == X86ISA::NUM_INTREGS + 7)))
                {
                    if (tc->PointerTrackerTable[base] != TheISA::PointerID(0))
                    {
                       si->uop_pid = tc->PointerTrackerTable[base];
                       if (ENABLE_POINTER_TRACKER_DEBUG)
                       {std::cout << "LD NEED AN INJECTION: " <<
                            si->uop_pid << " " << tc->CapRegsFile.size() <<
                            nextPC <<
                            std::endl;}
                    }

                }
           }
           else if (si->isStore() && !si->checked)
           {

               int base = si->getBase();
               if ((base != INTREG_RSP) &&
                   ((base < X86ISA::NUM_INTREGS) ||
                    (base == X86ISA::NUM_INTREGS + 7)))
               {
                   if (tc->PointerTrackerTable[base] != TheISA::PointerID(0))
                   {
                      si->uop_pid = tc->PointerTrackerTable[base];
                      if (ENABLE_POINTER_TRACKER_DEBUG){
                        std::cout << "ST NEED AN INJECTION: " <<
                             si->uop_pid << " " << tc->CapRegsFile.size() <<
                             nextPC <<
                             std::endl;
                      }
                   }

               }
           }


          if ((si->getName().compare("and") == 0)){

              if (si->destRegIdx(0).isIntReg() &&
                  si->srcRegIdx(0).isIntReg()  &&
                  si->srcRegIdx(1).isIntReg())
              {
                  X86ISA::IntRegIndex   src1 = (X86ISA::IntRegIndex)
                                                si->srcRegIdx(0).index();
                  X86ISA::IntRegIndex   src2 = (X86ISA::IntRegIndex)
                                                si->srcRegIdx(1).index();
                  X86ISA::IntRegIndex   dest = (X86ISA::IntRegIndex)
                                                si->destRegIdx(0).index();
                  if (dest < X86ISA::INTREG_RAX ||
                      dest > X86ISA::NUM_INTREGS + 15)
                        return;
                  if (src1 < X86ISA::INTREG_RAX ||
                      src1 > X86ISA::NUM_INTREGS + 15)
                        return;
                  if (src2 < X86ISA::INTREG_RAX ||
                      src2 > X86ISA::NUM_INTREGS + 15)
                        return;

                  TheISA::PointerID _pid_src1 = tc->PointerTrackerTable[src1];
                  TheISA::PointerID _pid_src2 = tc->PointerTrackerTable[src2];

                  if (src1 == src2)
                  {
                    tc->PointerTrackerTable[dest] = _pid_src1;
                  }
                  else
                  {
                    if ( _pid_src1 != TheISA::PointerID(0) &&
                         _pid_src2 == TheISA::PointerID(0))
                    {
                         tc->PointerTrackerTable[dest] = _pid_src1;
                    }
                    else if ( _pid_src1 == TheISA::PointerID(0) &&
                              _pid_src2 != TheISA::PointerID(0))
                    {
                         tc->PointerTrackerTable[dest] = _pid_src2;
                    }
                    else
                    {
                         tc->PointerTrackerTable[dest] = TheISA::PointerID(0);
                    }
                  }
               }
          }
          else if ((si->getName().compare("xor") == 0)){
            if (si->destRegIdx(0).isIntReg() &&
                si->srcRegIdx(0).isIntReg()  &&
                si->srcRegIdx(1).isIntReg())
            {
                X86ISA::IntRegIndex   src1 = (X86ISA::IntRegIndex)
                                            si->srcRegIdx(0).index();
                X86ISA::IntRegIndex   src2 = (X86ISA::IntRegIndex)
                                            si->srcRegIdx(1).index();
                X86ISA::IntRegIndex   dest = (X86ISA::IntRegIndex)
                                            si->destRegIdx(0).index();

                if (dest < X86ISA::INTREG_RAX ||
                    dest >= X86ISA::NUM_INTREGS + 15)
                      return;
                if (src1 < X86ISA::INTREG_RAX ||
                    src1 >= X86ISA::NUM_INTREGS + 15)
                      return;
                if (src2 < X86ISA::INTREG_RAX ||
                    src2 >= X86ISA::NUM_INTREGS + 15)
                      return;

                tc->PointerTrackerTable[dest] = TheISA::PointerID(0);
            }

          }
          else if ((si->getName().compare("sub") == 0) ||
                   (si->getName().compare("sbb") == 0))
          {
              if (si->destRegIdx(0).isIntReg() &&
                  si->srcRegIdx(0).isIntReg()  &&
                  si->srcRegIdx(1).isIntReg())
              {
                  X86ISA::IntRegIndex   src1 = (X86ISA::IntRegIndex)
                                                si->srcRegIdx(0).index();
                  X86ISA::IntRegIndex   src2 = (X86ISA::IntRegIndex)
                                                si->srcRegIdx(1).index();
                  X86ISA::IntRegIndex   dest = (X86ISA::IntRegIndex)
                                                si->destRegIdx(0).index();

                  if (dest < X86ISA::INTREG_RAX ||
                      dest >= X86ISA::NUM_INTREGS + 15)
                        return;
                  if (src1 < X86ISA::INTREG_RAX ||
                      src1 >= X86ISA::NUM_INTREGS + 15)
                        return;
                  if (src2 < X86ISA::INTREG_RAX ||
                      src2 >= X86ISA::NUM_INTREGS + 15)
                        return;

                  TheISA::PointerID _pid_src1 = tc->PointerTrackerTable[src1];
                  TheISA::PointerID _pid_src2 = tc->PointerTrackerTable[src2];

                  if (_pid_src1 != TheISA::PointerID(0) &&
                      _pid_src2 != TheISA::PointerID(0))
                  {
                      tc->PointerTrackerTable[dest] = TheISA::PointerID(0);
                  }
                  else if (_pid_src1 != TheISA::PointerID(0))
                  {
                      tc->PointerTrackerTable[dest] = _pid_src1;
                  }
                  else if (_pid_src2 != TheISA::PointerID(0))
                  {
                      tc->PointerTrackerTable[dest] = _pid_src2;
                  }
            }
          }
          else if ((si->getName().compare("add") == 0) ||
                   (si->getName().compare("adc") == 0))
          {
              if (si->destRegIdx(0).isIntReg() &&
                  si->srcRegIdx(0).isIntReg()  &&
                  si->srcRegIdx(1).isIntReg())
              {
                  X86ISA::IntRegIndex   src1 = (X86ISA::IntRegIndex)
                                                si->srcRegIdx(0).index();
                  X86ISA::IntRegIndex   src2 = (X86ISA::IntRegIndex)
                                                si->srcRegIdx(1).index();
                  X86ISA::IntRegIndex   dest = (X86ISA::IntRegIndex)
                                                si->destRegIdx(0).index();

                  if (dest < X86ISA::INTREG_RAX ||
                      dest >= X86ISA::NUM_INTREGS + 15)
                        return;
                  if (src1 < X86ISA::INTREG_RAX ||
                      src1 >= X86ISA::NUM_INTREGS + 15)
                        return;
                  if (src2 < X86ISA::INTREG_RAX ||
                      src2 >= X86ISA::NUM_INTREGS + 15)
                        return;

                  TheISA::PointerID _pid_src1 = tc->PointerTrackerTable[src1];
                  TheISA::PointerID _pid_src2 = tc->PointerTrackerTable[src2];


                  if (_pid_src1 != TheISA::PointerID(0) &&
                      _pid_src2 != TheISA::PointerID(0))
                  {
                      tc->PointerTrackerTable[dest] = TheISA::PointerID(0);
                  }
                  else if (_pid_src1 != TheISA::PointerID(0))
                  {
                      tc->PointerTrackerTable[dest] = _pid_src1;
                  }
                  else if (_pid_src2 != TheISA::PointerID(0)){
                      tc->PointerTrackerTable[dest] = _pid_src2;
                  }
              }
          }
          else if ((si->getName().compare("andi") == 0))
          {
              if (si->destRegIdx(0).isIntReg() &&
                  si->srcRegIdx(0).isIntReg())
              {
                  X86ISA::IntRegIndex   src1 =
                          (X86ISA::IntRegIndex)si->srcRegIdx(0).index();
                  X86ISA::IntRegIndex   dest =
                          (X86ISA::IntRegIndex)si->destRegIdx(0).index();

                  if (dest < X86ISA::INTREG_RAX ||
                      dest >= X86ISA::NUM_INTREGS + 15)
                        return;
                  if (src1 < X86ISA::INTREG_RAX ||
                      src1 >= X86ISA::NUM_INTREGS + 15)
                        return;

                  TheISA::PointerID _pid_src1 = tc->PointerTrackerTable[src1];
                  tc->PointerTrackerTable[dest] = _pid_src1;
              }
          }

          else if ((si->getName().compare("addi") == 0) ||
                  (si->getName().compare("adci") == 0) ||
                  (si->getName().compare("subi") == 0) ||
                  (si->getName().compare("sbbi") == 0)
                  )
          {
            if (si->destRegIdx(0).isIntReg() &&
                si->srcRegIdx(0).isIntReg())
            {
                X86ISA::IntRegIndex   src1 =
                        (X86ISA::IntRegIndex)si->srcRegIdx(0).index();
                X86ISA::IntRegIndex   dest =
                        (X86ISA::IntRegIndex)si->destRegIdx(0).index();

                if (dest < X86ISA::INTREG_RAX ||
                    dest >= X86ISA::NUM_INTREGS + 15)
                      return;
                if (src1 < X86ISA::INTREG_RAX ||
                            src1 >= X86ISA::NUM_INTREGS + 15)
                      return;


                TheISA::PointerID _pid_src1 = tc->PointerTrackerTable[src1];
                tc->PointerTrackerTable[dest] = _pid_src1;
            }
          }

          else if ((si->getName().compare("mov") == 0))
          {
              if (si->destRegIdx(0).isIntReg() &&
                  si->srcRegIdx(1).isIntReg())
              {
                X86ISA::IntRegIndex   src1 =
                            (X86ISA::IntRegIndex)si->srcRegIdx(1).index();
                X86ISA::IntRegIndex   dest =
                            (X86ISA::IntRegIndex)si->destRegIdx(0).index();
                if (dest < X86ISA::INTREG_RAX ||
                    dest >= X86ISA::NUM_INTREGS + 15)
                      return;
                if (src1 < X86ISA::INTREG_RAX ||
                    src1 >= X86ISA::NUM_INTREGS + 15)
                      return;

                TheISA::PointerID _pid_src1 = tc->PointerTrackerTable[src1];
                tc->PointerTrackerTable[dest] = _pid_src1;
              }

          }

          else if ((si->getName().compare("ld") == 0) ||
                   (si->getName().compare("ldis") == 0))
          {
              if (si->destRegIdx(0).isIntReg()){
                  X86ISA::IntRegIndex   dest =
                          (X86ISA::IntRegIndex)si->destRegIdx(0).index();
                  if (dest < X86ISA::INTREG_RAX ||
                      dest >= X86ISA::NUM_INTREGS + 15)
                      return;

                tc->PointerTrackerTable[dest] = macroop_pid;

              }
          }


          if (si->isLastMicroop()){
              for (int i = 16;i < TheISA::NumIntRegsToTrack; ++i)
              {
                  tc->PointerTrackerTable[i] = TheISA::PointerID(0);
              }
          }

       }

}


bool MacroopBase::injectCheckMicroops(){

  if ((numMicroops > 0) && (!_isInjected))
  {

      for (int j = 0; j < numMicroops; ++j)
      {
          if ((microops[j]->getName().compare("br") == 0))
          {
              return false;
          }
      }

      int i;
      for (i = 0; i < numMicroops; ++i)
      {
          if ((microops[i]->getName().compare("ld") == 0) &&
              microops[i]->uop_pid != TheISA::PointerID(0))
          {
              return true;
          }
          else if ((microops[i]->getName().compare("st") == 0) &&
              microops[i]->uop_pid != TheISA::PointerID(0))
          {
              return true;
          }
      }

      if (i >= numMicroops) {
        return false;
      }

  }
  return false;

}

void MacroopBase::undoInjecttion(){
    if (numMicroops > 0 &&  _isInjected){

        _isInjected = false;

        panic_if(numOfOriginalMicroops >= numMicroops,
                "numOfOriginalMicroops >= numMicroops");

        StaticInstPtr * microopTemp = new StaticInstPtr[numOfOriginalMicroops];
        int numOfInjectedUops = numMicroops - numOfOriginalMicroops;
        for (int i=0; i < numOfOriginalMicroops; i++)
            microopTemp[i] = microops[i + numOfInjectedUops];

        delete [] microops;
        microops = microopTemp;
        microops[0]->setFirstMicroop();
        numMicroops = numOfOriginalMicroops;

        // for (size_t i = 0; i < numMicroops; i++) {
        //     if (microops[i]->uop_pid != TheISA::PointerID(0) &&
        //         (microops[i]->getName().compare("ld") == 0)){
        //       // assert((microops[i]->getName().compare("ld") == 0) ||
        //       //        (microops[i]->getName().compare("st") == 0));
        //       microops[i]->removeSrcReg();
        //     }
        // }

    }
    else if (numMicroops <= 0){
        //DPRINTF(Capability, "INVALID NUMBER OF MICROOPS\n");
        panic("Invalid  Number Of Microops");
    }

}


void
MacroopBase::injectBoundsCheck(PCState &nextPC){


    if (numMicroops > 0 && !_isInjected)
    {

        // remember to set and clear last micro of original microops
        for (int idx = 0; idx < numMicroops; ++idx)
        {
            if ((microops[idx]->getName().compare("st") == 0) &&
                microops[idx]->uop_pid != TheISA::PointerID(0))
            {
              //microops[idx]->addSrcReg(InstRegIndex(NUM_INTREGS+18));

              microops[0]->clearFirstMicroop();

              StaticInstPtr * microopTemp =
                                  new StaticInstPtr[numMicroops + 1];

              for (int i=0; i < numMicroops; i++)
                  microopTemp[i+1] = microops[i];

              StaticInstPtr micro_0 = (env.dataSize >= 4) ?
                      (StaticInstPtr)(new X86ISAInst::LdBig(machInst,
                        "AP_BOUNDS_CHECK_INJECT",
                        (1ULL << StaticInst::IsMicroop) |
                        (1ULL << StaticInst::IsFirstMicroop) |
                        (1ULL << StaticInst::IsMicroopInjected)|
                        (1ULL << StaticInst::IsBoundsCheckMicroop),
                        env.scale,
                        InstRegIndex(microops[idx]->srcRegIdx(0).index()),
                        InstRegIndex(microops[idx]->srcRegIdx(1).index()),
                        microops[idx]->getDisp(),
                        InstRegIndex(env.seg),
                        InstRegIndex(NUM_INTREGS+18),
                        env.dataSize,
                        env.addressSize,
                        0 | (machInst.legacy.addr ?
                        (AddrSizeFlagBit << FlagShift) : 0))) :
                    (StaticInstPtr)(new X86ISAInst::Ld(machInst,
                        "AP_BOUNDS_CHECK_INJECT",
                        (1ULL << StaticInst::IsMicroop) |
                        (1ULL << StaticInst::IsFirstMicroop) |
                        (1ULL << StaticInst::IsMicroopInjected)|
                        (1ULL << StaticInst::IsBoundsCheckMicroop),
                        env.scale,
                        InstRegIndex(microops[idx]->srcRegIdx(0).index()),
                        InstRegIndex(microops[idx]->srcRegIdx(1).index()),
                        microops[idx]->getDisp(),
                        InstRegIndex(env.seg),
                        InstRegIndex(NUM_INTREGS+18),
                        env.dataSize,
                        env.addressSize,
                        0 | (machInst.legacy.addr ?
                        (AddrSizeFlagBit << FlagShift) : 0))
                        );
              microopTemp[0] = micro_0;

              delete [] microops;
              microops = microopTemp;
              numMicroops = numMicroops + 1;
              _isInjected = true;
              break;
            }

            else if ((microops[idx]->getName().compare("ld") == 0) &&
                microops[idx]->uop_pid != TheISA::PointerID(0))
            {
                //microops[idx]->addSrcReg(InstRegIndex(NUM_INTREGS+18));

                microops[0]->clearFirstMicroop();

                StaticInstPtr * microopTemp =
                                    new StaticInstPtr[numMicroops + 1];

                for (int i=0; i < numMicroops; i++)
                    microopTemp[i+1] = microops[i];

                StaticInstPtr micro_0 = (env.dataSize >= 4) ?
                        (StaticInstPtr)(new X86ISAInst::LdBig(machInst,
                          "AP_BOUNDS_CHECK_INJECT",
                          (1ULL << StaticInst::IsMicroop) |
                          (1ULL << StaticInst::IsFirstMicroop) |
                          (1ULL << StaticInst::IsMicroopInjected)|
                          (1ULL << StaticInst::IsBoundsCheckMicroop),
                          env.scale,
                          InstRegIndex(microops[idx]->srcRegIdx(0).index()),
                          InstRegIndex(microops[idx]->srcRegIdx(1).index()),
                          microops[idx]->getDisp(),
                          InstRegIndex(env.seg),
                          InstRegIndex(NUM_INTREGS+18),
                          env.dataSize,
                          env.addressSize,
                          0 | (machInst.legacy.addr ?
                          (AddrSizeFlagBit << FlagShift) : 0))) :
                      (StaticInstPtr)(new X86ISAInst::Ld(machInst,
                          "AP_BOUNDS_CHECK_INJECT",
                          (1ULL << StaticInst::IsMicroop) |
                          (1ULL << StaticInst::IsFirstMicroop) |
                          (1ULL << StaticInst::IsMicroopInjected)|
                          (1ULL << StaticInst::IsBoundsCheckMicroop),
                          env.scale,
                          InstRegIndex(microops[idx]->srcRegIdx(0).index()),
                          InstRegIndex(microops[idx]->srcRegIdx(1).index()),
                          microops[idx]->getDisp(),
                          InstRegIndex(env.seg),
                          InstRegIndex(NUM_INTREGS+18),
                          env.dataSize,
                          env.addressSize,
                          0 | (machInst.legacy.addr ?
                          (AddrSizeFlagBit << FlagShift) : 0))
                          );
                microopTemp[0] = micro_0;

                delete [] microops;
                microops = microopTemp;
                numMicroops = numMicroops + 1;
                _isInjected = true;
                break;
            }



        }

        if (!_isInjected)
            panic("wrong injection!");

    }

    else if (numMicroops <= 0)
    {
        panic("Invalid  Number Of Microops");
    }

 }

void
MacroopBase::injectMicroops(ThreadContext * _tc,
                            PCState &nextPC, TheISA::CheckType _sym)
{


    if (_sym == TheISA::CheckType::AP_MALLOC_BASE_COLLECT){
        injectAPMallocBaseCollector(_tc, nextPC);
      }
    else if (_sym == TheISA::CheckType::AP_MALLOC_SIZE_COLLECT){
        injectAPMallocSizeCollector(_tc, nextPC);
      }
    else if (_sym == TheISA::CheckType::AP_FREE_CALL){
        injectAPFreeCall(_tc, nextPC);
      }
    else if (_sym == TheISA::CheckType::AP_FREE_RET){
        injectAPFreeRet(_tc, nextPC);
      }
    else if (_sym == TheISA::CheckType::AP_BOUNDS_INJECT){
        injectBoundsCheck(nextPC);
      }
    if (_sym == TheISA::CheckType::AP_CALLOC_BASE_COLLECT){
        injectAPCallocBaseCollector(_tc, nextPC);
      }
    else if (_sym == TheISA::CheckType::AP_CALLOC_SIZE_COLLECT){
        injectAPCallocSizeCollector(_tc, nextPC);
      }
    if (_sym == TheISA::CheckType::AP_REALLOC_BASE_COLLECT){
        injectAPReallocBaseCollector(_tc, nextPC);
      }
    else if (_sym == TheISA::CheckType::AP_REALLOC_SIZE_COLLECT){
        injectAPReallocSizeCollector(_tc, nextPC);
      }

}

void
MacroopBase::injectAPFreeRet(ThreadContext * _tc, PCState &nextPC){

  if (numMicroops > 0 && !_isInjected){

          _isInjected = true;

          // remember to set and clear last micro of original microops
          microops[0]->clearFirstMicroop();


          StaticInstPtr * microopTemp = new StaticInstPtr[numMicroops + 1];

          for (int i=0; i < numMicroops; i++)
              microopTemp[i+1] = microops[i];

          StaticInstPtr micro_0 = new X86ISAInst::Mov(
                          machInst,
                          "AP_FREE_RET_INJECT",
                          (1ULL << StaticInst::IsMicroop) |
                          (1ULL << StaticInst::IsFirstMicroop) |
                          (1ULL << StaticInst::IsMicroopInjected)|
                          (1ULL << StaticInst::IsSerializing)|
                          (1ULL << StaticInst::IsSerializeBefore)|
                          (1ULL << StaticInst::IsFreeRetMicroop),
                          InstRegIndex(X86ISA::NUM_INTREGS+17),
                          InstRegIndex(X86ISA::INTREG_RDI),
                          InstRegIndex(X86ISA::NUM_INTREGS+17),
                          8,
                          0);


          microopTemp[0]   = micro_0;

          delete [] microops;
          microops = microopTemp;
          numMicroops = numMicroops + 1;




      }
      else if (numMicroops <= 0){
          panic("Invalid  Number Of Microops");

      }


}

void
MacroopBase::injectAPFreeCall(ThreadContext * _tc, PCState &nextPC){

if (numMicroops > 0 && !_isInjected){

        _isInjected = true;

        // remember to set and clear last micro of original microops
        microops[0]->clearFirstMicroop();


        StaticInstPtr * microopTemp = new StaticInstPtr[numMicroops + 1];

        for (int i=0; i < numMicroops; i++)
            microopTemp[i+1] = microops[i];

        StaticInstPtr micro_0 = new X86ISAInst::Mov(
                        machInst,
                        "AP_FREE_CALL_INJECT",
                        (1ULL << StaticInst::IsMicroop) |
                        (1ULL << StaticInst::IsFirstMicroop) |
                        (1ULL << StaticInst::IsMicroopInjected)|
                        (1ULL << StaticInst::IsSerializing)|
                        (1ULL << StaticInst::IsSerializeBefore)|
                        (1ULL << StaticInst::IsFreeCallMicroop),
                        InstRegIndex(X86ISA::NUM_INTREGS+17),
                        InstRegIndex(X86ISA::INTREG_RDI),
                        InstRegIndex(X86ISA::NUM_INTREGS+17),
                        8,
                        0);


        microopTemp[0]   = micro_0;

        delete [] microops;
        microops = microopTemp;
        numMicroops = numMicroops + 1;




    }
    else if (numMicroops <= 0){
        panic("Invalid  Number Of Microops");

    }

}

void
MacroopBase::injectAPMallocSizeCollector(ThreadContext * _tc, PCState &nextPC){


    if (numMicroops > 0 && !_isInjected){

        _isInjected = true;

        // remember to set and clear last micro of original microops
        microops[0]->clearFirstMicroop();


        StaticInstPtr * microopTemp = new StaticInstPtr[numMicroops + 2];

        for (int i=0; i < numMicroops; i++)
            microopTemp[i+2] = microops[i];

        StaticInstPtr micro_1 = new X86ISAInst::AddImmBig(
                        machInst,
                        "AP_MALLOC_SIZE_COLLECT_INJECT",
                        (1ULL << StaticInst::IsMicroop) |
                        (1ULL << StaticInst::IsMicroopInjected)|
                        (1ULL << StaticInst::IsSerializing)|
                        (1ULL << StaticInst::IsSerializeBefore),
                        InstRegIndex(X86ISA::INTREG_R16),
                        1,
                        InstRegIndex(X86ISA::INTREG_R16),
                        8,
                        0);

        StaticInstPtr micro_0 = new X86ISAInst::Mov(
                        machInst,
                        "AP_MALLOC_SIZE_COLLECT_INJECT",
                        (1ULL << StaticInst::IsMicroop) |
                        (1ULL << StaticInst::IsMallocSizeCollectorMicroop)|
                        (1ULL << StaticInst::IsMicroopInjected)|
                        (1ULL << StaticInst::IsFirstMicroop)|
                        (1ULL << StaticInst::IsSerializing)|
                        (1ULL << StaticInst::IsSerializeBefore),
                        InstRegIndex(X86ISA::NUM_INTREGS+17),
                        InstRegIndex(X86ISA::INTREG_RDI),
                        InstRegIndex(X86ISA::NUM_INTREGS+17),
                        8,
                        0);



        microopTemp[0]   = micro_0;
        microopTemp[1]   = micro_1;

        delete [] microops;
        microops = microopTemp;
        numMicroops = numMicroops + 2;

    }
    else if (numMicroops <= 0){
        panic("Invalid  Number Of Microops");

    }

}


void
MacroopBase::injectAPMallocBaseCollector(ThreadContext * _tc, PCState &nextPC){

    if (numMicroops > 0 && !_isInjected){

        _isInjected = true;

        // remember to set and clear last micro of original microops
        microops[0]->clearFirstMicroop();


        StaticInstPtr * microopTemp = new StaticInstPtr[numMicroops + 2];

        for (int i=0; i < numMicroops; i++)
            microopTemp[i+2] = microops[i];

        StaticInstPtr micro_0 = new X86ISAInst::Mov(
                              machInst,
                              "AP_MALLOC_BASE_COLLECT_INJECT",
                              (1ULL << StaticInst::IsMicroop) |
                              (1ULL << StaticInst::IsFirstMicroop) |
                              (1ULL << StaticInst::IsMicroopInjected)|
                              (1ULL << StaticInst::IsSerializing)|
                              (1ULL << StaticInst::IsSerializeBefore),
                              InstRegIndex(X86ISA::INTREG_R16),
                              InstRegIndex(X86ISA::INTREG_R16),
                              InstRegIndex(X86ISA::INTREG_R16),
                              8,
                              0);

        StaticInstPtr micro_1 = new X86ISAInst::Mov(
                        machInst,
                        "AP_MALLOC_BASE_COLLECT_INJECT",
                        (1ULL << StaticInst::IsMicroop) |
                        (1ULL << StaticInst::IsMicroopInjected)|
                        (1ULL << StaticInst::IsMallocBaseCollectorMicroop)|
                        (1ULL << StaticInst::IsSerializing)|
                        (1ULL << StaticInst::IsSerializeBefore),
                        InstRegIndex(X86ISA::NUM_INTREGS+17),
                        InstRegIndex(X86ISA::INTREG_RAX),
                        InstRegIndex(X86ISA::NUM_INTREGS+17),
                        8,
                        0);

        microopTemp[0] = micro_0;
        microopTemp[1] = micro_1;

        delete [] microops;
        microops = microopTemp;
        numMicroops = numMicroops + 2;

    }
    else if (numMicroops <= 0){
        panic("Invalid  Number Of Microops");
        //DPRINTF(Capability, "INVALID NUMBER OF MICROOPS\n");

    }
}




 void
 MacroopBase::injectAPCallocSizeCollector(ThreadContext * _tc, PCState &nextPC)
 {


     if (numMicroops > 0 && !_isInjected){

         _isInjected = true;

         // remember to set and clear last micro of original microops
         microops[0]->clearFirstMicroop();


         StaticInstPtr * microopTemp = new StaticInstPtr[numMicroops + 1];

         for (int i=0; i < numMicroops; i++)
             microopTemp[i+1] = microops[i];

         StaticInstPtr micro_0 = new X86ISAInst::Mov(
                         machInst,
                         "AP_MAALOC_SIZE_COLLECT_INJECT",
                         (1ULL << StaticInst::IsMicroop) |
                         (1ULL << StaticInst::IsFirstMicroop) |
                         (1ULL << StaticInst::IsMicroopInjected)|
                         (1ULL << StaticInst::IsCallocSizeCollectorMicroop),
                         InstRegIndex(X86ISA::NUM_INTREGS+17),
                         InstRegIndex(X86ISA::INTREG_RDI),
                         InstRegIndex(X86ISA::NUM_INTREGS+17),
                         /*env.dataSize*/8,
                         0);


         microopTemp[0]   = micro_0;

         delete [] microops;
         microops = microopTemp;
         numMicroops = numMicroops + 1;




     }
     else if (numMicroops <= 0){
         panic("Invalid  Number Of Microops");

     }

 }


 void
 MacroopBase::injectAPCallocBaseCollector(ThreadContext * _tc, PCState &nextPC)
 {

     if (numMicroops > 0 && !_isInjected){

         _isInjected = true;

         // remember to set and clear last micro of original microops
         microops[0]->clearFirstMicroop();


         StaticInstPtr * microopTemp = new StaticInstPtr[numMicroops + 1];

         for (int i=0; i < numMicroops; i++)
             microopTemp[i+1] = microops[i];

         StaticInstPtr micro_0 = new X86ISAInst::Mov(
                         machInst,
                         "AP_MALLOC_BASE_COLLECT_INJECT",
                         (1ULL << StaticInst::IsMicroop) |
                         (1ULL << StaticInst::IsFirstMicroop) |
                         (1ULL << StaticInst::IsMicroopInjected)|
                         (1ULL << StaticInst::IsCallocBaseCollectorMicroop),
                         InstRegIndex(X86ISA::NUM_INTREGS+17),
                         InstRegIndex(X86ISA::INTREG_RAX),
                         InstRegIndex(X86ISA::NUM_INTREGS+17),
                         /*env.dataSize*/8,
                         0);


         microopTemp[0]   = micro_0;

         delete [] microops;
         microops = microopTemp;
         numMicroops = numMicroops + 1;



     }
     else if (numMicroops <= 0){
         panic("Invalid  Number Of Microops");
         //DPRINTF(Capability, "INVALID NUMBER OF MICROOPS\n");

     }
 }


 void
 MacroopBase::injectAPReallocSizeCollector(ThreadContext * _tc,
                                          PCState &nextPC)
{


     if (numMicroops > 0 && !_isInjected){

         _isInjected = true;

         // remember to set and clear last micro of original microops
         microops[0]->clearFirstMicroop();


         StaticInstPtr * microopTemp = new StaticInstPtr[numMicroops + 1];

         for (int i=0; i < numMicroops; i++)
             microopTemp[i+1] = microops[i];

         StaticInstPtr micro_0 = new X86ISAInst::Mov(
                         machInst,
                         "AP_MAALOC_SIZE_COLLECT_INJECT",
                         (1ULL << StaticInst::IsMicroop) |
                         (1ULL << StaticInst::IsFirstMicroop) |
                         (1ULL << StaticInst::IsMicroopInjected)|
                         (1ULL << StaticInst::IsReallocSizeCollectorMicroop),
                         InstRegIndex(X86ISA::NUM_INTREGS+17),
                         InstRegIndex(X86ISA::INTREG_RDI),
                         InstRegIndex(X86ISA::NUM_INTREGS+17),
                         /*env.dataSize*/8,
                         0);


         microopTemp[0]   = micro_0;

         delete [] microops;
         microops = microopTemp;
         numMicroops = numMicroops + 1;




     }
     else if (numMicroops <= 0){
         panic("Invalid  Number Of Microops");

     }

 }


 void
 MacroopBase::injectAPReallocBaseCollector(ThreadContext * _tc,
                                          PCState &nextPC)
{

     if (numMicroops > 0 && !_isInjected){

         _isInjected = true;

         // remember to set and clear last micro of original microops
         microops[0]->clearFirstMicroop();


         StaticInstPtr * microopTemp = new StaticInstPtr[numMicroops + 1];

         for (int i=0; i < numMicroops; i++)
             microopTemp[i+1] = microops[i];

         StaticInstPtr micro_0 = new X86ISAInst::Mov(
                         machInst,
                         "AP_MALLOC_BASE_COLLECT_INJECT",
                         (1ULL << StaticInst::IsMicroop) |
                         (1ULL << StaticInst::IsFirstMicroop) |
                         (1ULL << StaticInst::IsMicroopInjected)|
                         (1ULL << StaticInst::IsReallocBaseCollectorMicroop),
                         InstRegIndex(X86ISA::NUM_INTREGS+17),
                         InstRegIndex(X86ISA::INTREG_RAX),
                         InstRegIndex(X86ISA::NUM_INTREGS+17),
                         /*env.dataSize*/8,
                         0);


         microopTemp[0]   = micro_0;

         delete [] microops;
         microops = microopTemp;
         numMicroops = numMicroops + 1;



     }
     else if (numMicroops <= 0){
         panic("Invalid  Number Of Microops");
         //DPRINTF(Capability, "INVALID NUMBER OF MICROOPS\n");

     }
 }


}
