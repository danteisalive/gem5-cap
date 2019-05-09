/*
 * macroop.cc
 *
 * Created on: Feb 27, 2017
 * Author: mkt
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

void MacroopBase::updatePointerTracker(ThreadContext * tc,
                                        TheISA::PointerID& _pred_pid)
  {
      for (size_t i = 0; i < numMicroops; i++) {

           const StaticInstPtr si = microops[i];
           #define ENABLE_CAPABILITY_DEBUG 0

           // sanitization
           //if (tc->DisablePointerTracker) return;
           if (si->isMicroopInjected()) return;
           if (tc->stopTracking) return;


          if ((si->getName().compare("and") == 0)){

            X86ISA::IntRegIndex   src1 = (X86ISA::IntRegIndex)
                                          si->srcRegIdx(0).index();
            X86ISA::IntRegIndex   src2 = (X86ISA::IntRegIndex)
                                          si->srcRegIdx(1).index();
            X86ISA::IntRegIndex   dest = (X86ISA::IntRegIndex)
                                          si->destRegIdx(0).index();
            TheISA::PointerID _pid_src1 = tc->RegTrackTable[src1];
            TheISA::PointerID _pid_src2 = tc->RegTrackTable[src2];
            TheISA::PointerID _pid_dest = tc->RegTrackTable[dest];


            if (src1 == src2)
            {
              tc->RegTrackTable[dest] = _pid_src1;
            }
            else
            {
              if ( _pid_src1 != TheISA::PointerID(0) &&
                   _pid_src2 == TheISA::PointerID(0))
              {
                   tc->RegTrackTable[dest] = _pid_src1;
              }
              else if ( _pid_src1 == TheISA::PointerID(0) &&
                        _pid_src2 != TheISA::PointerID(0))
              {
                   tc->RegTrackTable[dest] = _pid_src2;
              }
              else
              {
                   tc->RegTrackTable[dest] = TheISA::PointerID(0);
              }
            }

          }
          else if ((si->getName().compare("xor") == 0)){

            X86ISA::IntRegIndex   src1 = (X86ISA::IntRegIndex)
                                          si->srcRegIdx(0).index();
            X86ISA::IntRegIndex   src2 = (X86ISA::IntRegIndex)
                                          si->srcRegIdx(1).index();
            X86ISA::IntRegIndex   dest = (X86ISA::IntRegIndex)
                                          si->destRegIdx(0).index();
            TheISA::PointerID _pid_src1 = tc->RegTrackTable[src1];
            TheISA::PointerID _pid_src2 = tc->RegTrackTable[src2];
            TheISA::PointerID _pid_dest = tc->RegTrackTable[dest];

              tc->RegTrackTable[dest] = TheISA::PointerID(0);

          }
          else if ((si->getName().compare("sub") == 0) ||
                   (si->getName().compare("sbb") == 0))
          {

            X86ISA::IntRegIndex   src1 = (X86ISA::IntRegIndex)
                                          si->srcRegIdx(0).index();
            X86ISA::IntRegIndex   src2 = (X86ISA::IntRegIndex)
                                          si->srcRegIdx(1).index();
            X86ISA::IntRegIndex   dest = (X86ISA::IntRegIndex)
                                          si->destRegIdx(0).index();
            TheISA::PointerID _pid_src1 = tc->RegTrackTable[src1];
            TheISA::PointerID _pid_src2 = tc->RegTrackTable[src2];
            TheISA::PointerID _pid_dest = tc->RegTrackTable[dest];

            if (_pid_src1 != TheISA::PointerID(0) &&
                _pid_src2 != TheISA::PointerID(0))
            {
                tc->RegTrackTable[dest] = TheISA::PointerID(0);
            }
            else if (_pid_src1 != TheISA::PointerID(0))
            {
                tc->RegTrackTable[dest] = _pid_src1;
            }
            else if (_pid_src2 != TheISA::PointerID(0))
            {
                tc->RegTrackTable[dest] = _pid_src2;
            }
          }
          else if ((si->getName().compare("add") == 0) ||
                   (si->getName().compare("adc") == 0))
          {

            X86ISA::IntRegIndex   src1 = (X86ISA::IntRegIndex)
                                          si->srcRegIdx(0).index();
            X86ISA::IntRegIndex   src2 = (X86ISA::IntRegIndex)
                                          si->srcRegIdx(1).index();
            X86ISA::IntRegIndex   dest = (X86ISA::IntRegIndex)
                                          si->destRegIdx(0).index();
            TheISA::PointerID _pid_src1 = tc->RegTrackTable[src1];
            TheISA::PointerID _pid_src2 = tc->RegTrackTable[src2];
            TheISA::PointerID _pid_dest = tc->RegTrackTable[dest];

            if (_pid_src1 != TheISA::PointerID(0) &&
                _pid_src2 != TheISA::PointerID(0))
            {
                tc->RegTrackTable[dest] = TheISA::PointerID(0);
            }
            else if (_pid_src1 != TheISA::PointerID(0))
            {
                tc->RegTrackTable[dest] = _pid_src1;
            }
            else if (_pid_src2 != TheISA::PointerID(0)){
                tc->RegTrackTable[dest] = _pid_src2;
            }

          }
          else if ((si->getName().compare("andi") == 0))
          {
              X86ISA::IntRegIndex   src1 =
                      (X86ISA::IntRegIndex)si->srcRegIdx(0).index();
              X86ISA::IntRegIndex   dest =
                      (X86ISA::IntRegIndex)si->destRegIdx(0).index();
              TheISA::PointerID _pid_src1 = tc->RegTrackTable[src1];
              TheISA::PointerID _pid_dest = tc->RegTrackTable[dest];

              tc->RegTrackTable[dest] = _pid_src1;

          }

          else if ((si->getName().compare("addi") == 0) ||
                  (si->getName().compare("adci") == 0) ||
                  (si->getName().compare("subi") == 0) ||
                  (si->getName().compare("sbbi") == 0)
                  )
          {
              X86ISA::IntRegIndex   src1 =
                      (X86ISA::IntRegIndex)si->srcRegIdx(0).index();
              X86ISA::IntRegIndex   dest =
                      (X86ISA::IntRegIndex)si->destRegIdx(0).index();
              TheISA::PointerID _pid_src1 = tc->RegTrackTable[src1];
              TheISA::PointerID _pid_dest = tc->RegTrackTable[dest];


              tc->RegTrackTable[dest] = _pid_src1;
          }

          else if ((si->getName().compare("mov") == 0)){

              X86ISA::IntRegIndex   src1 =
                              (X86ISA::IntRegIndex)si->srcRegIdx(1).index();
              X86ISA::IntRegIndex   dest =
                              (X86ISA::IntRegIndex)si->destRegIdx(0).index();
              TheISA::PointerID _pid_src1 = tc->RegTrackTable[src1];
              TheISA::PointerID _pid_dest = tc->RegTrackTable[dest];

              tc->RegTrackTable[dest] = _pid_src1;

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

              tc->RegTrackTable[dest] = _pred_pid;

            }


          }


          if (si->isLastMicroop()){
              for (int i = X86ISA::NUM_INTREGS;
                    i < X86ISA::NUM_INTREGS + 16;
                    ++i)
              {
                  tc->RegTrackTable[(X86ISA::IntRegIndex)i] =
                                                  TheISA::PointerID(0);
              }
          }

       }

  }


void MacroopBase::injectCheckMicroops(TheISA::PointerID& _pid){

  if ((numMicroops > 0) && (!_isInjected) && (_pid != TheISA::PointerID(0))){

      int i;
      for (i = 0; i < numMicroops; ++i)
      {
          if (microops[i]->isStore() || microops[i]->isLoad()){
              break;
          }
      }

      if (i >= numMicroops) {
        // there is no load/store microop in this macropp
        //TODO: what should we do in this case?
        // this should not happen as we already know from
        // pointer tracker logic which macro is going to load/store
        // this case is a defenetly a missprediction which will be handled in
        // execute stage
        return;
      }

       // at this point we are sure that this macro has a load/store macropp
      // remember to set and clear last micro of original microops

      microops[0]->clearFirstMicroop();

      StaticInstPtr * microopTemp = new StaticInstPtr[numMicroops + 1];

      for (int i=0; i < numMicroops; i++)
          microopTemp[i+1] = microops[i];
      StaticInstPtr micro_0;
      // TODO: do we need new flag or we can use the old one?
      if (microops[i]->isStore()){
          micro_0 = new X86ISAInst::St(machInst,
                              "AP_BOUNDS_CHECK_INJECT",
                              (1ULL << StaticInst::IsMicroop) |
                               (1ULL << StaticInst::IsFirstMicroop) |
                               (1ULL << StaticInst::IsMicroopInjected)|
                               (1ULL << StaticInst::IsBoundsCheckMicroop),
                              env.scale,
                              InstRegIndex(env.index),
                              InstRegIndex(env.base),
                              microops[i]->getDisp(),
                              InstRegIndex(env.seg),
                              InstRegIndex(NUM_INTREGS+15),
                              2,
                              env.addressSize,
                              0 |
                              (machInst.legacy.addr ?
                                (AddrSizeFlagBit << FlagShift) : 0)
                              );
      }
      else if (microops[i]->isLoad()){
          micro_0 = (env.dataSize >= 4) ?
                  (StaticInstPtr)(new X86ISAInst::LdBig(machInst,
                                  "AP_BOUNDS_CHECK_INJECT",
                                  (1ULL << StaticInst::IsMicroop) |
                                  (1ULL << StaticInst::IsFirstMicroop) |
                                  (1ULL << StaticInst::IsMicroopInjected)|
                                  (1ULL << StaticInst::IsBoundsCheckMicroop),
                                  env.scale,
                                  InstRegIndex(env.index),
                                  InstRegIndex(env.base),
                                   microops[i]->getDisp(),
                                  InstRegIndex(env.seg),
                                  InstRegIndex(env.reg),
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
                                  InstRegIndex(env.index),
                                  InstRegIndex(env.base),
                                   microops[i]->getDisp(),
                                  InstRegIndex(env.seg),
                                  InstRegIndex(env.reg),
                                  env.dataSize,
                                  env.addressSize,
                                  0 | (machInst.legacy.addr ?
                                    (AddrSizeFlagBit << FlagShift) : 0))
                                  );
      }
      else{
          panic("Not a Store/Load Microop!");
      }

      microopTemp[0] = micro_0;

      delete [] microops;
      microops = microopTemp;
      numMicroops = numMicroops + 1;
      _isInjected = true;


  }
  else if (numMicroops <= 0){
      //DPRINTF(Capability, "INVALID NUMBER OF MICROOPS\n");
      panic("Invalid  Number Of Microops");
  }
}

void MacroopBase::undoCheckMicroops(){

  if (numMicroops > 0 &&  _isInjected){

      _isInjected = false;


      StaticInstPtr * microopTemp = new StaticInstPtr[numOfOriginalMicroops];

      for (int i=0; i < numOfOriginalMicroops; i++)
          microopTemp[i] = microops[i+1];

      delete [] microops;
      microops = microopTemp;
      microops[0]->setFirstMicroop();
      numMicroops = numOfOriginalMicroops;
  }
  else if (numMicroops <= 0){
      //DPRINTF(Capability, "INVALID NUMBER OF MICROOPS\n");
      panic("Invalid  Number Of Microops");
  }


}



void MacroopBase::undoInjecttion(){
    if (numMicroops > 0 &&  _isInjected){

        _isInjected = false;


        StaticInstPtr * microopTemp = new StaticInstPtr[numOfOriginalMicroops];

        for (int i=0; i < numOfOriginalMicroops; i++)
            microopTemp[i] = microops[i+1];

        delete [] microops;
        microops = microopTemp;
        microops[0]->setFirstMicroop();
        numMicroops = numOfOriginalMicroops;
        //DPRINTF(Capability, "NUMBER Of MICROOPS AFTER EJECT: %d\n",numMicroops);
    }
    else if (numMicroops <= 0){
        //DPRINTF(Capability, "INVALID NUMBER OF MICROOPS\n");
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
                          (1ULL << StaticInst::IsFreeRetMicroop),
                          InstRegIndex(X86ISA::INTREG_RDI),
                          InstRegIndex(X86ISA::INTREG_RDI),
                          InstRegIndex(X86ISA::NUM_INTREGS+15),
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
                        (1ULL << StaticInst::IsFreeCallMicroop),
                        InstRegIndex(X86ISA::INTREG_RDI),
                        InstRegIndex(X86ISA::INTREG_RDI),
                        InstRegIndex(X86ISA::NUM_INTREGS+15),
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
MacroopBase::injectAPMallocSizeCollector(ThreadContext * _tc, PCState &nextPC){


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
                        (1ULL << StaticInst::IsMallocSizeCollectorMicroop),
                        InstRegIndex(X86ISA::INTREG_RDI),
                        InstRegIndex(X86ISA::INTREG_RDI),
                        InstRegIndex(X86ISA::NUM_INTREGS+15),
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
MacroopBase::injectAPMallocBaseCollector(ThreadContext * _tc, PCState &nextPC){

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
                        (1ULL << StaticInst::IsMallocBaseCollectorMicroop),
                        InstRegIndex(X86ISA::INTREG_RAX),
                        InstRegIndex(X86ISA::INTREG_RAX),
                        InstRegIndex(X86ISA::NUM_INTREGS+15),
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
MacroopBase::injectBoundsCheck(PCState &nextPC){


    if (numMicroops > 0 && !_isInjected){

        _isInjected = true;

        // remember to set and clear last micro of original microops

        microops[0]->clearFirstMicroop();

        StaticInstPtr * microopTemp = new StaticInstPtr[numMicroops + 1];

        for (int i=0; i < numMicroops; i++)
            microopTemp[i+1] = microops[i];



        int i;
        for (i = 0; i < numMicroops; ++i)
        {
            if (microops[i]->isMemRef()){
                break;
            }
        }

        panic_if(i >= numMicroops, "Injecting to a non memory instruction!");

        StaticInstPtr micro_0;

        if (microops[i]->isStore()){
            micro_0 = new X86ISAInst::St(machInst,
                                "AP_BOUNDS_CHECK_INJECT",
                                (1ULL << StaticInst::IsMicroop) |
                                 (1ULL << StaticInst::IsFirstMicroop) |
                                 (1ULL << StaticInst::IsMicroopInjected)|
                                 (1ULL << StaticInst::IsBoundsCheckMicroop),
                                env.scale,
                                InstRegIndex(env.index),
                                InstRegIndex(env.base),
                                microops[i]->getDisp(),
                                InstRegIndex(env.seg),
                                InstRegIndex(NUM_INTREGS+15),
                                2,
                                env.addressSize,
                                0 |
                                (machInst.legacy.addr ?
                                  (AddrSizeFlagBit << FlagShift) : 0)
                                );
        }
        else if (microops[i]->isLoad()){
            micro_0 = (env.dataSize >= 4) ?
                    (StaticInstPtr)(new X86ISAInst::LdBig(machInst,
                                    "AP_BOUNDS_CHECK_INJECT",
                                    (1ULL << StaticInst::IsMicroop) |
                                    (1ULL << StaticInst::IsFirstMicroop) |
                                    (1ULL << StaticInst::IsMicroopInjected)|
                                    (1ULL << StaticInst::IsBoundsCheckMicroop),
                                    env.scale,
                                    InstRegIndex(env.index),
                                    InstRegIndex(env.base),
                                     microops[i]->getDisp(),
                                    InstRegIndex(env.seg),
                                    InstRegIndex(env.reg),
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
                                    InstRegIndex(env.index),
                                    InstRegIndex(env.base),
                                     microops[i]->getDisp(),
                                    InstRegIndex(env.seg),
                                    InstRegIndex(env.reg),
                                    env.dataSize,
                                    env.addressSize,
                                    0 | (machInst.legacy.addr ?
                                      (AddrSizeFlagBit << FlagShift) : 0))
                                    );
        }

        else{
            panic("Not a Store/Load Microop!");
        }

        microopTemp[0] = micro_0;

        delete [] microops;
        microops = microopTemp;
        numMicroops = numMicroops + 1;



    }
    else if (numMicroops <= 0){
        //DPRINTF(Capability, "INVALID NUMBER OF MICROOPS\n");
        panic("Invalid  Number Of Microops");
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
                         InstRegIndex(X86ISA::INTREG_RDI),
                         InstRegIndex(X86ISA::INTREG_RDI),
                         InstRegIndex(X86ISA::NUM_INTREGS+15),
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
                         InstRegIndex(X86ISA::INTREG_RAX),
                         InstRegIndex(X86ISA::INTREG_RAX),
                         InstRegIndex(X86ISA::NUM_INTREGS+15),
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
                         InstRegIndex(X86ISA::INTREG_RDI),
                         InstRegIndex(X86ISA::INTREG_RDI),
                         InstRegIndex(X86ISA::NUM_INTREGS+15),
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
                         InstRegIndex(X86ISA::INTREG_RAX),
                         InstRegIndex(X86ISA::INTREG_RAX),
                         InstRegIndex(X86ISA::NUM_INTREGS+15),
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
