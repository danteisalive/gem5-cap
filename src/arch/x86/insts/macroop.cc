/*
 * macroop.cc
 *
 * Created on: Feb 27, 2017
 * Author: mkt
 */



namespace X86ISA
{



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
