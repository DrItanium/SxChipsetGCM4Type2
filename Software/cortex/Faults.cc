//
// Created by jwscoggins on 8/28/21.
//

#include "Faults.h"
#include "chipset/ChipsetInteract.h"
#include <stdio.h>
namespace cortex
{
    void
    FaultData::display() {
        printf("Fault Type: %x\n", ftype);
        switch (ftype) {
            case 1: printf("\tTrace Fault\n"); break;
            case 2: printf("\tOperation Fault\n"); break;
            case 3: printf("\tArithmetic Fault\n"); break;
            case 4: printf("\tFloating-Point Fault\n"); break;
            case 5: printf("\tConstraint Fault\n"); break;
            case 7: printf("\tProtection Fault\n"); break;
            case 0xa: printf("\tType Fault\n"); break;
            default:
                break;
        }
        printf("Fault Subtype: %x\n", fsubtype);
        if (ftype == 2) {
            // load and display the operation in question
            if (fsubtype == 1) {
                printf("\tInvalid Opcode\n");
            } else if (fsubtype == 4) {
                printf("\tInvalid Operand\n");
            }
            volatile unsigned int* ptr = faddress;
            unsigned int container[8] = { 0 };
            for (int i = 0;i < 8; ++i, ++ptr) {
                container[i] = *ptr;
            }
            printf("\t\tContents: \n");
            for (int i = 0; i < 8; ++i) {
               printf("\t\t\t0x%x\n", container[i]);
            }
        }
        printf("Faulting Address: %p\n", faddress);
        printf("PC: %x\n", pc);
        printf("AC: %x\n", ac);
    }
    namespace {
        FaultHandler userReserved_ = 0;
        FaultHandler userTrace_ = 0;
        FaultHandler userOperation_ = 0;
        FaultHandler userArithmetic_ = 0;
        FaultHandler userRealArithmetic_ = 0;
        FaultHandler userConstraint_ = 0;
        FaultHandler userProtection_ = 0;
        FaultHandler userMachine_ = 0;
        FaultHandler userType_ = 0;
    }
    FaultHandler getUserReservedFaultHandler() { return userReserved_; }
    FaultHandler getUserTraceFaultHandler() { return userTrace_; }
    FaultHandler getUserOperationFaultHandler() { return userOperation_; }
    FaultHandler getUserArithmeticFaultHandler() { return userArithmetic_; }
    FaultHandler getUserRealArithmeticFaultHandler() { return userRealArithmetic_; }
    FaultHandler getUserConstraintFaultHandler() { return userConstraint_; }
    FaultHandler getUserProtectionFaultHandler() { return userProtection_; }
    FaultHandler getUserMachineFaultHandler() { return userMachine_; }
    FaultHandler getUserTypeFaultHandler() { return userType_; }
    void setUserReservedFaultHandler(FaultHandler handler) { userReserved_ = handler; }
    void setUserTraceFaultHandler(FaultHandler handler) { userTrace_ = handler; }
    void setUserOperationFaultHandler(FaultHandler handler) { userOperation_ = handler; }
    void setUserArithmeticFaultHandler(FaultHandler handler) { userArithmetic_ = handler; }
    void setUserRealArithmeticFaultHandler(FaultHandler handler) { userRealArithmetic_ = handler; }
    void setUserConstraintFaultHandler(FaultHandler handler) { userConstraint_ = handler; }
    void setUserProtectionFaultHandler(FaultHandler handler) { userProtection_ = handler; }
    void setUserMachineFaultHandler(FaultHandler handler) { userMachine_ = handler; }
    void setUserTypeFaultHandler(FaultHandler handler) { userType_ = handler; }
}
