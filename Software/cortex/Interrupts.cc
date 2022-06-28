//
// Created by jwscoggins on 8/28/21.
//

#include "Interrupts.h"

namespace {
    InterruptFunction nmiFunction_ = 0;
    InterruptFunction isr0Function_ = 0;
}
extern "C" InterruptFunction getNMIFunction() { return nmiFunction_; }
extern "C" InterruptFunction getISR0Function() { return isr0Function_; }
extern "C" void setNMIFunction(InterruptFunction func) {nmiFunction_ = func; }
extern "C" void setISR0Function(InterruptFunction func) { isr0Function_ = func; }

