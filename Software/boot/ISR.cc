//
// Created by jwscoggins on 3/27/21.
//
#include "chipset/IODevice.h"
#include "cortex/Interrupts.h"
extern "C" void ISR0(void);
extern "C" void ISR_NMI(void);

extern "C"
void
ISR0(void) {
    InterruptFunction fn = getISR0Function();
    if (fn) {
        fn();
    }
}

extern "C"
void
ISR_NMI(void) {
    InterruptFunction fn = getNMIFunction();
    if (fn) {
        fn();
    }
}
