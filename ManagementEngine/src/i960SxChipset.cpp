/*
i960SxManagementEngine
Copyright (c) 2020-2022, Joshua Scoggins
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/// i960Sx management engine, based on atmega4809 with fuses set for:
/// - C++17
/// Board Platform: MegaCoreX
#include <Arduino.h>
#include <Event.h>
#include <Logic.h>
#include "Pinout.h"
#include "Configuration.h"

enum class i960Pinout : int {
    // PORT A
    FAIL = PIN_PA0,
    CHIPSET_BOOTED_ = PIN_PA1,
    READY_CHIPSET_ = PIN_PA2,
    CLK = PIN_PA3,
    ME_PA4 = PIN_PA4,
    ME_PA5 = PIN_PA5,
    ME_PA6 = PIN_PA6,
    CLK2 = PIN_PA7,
    // PORT C
    BurstLast_ = PIN_PC0,
    InTransaction_ = PIN_PC1,
    DoCycle_ = PIN_PC2,
    BootSuccessful_ = PIN_PC3,
    UPDI_TXD = PIN_PC4,
    UPDI_RXD = PIN_PC5,
    // PORT D
    CHIPSET_INT_ = PIN_PD0,
    ME_PD1 = PIN_PD1,
    ME_PD2 = PIN_PD2,
    INT0_OUT = PIN_PD3,
    ME_PD4 = PIN_PD4,
    ME_PD5 = PIN_PD5,
    ME_PD6 = PIN_PD6,
    ME_PD7 = PIN_PD7,
    // PORT E
    ME_PE0 = PIN_PE0,
    ME_PE1 = PIN_PE1,
    ME_PE2 = PIN_PE2,
    ME_PE3 = PIN_PE3,
    // PORTF
    ME_Ready = PIN_PF0,
    ME_PF1 = PIN_PF1,
    RESET960_ = PIN_PF2,
    READY960 = PIN_PF3, // connected to external inverter, CCL connector
    BLAST_ = PIN_PF4, 
    DEN960 = PIN_PF5,
};




using Den960 = InputPin<i960Pinout::DEN960, LOW, HIGH>;
using BlastPin = InputPin<i960Pinout::BLAST_, LOW, HIGH>;
using FailPin = InputPin<i960Pinout::FAIL, HIGH, LOW>;
using BootSuccessfulPin = ActiveHighOutputPin<i960Pinout::BootSuccessful_>;
using ChipsetBootedPin = ActiveLowInputPin<i960Pinout::CHIPSET_BOOTED_>;
using DoCyclePin = ActiveLowOutputPin<i960Pinout::DoCycle_>;
using ReadySyncPin = ActiveLowOutputPin<i960Pinout::ME_Ready>;
using ReadyChipsetPin = ActiveLowInputPin<i960Pinout::READY_CHIPSET_>;
using InTransactionPin = ActiveLowOutputPin<i960Pinout::InTransaction_>;
using BurstNext = ActiveHighOutputPin<i960Pinout::BurstLast_>;
// Type2 connects this pin to an inverter externally so we can attach a pullup to this output pin
// It allows the i960 to be forced into reset until the management engine is actively controlling it.
// It prevents all sorts of shenanigans
using ResetPin = ActiveHighOutputPin<i960Pinout::RESET960_>;

void
setupPins() noexcept {
    configurePins<FailPin,
            BlastPin ,
            Den960 ,
            ChipsetBootedPin,
            DoCyclePin,
            ReadySyncPin,
            ReadyChipsetPin,
            InTransactionPin,
            BurstNext
    >();
    // the lock pin is special as it is an open collector pin, we want to stay off of it as much as possible
    DoCyclePin ::deassertPin();
    ReadySyncPin :: deassertPin();
    InTransactionPin :: deassertPin();
    BurstNext :: deassertPin();
    // make all outputs deasserted
}

[[noreturn]]
void
terminateExecution() noexcept {
    BootSuccessfulPin :: deassertPin();
    while(true) {
        delay(1000);
    }
}
[[noreturn]]
void
handleChecksumFail() noexcept {
    terminateExecution();
}

[[noreturn]]
void
tooManyCyclesForTransaction() noexcept {
   terminateExecution();
}

void
configureClockSource() noexcept {
    CCP = 0xD8;
    CLKCTRL.MCLKCTRLA = 0b1000'0000;
    CCP = 0xD8;
    CCP = 0xD8;
    CLKCTRL.OSC20MCTRLA |= 0b0000'0010;
    CCP = 0xD8;
}
void
setup10MHz_CCL() noexcept {
    // use CCL0 and CCL1 to generate a 10MHz clock source
    // connect the output pin (pa7) to the event system which is the 20MHz clock
    Event0.set_generator(gen0::pin_pa7);
    Event0.set_user(user::ccl1_event_a);
    Event0.set_user(user::ccl0_event_a);
    Event1.set_generator(gen::ccl0_out);
    Event1.set_user(user::ccl3_event_b);
    Event1.set_user(user::ccl2_event_b);
    Logic0.enable = true;
    Logic0.input0 = in::feedback; // route the result of the flipflop back to CCL0 to generate the divider effect
    Logic0.input1 = in::disable;
    Logic0.input2 = in::disable;
    Logic0.output = out::enable; // output the 10MHz clock from here
    Logic0.truth = 0b01010101;
    Logic0.sequencer = sequencer::jk_flip_flop;

    Logic1.enable = true;
    Logic1.input0 = in::event_a;
    Logic1.input1 = in::disable;
    Logic1.input2 = in::disable;
    Logic1.output = out::disable;
    Logic1.sequencer = sequencer::disable;
    Logic1.truth = 0b01010101;
    Logic0.init();
    Logic1.init();
    Event0.start();
    Event1.start();
}
void
setupInterruptEdgeDetector_CCL() noexcept {
    Logic2.enable = true;
    Logic2.input0 = in::pin;
    Logic2.input1 = in::disable;
    Logic2.input2 = in::event_b;
    Logic2.output = out::enable;
    Logic2.sequencer = sequencer::disable;
    Logic2.clocksource = clocksource::in2;
    Logic2.edgedetect = edgedetect::enable;
    Logic2.filter = filter::sync;
    Logic2.sequencer = sequencer::disable;
    Logic2.truth = 0b0001'0001;
    Logic2.init();
}
void
setupReadySignal_CCL() noexcept {
    Event5.set_generator(gen5::pin_pf0);
    Event5.set_user(user::ccl3_event_a);
    Logic3.enable = true;
    Logic3.input0 = in::event_a;
    Logic3.input1 = in::disable;
    Logic3.input2 = in::event_b;
    Logic3.output = out::enable;
    Logic3.clocksource = clocksource::in2;
    Logic3.edgedetect = edgedetect::enable;
    Logic3.filter = filter::sync;
    Logic3.sequencer = sequencer::disable;
    Logic3.truth = 0b0001'0001;
    Logic3.init();
    Event5.start();
}
void
setupCCL() noexcept {
    setup10MHz_CCL();
    setupInterruptEdgeDetector_CCL();
    setupReadySignal_CCL();
    Logic::start();
}
// the setup routine runs once when you press reset:
void setup() {
    ResetPin ::configure();
    ResetPin ::assertPin();
    BootSuccessfulPin :: configure();
    BootSuccessfulPin :: deassertPin();
    configureClockSource();
    delay(2000);
    setupPins();
    setupCCL();
    // at this point all of the clock sources are ready
    // now wait for chipset to boot and pic to be booted
    ChipsetBootedPin :: waitUntilPinIsAsserted();
    ResetPin :: deassertPin();
    // no need to wait for the chipset to release control
    while (FailPin::inputLow()) {
        if (Den960::inputAsserted()) {
            break;
        }
    }
    while (FailPin::inputHigh()) {
        if (Den960::inputAsserted()) {
            break;
        }
    }
    BootSuccessfulPin::assertPin();
    // at this point, if we ever go from low to high again then we have a checksum failure
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::FAIL)), handleChecksumFail, RISING);
}
template<bool enable = currentConfiguration.enableOneCycleWaitStates()>
[[gnu::always_inline]]
inline void waitOneBusCycle() noexcept {
    if constexpr (enable) {
        // in this case lets wait at least two bus cycles to be on the safe
        // side
        __builtin_avr_nops(4);
    }
}
[[gnu::always_inline]]
inline void informCPUAndWait() noexcept {
    ReadySyncPin :: pulse();
    ReadyChipsetPin :: waitUntilPinIsDeasserted();
    if constexpr (currentConfiguration.performDoubleCheckLocks()) {
        ReadyChipsetPin :: waitUntilPinIsDeasserted();
    }
    waitOneBusCycle();
}

[[gnu::always_inline]]
inline void waitForCycleEnd() noexcept {
    ReadyChipsetPin :: waitUntilPinIsAsserted();
    if constexpr (currentConfiguration.performDoubleCheckLocks()) {
        ReadyChipsetPin :: waitUntilPinIsAsserted();
    }
    BurstNext :: deassertPin();
    DoCyclePin ::deassertPin();
    waitOneBusCycle();
}
volatile byte numCycles = 0;
[[noreturn]]
void loop() {
    for (;;) {
        // introduce some delay to make sure the bus has time to recover properly
        waitOneBusCycle();
        // okay so we need to wait for DEN to go low
        Den960 :: waitUntilPinIsAsserted();
        if constexpr (currentConfiguration.performDoubleCheckLocks()) {
            Den960 :: waitUntilPinIsAsserted();
        }
        if (numCycles >= currentConfiguration.getMaxNumberOfCyclesBeforePause()) {
            // provide a pause/cooldown period after a new data request to make sure that the bus has time to "cool".
            // Failure to do so can cause very strange checksum failures / chipset faults to happen with the GCM4
            // this is not an issue since the i960 will wait until ready is signaled
            while (numCycles > 0) {
                // use the loop itself to provide some amount of time to cool off
                // numCycles is volatile to prevent the compiler from optimizing any of this away.
               --numCycles;
            }
        }
        // now do the logic
        {
            // tell the chipset we are starting a transaction
            InTransactionPin :: assertPin();
            // okay now we need to emulate a wait loop to allow the chipset time to do its thing
           for (;;) {
               // instead of pulsing do cycle, we just assert it while we wait
               // this has the added benefit of providing proper synchronization between two different clock domains
               // for example, the GCM4 runs at 120MHz while this chip runs at 20MHz. Making the chipset wait provides implicit
               // synchronization
               if (BlastPin::inputAsserted()) {
                   // set BLAST LOW
                   BurstNext ::deassertPin();
               } else {
                   BurstNext ::assertPin();
               }
               // now do the cycle itself
               DoCyclePin :: assertPin();
               // we have entered a new transaction so increment the counter
               // we want to count the number of transaction cycles
               ++numCycles;
               // now wait for the chipset to tell us that it has satisifed the current part of the transaction
               if (BlastPin::inputAsserted()) {
                   // if it turns out that blast is asserted then we break out of this loop and handle it specially
                   break;
               }
               // we are dealing with a burst transaction at this point
               waitForCycleEnd();
               {
                   // let the chipset know that the operation will continue
                   // we repurpose the burst pin for the original purpose of active high implying a continue state!
                   PinAsserter<BurstNext> letChipsetKnowBurstNext;
                   informCPUAndWait();
               }
           }
           // the end of the current transaction needs to be straighline code
           waitForCycleEnd();
           // okay tell the chipset transaction complete
            InTransactionPin :: deassertPin();
        }
        // we have to tie off the transaction itself first
        // let the i960 know and then wait for the chipset to pull MCU READY high
        informCPUAndWait();
        // to make sure the bus has time to recover we can introduce an i960 bus cycle worth of delay
        waitOneBusCycle();
    }
}

