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


enum class PICPinout : int {
    // interrupt output pins from CCLs
    INT0_OUT = PIN_PC3,
    INT1_OUT = PIN_PD3,
    INT2_OUT = PIN_PF3,
    INT3_OUT = PIN_PA3,
    // clock related activities
    CLK_READY = PIN_PA7,
    CLK = PIN_PA2,
    CLK2 = PIN_PA0,
    // communication channel activities
    CHIPSET_COMM_TX = PIN_PA4,
    CHIPSET_COMM_RX = PIN_PA5,
    // Miscellaneous controls
    BUF_ENABLE = PIN_PF4,
    PIC_BOOTED = PIN_PF5,

};

using ClockReadyPin = ActiveLowInputPin<PICPinout::CLK_READY>;
using ClkPin = ActiveLowInputPin<PICPinout::CLK>;
using Clk2Pin = ActiveLowInputPin<PICPinout::CLK2>;
using BootedPin = ActiveLowOutputPin<PICPinout::PIC_BOOTED>;
using BufferEnablePin = ActiveLowOutputPin<PICPinout::BUF_ENABLE>;
using Int0Output = ActiveHighOutputPin<PICPinout::INT0_OUT>;
using Int1Output = ActiveHighOutputPin<PICPinout::INT1_OUT>;
using Int2Output = ActiveHighOutputPin<PICPinout::INT2_OUT>;
using Int3Output = ActiveHighOutputPin<PICPinout::INT3_OUT>;

/**
 * @brief Abstract interface to the Serial object used as the communication and configuration channel
 */
decltype(Serial)& CommunicationChannel = Serial;



void
configureClockSource() noexcept {
    CCP = 0xD8;
    CLKCTRL.MCLKCTRLA |= 0b0000'0011;
    CCP = 0xD8;

    CCP = 0xD8;
    CLKCTRL.OSC20MCTRLA |= 0b0000'0010;
    CCP = 0xD8;
}
void
configConfigure10MHzExternalSource() {
    ClkPin::configure();
    Event0.set_generator(gen0::pin_pa2);
    Event0.set_generator(user::ccl0_event_b);
    Event0.set_generator(user::ccl1_event_b);
    Event0.set_generator(user::ccl2_event_b);
    Event0.set_generator(user::ccl3_event_b);
    Event0.start();
}
void
configChipsetInterruptLine() noexcept {
    Logic0.enable = true;
    Logic0.edgedetect = edgedetect::enable;
    Logic0.filter = filter::sync;
    Logic0.input0 = in::disable;
    Logic0.input1 = in::input_pullup;
    Logic0.input2 = in::event_b;
    Logic0.clocksource = clocksource::in2;
    Logic0.output = out::enable;
    Logic0.sequencer = sequencer::disable;
    Logic0.truth = 0b0001'0001;
    Logic0.init();
}
void
configINT0() noexcept {
    // provide a default configuration of doing nothing!
    Logic1.enable = true;
    Logic1.input0 = in::disable;
    Logic1.input1 = in::disable;
    Logic1.input2 = in::event_b;
    Logic1.clocksource = clocksource::in2;
    Logic1.output = out::enable;
    Logic1.sequencer = sequencer::disable;
    Logic1.edgedetect = edgedetect::enable;
    Logic1.filter = filter::sync;
    Logic1.truth = 0b0001'0001;
    Logic1.init();
}

void
configINT1() noexcept {
    // provide a default configuration of doing nothing!
    Logic2.enable = true;
    Logic2.input0 = in::disable;
    Logic2.input1 = in::disable;
    Logic2.input2 = in::event_b;
    Logic2.clocksource = clocksource::in2;
    Logic2.output = out::enable;
    Logic2.sequencer = sequencer::disable;
    Logic2.edgedetect = edgedetect::enable;
    Logic2.filter = filter::sync;
    Logic2.truth = 0b0001'0001;
    Logic2.init();
}

void
configINT2() noexcept {
    // provide a default configuration of doing nothing!
    Logic3.enable = true;
    Logic3.input0 = in::disable;
    Logic3.input1 = in::disable;
    Logic3.input2 = in::event_b;
    Logic3.clocksource = clocksource::in2;
    Logic3.output = out::enable;
    Logic3.sequencer = sequencer::disable;
    Logic3.edgedetect = edgedetect::enable;
    Logic3.filter = filter::sync;
    Logic3.truth = 0b0001'0001;
    Logic3.init();
}

void
configurePIC() noexcept {
    // we can set this all up ahead of time
    configConfigure10MHzExternalSource();
    configChipsetInterruptLine();
    configINT0();
    configINT1();
    configINT2();
    Logic::start();
}
// the setup routine runs once when you press reset:
void setup() {
    BootedPin::configure();
    BootedPin::deassertPin();
    ClockReadyPin::configure();

    delay(1000);
    while (ClockReadyPin::inputDeasserted()) {
        delay(10);
    }
    configureClockSource();
    configurePins<BufferEnablePin,
            Int0Output,
            Int1Output,
            Int2Output,
            Int3Output>();
    deassertPins<BufferEnablePin,
            Int0Output,
            Int1Output,
            Int2Output,
            Int3Output>();
    configurePIC();
    CommunicationChannel.begin(115200);
    // the booted pin is the reset pin conceptually
    delay(2000);
    // no need to wait for the chipset to release control
}
volatile bool commandComplete = false;
volatile byte opcode = 0;
void
loop() {
    if (commandComplete) {
        switch (opcode)  {
            default:
                break;
        }
        opcode = 0;
        commandComplete = false;
    }
}

void
serialEvent() {
    // absorb the communication data from the chipset communication channel
}