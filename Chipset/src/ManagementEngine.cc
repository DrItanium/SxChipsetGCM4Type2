/*
i960SxChipset
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
//
// Created by jwscoggins on 6/21/22.
//
#include "ManagementEngine.h"
#include "Pinout.h"
#include "ProcessorInterface.h"

namespace ManagementEngine
{
    void
    waitForCycleUnlock() noexcept {
        while (DigitalPin<i960Pinout::DoCycle>::isDeasserted());
    }
    bool informCPU() noexcept {
        // don't pulse READY, instead just pull it low, the interrupt latency on the 4809 is horrible
        // so we just pull Ready high as soon as we get the next phase in.
        DigitalPin<i960Pinout::Ready>::assertPin();
        // synchronize and wait for the i960 to tell me it is ready to go
        while (DigitalPin<i960Pinout::DoCycle>::isAsserted());
        // make sure that we just wait for the gating signal before continuing
        while (DigitalPin<i960Pinout::InTransaction>::isAsserted() && DigitalPin<i960Pinout::BurstNext>::isDeasserted());
        bool outcome = DigitalPin<i960Pinout::InTransaction>::isDeasserted();
        DigitalPin<i960Pinout::Ready>::deassertPin();
        return outcome;
    }
    void waitForBootSignal() noexcept {
        while (DigitalPin<i960Pinout::SuccessfulBoot>::read() == LOW);
        attachInterrupt(i960Pinout::SuccessfulBoot,
                        []() { signalHaltState("CHECKSUM FAILURE"); },
                        FALLING);
    }
    void
    configure() noexcept {
        // by resetting the 4809 we are resetting the entire system
        DigitalPin<i960Pinout::Reset4809>::configure();
        // Reset960 is waitboot960 in type 3.00 but in 3.01 it is actually used a chipset booted pin instead
        DigitalPin<i960Pinout::ChipsetBooted>::configure();
        DigitalPin<i960Pinout::InTransaction>::configure();
        DigitalPin<i960Pinout::DoCycle>::configure();
        DigitalPin<i960Pinout::BurstNext>::configure();
        DigitalPin<i960Pinout::SuccessfulBoot>::configure();
    }
    void
    holdInReset() noexcept {
        DigitalPin<i960Pinout::Reset4809>::assertPin();
    }
    void
    allowBoot() noexcept {
        DigitalPin<i960Pinout::Reset4809>::deassertPin();
    }
    void
    chipsetIsInSetup() noexcept {
        DigitalPin<i960Pinout::ChipsetBooted>::deassertPin();
    }
    void
    chipsetReady() noexcept {
        DigitalPin<i960Pinout::ChipsetBooted>::assertPin();
    }
    bool
    isLastCycleOfTransaction() noexcept {
        return DigitalPin<i960Pinout::BurstNext>::isDeasserted();
    }
}