/*
hitagimon
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
// Created by jwscoggins on 6/16/22.
//

#include "chipset/GPIO.h"
GPIOEngine::GPIOEngine() : raw_(memory<GPIOEngine::Registers>(getGPIOEngineBaseAddress())), initialized_(false) { }
GPIOEngine::~GPIOEngine() {}

void
GPIOEngine::begin() {
    if (!initialized_) {
        initialized_ = true;
    }
    // nothing to do
}

volatile GPIOEngine::Port&
GPIOEngine::getPort(int index) {
    return raw_.ports[index & 0x1F];
}
volatile const GPIOEngine::Port&
GPIOEngine::getPort(int index) const {
    return raw_.ports[index & 0x1F];
}

bool
GPIOEngine::Port::pinValid(int index) const volatile {
    // port relative index needs to be computed
    int offset = index % 32;
    // now get the mask offset
    uint32_t targetMask = 1 << offset;
    return (targetMask & mask) != 0;
}
bool
GPIOEngine::validPin(int pinIndex) const {
    int targetPort = pinToPort(pinIndex);
    if (validPort(targetPort)) {
        volatile const GPIOEngine::Port& port = getPort(targetPort);
        return port.pinValid(pinIndex);
    } else {
        return false;
    }
}

GPIOEngine&
getGPIOEngine() {
    static GPIOEngine theEngine;
    return theEngine;
}
void digitalWrite(int index, bool value) {
   GPIOEngine& gpio = getGPIOEngine();
   // we can safely ignore validity checks as the chipset will just accept modifications to invalid pins but not express them
   int targetPort = gpio.pinToPort(index);
   int portOffset = gpio.portOffset(index);
    int pinMask = 1 << portOffset;
   volatile GPIOEngine::Port& port = gpio.getPort(targetPort);
   uint32_t outputContents = port.output;
   if (value) {
       //set the pin
       outputContents |= pinMask;
   } else {
       outputContents &= ~pinMask;
   }
   port.output = outputContents;
}
bool
digitalRead(int index) {
    GPIOEngine& gpio = getGPIOEngine();
    // we can safely ignore validity checks as the chipset will just accept modifications to invalid pins but not express them
    int targetPort = gpio.pinToPort(index);
    int portOffset = gpio.portOffset(index);
    int pinMask = 1 << portOffset;
    volatile const GPIOEngine::Port& port = gpio.getPort(targetPort);
    return (port.input & pinMask) != 0;
}
namespace
{
    void
    configurePinForOutput(int index) {
        GPIOEngine& gpio = getGPIOEngine();
        volatile GPIOEngine::Port& port = gpio.getPort(gpio.pinToPort(index));
        uint32_t mask = 1 << gpio.portOffset(index);
        port.direction |= mask;
        port.pullup &= ~mask;
    }
    void
    configurePinForInput(int index) {
        GPIOEngine& gpio = getGPIOEngine();
        volatile GPIOEngine::Port& port = gpio.getPort(gpio.pinToPort(index));
        uint32_t mask = 1 << gpio.portOffset(index);
        port.direction &= ~mask;
        port.pullup &= ~mask;
    }
    void
    configurePinForInputPullup(int index) {
        GPIOEngine& gpio = getGPIOEngine();
        volatile GPIOEngine::Port& port = gpio.getPort(gpio.pinToPort(index));
        uint32_t mask = 1 << gpio.portOffset(index);
        port.direction &= ~mask;
        port.pullup |= mask;
    }
}
void
pinMode(int index, PinDirection dir) {
    switch (dir) {
        case OUTPUT:
            configurePinForOutput(index);
            break;
        case INPUT:
            configurePinForInput(index);
            break;
        case INPUT_PULLUP:
            configurePinForInputPullup(index);
            break;
        default:
            break;
    }
}
void
portWrite(int index, uint32_t value) {
    getGPIOEngine().getPort(index).output = value;
}
uint32_t
portRead(int index) {
    return getGPIOEngine().getPort(index).input;
}
bool
validPort(int index) {
    return getGPIOEngine().validPort(index);
}
bool
validPin(int index) {
    GPIOEngine& gpio = getGPIOEngine();
    return gpio.getPort(gpio.pinToPort(index)).pinValid(gpio.portOffset(index));
}
