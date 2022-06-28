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
#include "chipset/SPI.h"
#include <cmath>

SPIEngine::SPIEngine() :
        raw_(memory<SPIEngine::RawView>(getSPIEngineBaseAddress())),
        currentTransferRate_(getMaximumTransferRate()),
        mode_(0),
        dataOrder_(0) { }
SPIEngine::~SPIEngine() { }


void
SPIEngine::transfer(Buffer src, Buffer dest, uint8_t count, bool overwriteSource) {
    if (available()) {
        internalRequest_.src = src;
        internalRequest_.dest = dest;
        internalRequest_.transferRate = currentTransferRate_;
        internalRequest_.flags.count = count;
        internalRequest_.flags.mode = mode_;
        internalRequest_.flags.overwriteSrc = overwriteSource;
        raw_.requestBaseAddress = &internalRequest_;
        // now we are going to implicitly wait until SPI is done transferring, not much we can do here until it is finished!
        // but for future compatiblity purposes lets put a wait loop here. It is implicitly blocking on the 1284p based chipset
        while (!raw_.ready);
    }
}
void
SPIEngine::setCurrentTransferRate(uint32_t value) {
    currentTransferRate_ = std::min(value, getMaximumTransferRate());
}
void
SPIEngine::begin()
{

}

SPIEngine&
getSPIEngine() {
    static SPIEngine theEngine;
    return theEngine;
}

uint8_t
SPIEngine::transfer(uint8_t value) {
    uint8_t storage = value;
    transfer(&storage, sizeof(storage));
    return storage;
}
