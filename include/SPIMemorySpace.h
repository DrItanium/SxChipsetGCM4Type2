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
// Created by jwscoggins on 6/24/22.
//

#ifndef SXCHIPSETGCM4TYPE2_SPIMEMORYSPACE_H
#define SXCHIPSETGCM4TYPE2_SPIMEMORYSPACE_H
#include <Arduino.h>
#include <SPI.h>
#include "SinglePageMemorySpace.h"

class SPIMemorySpace : public SinglePageMemorySpace {
public:
    using Self = SPIMemorySpace;
    using Parent = SinglePageMemorySpace;
public:
    SPIMemorySpace() : Parent() { }
    ~SPIMemorySpace() override = default;
private:
    void performSPITransfer() noexcept;
public:
    void
    write32(uint32_t address, uint32_t value) noexcept override {
        switch (static_cast<uint8_t>(address)) {
            case 4:
                requestPointer_.setWholeValue(value);
                performSPITransfer();
                break;
            default:
                break;
        }
    }
    uint32_t
    read32(uint32_t address) const noexcept override {
        switch (static_cast<uint8_t>(address) >> 2) {
            case 0: return 0xFFFF'FFFF;
            case 1: return requestPointer_.getWholeValue();
            case 2: return maximumSpeed_.getWholeValue();
            /// @todo implement a proper ready flag
            case 3: return 0xFFFF'FFFF;
            default:
                return 0;
        }
    }
private:
    SplitWord32 requestPointer_;
    SplitWord32 maximumSpeed_{MAX_SPI};
};

#endif //SXCHIPSETGCM4TYPE2_SPIMEMORYSPACE_H
