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
    void performSPITransfer();
public:
    void write(uint32_t address, SplitWord16 value, LoadStoreStyle lss) noexcept override {
        // ignore non Full16 writes!
        if (lss != LoadStoreStyle::Full16) {
            return;
        }
        switch (static_cast<uint8_t>(address)) {
            // don't allow writes to valid
            case 4:
                requestPointer_.words_[0] = value;
                break;
            case 6:
                requestPointer_.words_[1] = value;
                performSPITransfer();
                break;
            default:
                break;
        }
    }
    uint16_t read(uint32_t address, LoadStoreStyle lss) const noexcept override {
        switch (static_cast<uint8_t>(address)) {
            case 0: return 0xFFFF;
            case 2: return 0xFFFF;
            case 4: return requestPointer_.getLowerHalf();
            case 6: return requestPointer_.getUpperHalf();
            case 8: return maximumSpeed_.getLowerHalf();
            case 10: return maximumSpeed_.getUpperHalf();
            /// @todo reimplement a proper ready flag later on
            case 12: return 0xFFFF;
            case 14: return 0xFFFF;
            default:
                return 0;
        }
    }
private:
    SplitWord32 requestPointer_;
    SplitWord32 maximumSpeed_{MAX_SPI};
};

#endif //SXCHIPSETGCM4TYPE2_SPIMEMORYSPACE_H
