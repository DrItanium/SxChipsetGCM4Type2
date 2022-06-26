/*
i960SxChipset
Copyright (c) 2020-2021, Joshua Scoggins
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

#ifndef ARDUINO_IOEXPANDERS_H
#define ARDUINO_IOEXPANDERS_H
#include <Arduino.h>
#include "Pinout.h"
#include "i960SxChipset.h"

class ProcessorInterface {
public:
    ProcessorInterface() = delete;
    ~ProcessorInterface() = delete;
    ProcessorInterface(const ProcessorInterface&) = delete;
    ProcessorInterface(ProcessorInterface&&) = delete;
    ProcessorInterface& operator=(const ProcessorInterface&) = delete;
    ProcessorInterface& operator=(ProcessorInterface&&) = delete;
public:
    [[nodiscard]] static constexpr Address getAddress() noexcept { return address_.getWholeValue(); }
    [[nodiscard]] static SplitWord16 getDataBits() noexcept;
    static void setDataBits(uint16_t value) noexcept;
    [[nodiscard]] static LoadStoreStyle getStyle() noexcept;
    [[nodiscard]] static bool isReadOperation() noexcept;
    template<byte offsetMask>
    [[nodiscard]] static auto getCacheOffsetEntry() noexcept { return getCacheOffsetEntry<offsetMask>(address_); }
    template<byte offsetMask>
    [[nodiscard]] static auto getCacheOffsetEntry(SplitWord32 word) noexcept { return (word.bytes[0] >> 1) & offsetMask; }
    static void setupDataLinesForWrite() noexcept;
    static void setupDataLinesForRead() noexcept;
private:
    static constexpr SplitWord16 extractAddress(uint32_t value) noexcept {
        // okay first step is to get the part of the value that we actually care about
        constexpr uint32_t LowerPortion =  0b0000000000000000'0000001111111111;
        constexpr uint32_t UpperPortion =  0b0000000000000011'1111000000000000;
        auto lowerPart = LowerPortion & value;
        auto upperPart = (UpperPortion & value) >> 2;
        // The AHC158 inverts the output
        return SplitWord16(~static_cast<uint16_t>(lowerPart | upperPart));
    }
    static SplitWord16 getHalfAddress() noexcept;
public:
    static void newAddress() noexcept;
    static void full32BitUpdate() noexcept {
        // The multiplexed address lines are not mapped 01/23
        // when high: 0-7, 16-23
        // when low: 8-15, 24-31
        // They are laid out 13/02 so it is necessary to unpack them and assign them as needed
        digitalWrite<i960Pinout::MUXSel0, LOW>();
        auto addressA = getHalfAddress();
        digitalWrite<i960Pinout::MUXSel0, HIGH>();
        auto addressB = getHalfAddress();
        auto lower = addressA.getLowerHalf();
        auto highest = addressA.getUpperHalf();
        auto lowest = addressB.getLowerHalf();
        auto higher = addressB.getUpperHalf();
        address_.bytes[0] = lowest;
        address_.bytes[1] = lower;
        address_.bytes[2] = higher;
        address_.bytes[3] = highest;
    }

    template<bool inDebugMode>
    static void newDataCycle() noexcept {
        full32BitUpdate<inDebugMode>();
        //delayMicroseconds(10); // this gets rid of a chipset halt problem that I'm not sure where is coming from
        //Serial.print(F("Address 0x")); Serial.println(address_.getWholeValue(), HEX);
        if (ProcessorInterface::isReadOperation()) {
            setupDataLinesForRead();
            getMemory().handleReadRequest();
        } else {
            setupDataLinesForWrite();
            getMemory().handleWriteRequest();
        }
    }
    template<bool advanceAddress = true>
    static void burstNext() noexcept {
        if constexpr (advanceAddress) {
            address_.wholeValue_ += 2;
        }
        // a test to see if we are running too fast and not allowing enough charge time between transaction points
        //delayMicroseconds(10);
    }
    /**
     * @brief Return the least significant byte of the address, useful for CoreChipsetFeatures
     * @return The LSB of the address
     */
    [[nodiscard]] static auto getPageOffset() noexcept { return address_.bytes[0]; }
    [[nodiscard]] static auto getPageIndex() noexcept { return address_.bytes[1]; }
    static void begin() noexcept;
private:
    static inline SplitWord32 address_{0};
    static inline bool isWriteOperation_ = false;
};

#endif //ARDUINO_IOEXPANDERS_H
