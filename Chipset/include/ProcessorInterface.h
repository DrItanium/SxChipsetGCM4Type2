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
public:
    static void newAddress() noexcept;

    template<bool inDebugMode>
    static void newDataCycle() noexcept {
        // read twice to see if we can't get around a really strange initial state problem
        newAddress();
        //delayMicroseconds(10); // this gets rid of a chipset halt problem that I'm not sure where is coming from
        //Serial.print(F("Address 0x")); Serial.println(address_.getWholeValue(), HEX);
        if (isReadOperation()) {
            setupDataLinesForRead();
            getMemory().handleReadRequest();
        } else {
            setupDataLinesForWrite();
            getMemory().handleWriteRequest();
        }
    }
    static void begin() noexcept;
private:
    static inline SplitWord32 address_{0};
    static inline bool isWriteOperation_ = false;
};

#endif //ARDUINO_IOEXPANDERS_H
