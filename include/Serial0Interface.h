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
//
// Created by jwscoggins on 10/17/21.
//

#ifndef SXCHIPSET_SERIAL0INTERFACE_H
#define SXCHIPSET_SERIAL0INTERFACE_H
#include <Arduino.h>
#include "MemorySpace.h"
//template<Address baseAddress, bool addressDebuggingAllowed, bool defaultAddressDebuggingModeTo = false>
class Serial0Interface : public MemorySpace {
public:
    enum class Registers : uint8_t {
#define TwoByteEntry(Prefix) Prefix ## 0, Prefix ## 1
#define FourByteEntry(Prefix) \
        TwoByteEntry(Prefix ## 0), \
        TwoByteEntry(Prefix ## 1)
#define EightByteEntry(Prefix) \
        FourByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define TwelveByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define SixteenByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        EightByteEntry(Prefix ## 1)
        TwoByteEntry(ConsoleIO),
        TwoByteEntry(ConsoleFlush),
        //FourByteEntry(ConsoleTimeout),
        //TwoByteEntry(ConsoleRXBufferSize),
        //TwoByteEntry(ConsoleTXBufferSize),
        //FourByteEntry(ChipsetClockSpeed),
        //TwoByteEntry(CacheLineCount),
        //TwoByteEntry(CacheLineSize),
        //TwoByteEntry(NumberOfCacheWays),
        //TwoByteEntry(TriggerInterrupt),
        //FourByteEntry(AddressDebuggingFlag),
#undef SixteenByteEntry
#undef TwelveByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
        End,
        ConsoleIO = ConsoleIO0,
        ConsoleFlush = ConsoleFlush0,
        //TriggerInterrupt = TriggerInterrupt0,
        //AddressDebuggingFlag = AddressDebuggingFlag00,
        // we ignore the upper half of the register but reserve it to make sure
    };
    using Self = Serial0Interface;
    using Parent = MemorySpace;
public:
    Serial0Interface(uint32_t baseAddress) : Parent(baseAddress, 1) { }
    ~Serial0Interface() override = default;
private:
    template<unsigned int usecDelay = 100, unsigned long cooloffThreshold = 128>
    uint16_t getConsoleInput() const noexcept {
        static volatile unsigned long numEmptyReads = 0;
        auto result = Serial.read();
        if (result == -1) {
            ++numEmptyReads;
        }
        if (numEmptyReads >= cooloffThreshold) {
            // introduce a cool off period instead of a fixed delay
            while (numEmptyReads != 0) {
                --numEmptyReads;
            }
        }
        // this is to prevent the serial console output from overwhelming the bus and causing a machine check exception from occurring
        // this does not seem to affect system performance at all beyond printing.
        /// @todo introduce bus gating into the management engine based on cycles provided
        if constexpr (usecDelay > 0) {
            delayMicroseconds(usecDelay);
        }
        return result;
    }
    template<unsigned int usecDelay = 100>
    void sendToConsole(char value) noexcept {
        // The serial console is very fast and seems to be out pacing the i960 and the rest of the bus.
        // So introduce this delay after writing to make sure we don't run into problems in the future.
        Serial.write(value);
        if constexpr (usecDelay > 0) {
            delayMicroseconds(usecDelay);
        }
    }
public:
    [[nodiscard]]
    uint16_t
    read(uint32_t address, LoadStoreStyle ) const noexcept override {
        switch (static_cast<uint8_t>(address)) {
            case 0: return getConsoleInput();
            default: return 0;
        }
    }

    void
    write(uint32_t address, SplitWord16 value, LoadStoreStyle) noexcept {
        switch (static_cast<uint8_t>(address)) {
            case 0: sendToConsole(static_cast<char>(value.getWholeValue())); break;
            case 2: Serial.flush(); break;
            default: break;
        }
    }
    uint32_t
    write(uint32_t baseAddress, uint8_t *data, uint32_t count) noexcept override {
        return 0;
    }
    uint32_t
    read(uint32_t baseAddress, uint8_t *data, uint32_t count) noexcept override {
        return 0;
    }
private:
    SplitWord32 value_ {0};
};
#endif //SXCHIPSET_SERIAL0INTERFACE_H
