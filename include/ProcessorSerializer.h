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
#include <libbonuspin.h>
#include "Pinout.h"

class ProcessorInterface {
public:
/**
 * @brief Sx Load/Store styles that the processor will request
 */
    enum class LoadStoreStyle : uint8_t {
        // based off of BE0,BE1 pins
        Full16 = 0b00,
        Upper8 = 0b01,
        Lower8 = 0b10,
        None = 0b11,
    };
    enum class IOExpanderAddress : byte {
        DataLines = 0b000,
        Lower16Lines,
        Upper16Lines,
        MemoryCommitExtras,
        OtherDevice0,
        OtherDevice1,
        OtherDevice2,
        OtherDevice3,
    };
// layout of the extra memory commit expander
// PA0 - BurstAddress1 - input
// PA1 - BurstAddress2 - input
// PA2 - BurstAddress3 - input
// PA3 - BE0_ - input
// PA4 - BE1_ - input
// PA5 - HOLD  - output
// PA6 - HLDA  - input
// PA7 - _LOCK - output
// PB0-PB7 - Unused

    enum class ExtraGPIOExpanderPinout : decltype(A0) {
        BurstAddress1,
        BurstAddress2,
        BurstAddress3,
        ByteEnable0,
        ByteEnable1,
        HOLD,
        HLDA,
        LOCK_,
        // add support for upto 256 spi devices
        Unused0,
        Unused1,
        Unused2,
        Unused3,
        Unused4,
        Unused5,
        Unused6,
        Unused7,
        Count,
    };
    static_assert(static_cast<int>(ExtraGPIOExpanderPinout::Count) == 16);
    template<IOExpanderAddress addr, int enablePin = static_cast<int>(i960Pinout::GPIOSelect)>
    using IOExpander = bonuspin::MCP23S17<static_cast<int>(addr), enablePin>;
public:
    void begin() noexcept;
    [[nodiscard]] Address getAddress() noexcept;
    [[nodiscard]] uint16_t getDataBits() noexcept;
    void setDataBits(uint16_t value) noexcept;
    [[nodiscard]] uint8_t getByteEnableBits() noexcept;
    [[nodiscard]] decltype(LOW) getByteEnable0() noexcept;
    [[nodiscard]] decltype(HIGH) getByteEnable1() noexcept;
    [[nodiscard]] LoadStoreStyle getStyle() noexcept;
    [[nodiscard]] uint8_t getBurstAddressBits() noexcept;

    static constexpr Address getBurstAddress(Address base, Address burstBits) noexcept {
        return (base & (~0b1110)) | burstBits;
    }
    [[nodiscard]] Address getBurstAddress(Address base) noexcept;
    [[nodiscard]] Address getBurstAddress() noexcept;

    [[nodiscard]] bool isReadOperation() const noexcept;
    [[nodiscard]] bool isWriteOperation() const noexcept;
    void setHOLDPin(decltype(LOW) value) noexcept;
    void setLOCKPin(decltype(LOW) value) noexcept;
    static constexpr auto computeAddressStart(Address start, Address size, Address count) noexcept {
        return start + (size * count);
    }
/**
 * Normally generated by the linker as the value used for validation purposes
 * on system bootup. Since we are on a microcontroller, this can be done
 * dynamically. This is taken from start.ld in the SA/SB Reference manual's
 * Appendix D.
 */
    static constexpr auto computeCS1(uint32_t satPtr, uint32_t pcrbPtr, uint32_t startIP) noexcept {
        return - (satPtr + pcrbPtr + startIP);
    }
private:
    IOExpander<IOExpanderAddress::DataLines> dataLines_;
    IOExpander<IOExpanderAddress::Lower16Lines> lower16_;
    IOExpander<IOExpanderAddress::Upper16Lines> upper16_;
    IOExpander<IOExpanderAddress::MemoryCommitExtras> extra_;
    bool initialized_ = false;
};

// 8 IOExpanders to a single enable line for SPI purposes
// 4 of them are reserved

#endif //ARDUINO_IOEXPANDERS_H
