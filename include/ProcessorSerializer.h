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
#include <SPI.h>
#include "Pinout.h"
#include "i960SxChipset.h"

class ProcessorInterface {
    enum class MCP23x17Registers : byte {
        IODIRA = 0,
        IODIRB,
        IPOLA,
        IPOLB,
        GPINTENA,
        GPINTENB,
        DEFVALA,
        DEFVALB,
        INTCONA,
        INTCONB,
        _IOCONA,
        _IOCONB,
        GPPUA,
        GPPUB,
        INTFA,
        INTFB,
        INTCAPA,
        INTCAPB,
        GPIOA,
        GPIOB,
        OLATA,
        OLATB,
        OLAT = OLATA,
        GPIO = GPIOA,
        IOCON = _IOCONA,
        IODIR = IODIRA,
        INTCAP = INTCAPA,
        INTF = INTFA,
        GPPU = GPPUA,
        INTCON = INTCONA,
        DEFVAL = DEFVALA,
        GPINTEN = GPINTENA,
        IPOL = IPOLA,
    };
    enum class IOExpanderAddress : byte {
        DataLines = 0b0000,
        Lower16Lines = 0b0010,
        Upper16Lines = 0b0100,
        MemoryCommitExtras = 0b0110,
        OtherDevice0 = 0b1000,
        OtherDevice1 = 0b1010,
        OtherDevice2 = 0b1100,
        OtherDevice3 = 0b1110,
    };
    static constexpr byte generateReadOpcode(ProcessorInterface::IOExpanderAddress address) noexcept {
        return 0b0100'0001 | static_cast<uint8_t>(address);
    }
    static constexpr byte generateWriteOpcode(ProcessorInterface::IOExpanderAddress address) noexcept {
        return 0b0100'0000 | static_cast<uint8_t>(address);
    }
    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    static SplitWord16 read16() noexcept {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        SplitWord16 output(0);
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
#if defined(ARDUINO_AVR_ATmega1284)
        SPDR = generateReadOpcode(addr);
        /*
         * The following NOP introduces a small delay that can prevent the wait
         * loop form iterating when running at the maximum speed. This gives
         * about 10% more speed, even if it seems counter-intuitive. At lower
         * speeds it is unnoticed.
         */
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait

        SPDR = static_cast<byte>(opcode) ;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait

        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        auto lower = SPDR;
        SPDR = 0;
        asm volatile("nop");
        {
            output.bytes[0] = lower;
        }
        while (!(SPSR & _BV(SPIF))) ; // wait
        output.bytes[1] = SPDR;
#else
        SPI.transfer(generateReadOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        output.bytes[0] = SPI.transfer(0);
        output.bytes[1] = SPI.transfer(0);
#endif
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
        return output;
    }
    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    static uint8_t read8() noexcept {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
#if defined(ARDUINO_AVR_ATmega1284)
        SPDR = generateReadOpcode(addr);
        /*
         * The following NOP introduces a small delay that can prevent the wait
         * loop form iterating when running at the maximum speed. This gives
         * about 10% more speed, even if it seems counter-intuitive. At lower
         * speeds it is unnoticed.
         */
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = static_cast<byte>(opcode);
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
#else
        SPI.transfer(generateReadOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        auto outcome = SPI.transfer(0);
#endif
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
#if defined(ARDUINO_AVR_ATmega1284)
        return SPDR;
#else
      return outcome;
#endif
    }

    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    static void write16(uint16_t value) noexcept {
        SplitWord16 valueDiv(value);
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
#if defined(ARDUINO_AVR_ATmega1284)
        SPDR = generateWriteOpcode(addr);
        /*
         * The following NOP introduces a small delay that can prevent the wait
         * loop form iterating when running at the maximum speed. This gives
         * about 10% more speed, even if it seems counter-intuitive. At lower
         * speeds it is unnoticed.
         */
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = static_cast<byte>(opcode);
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = valueDiv.bytes[0];
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = valueDiv.bytes[1];
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
#else
    SPI.transfer(generateWriteOpcode(addr));
    SPI.transfer(static_cast<byte>(opcode));
    SPI.transfer(valueDiv.bytes[0]);
    SPI.transfer(valueDiv.bytes[1]);
#endif
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
    }
    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    static void write8(uint8_t value) noexcept {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
#if defined(ARDUINO_AVR_ATmega1284)
        SPDR = generateWriteOpcode(addr);
        /*
         * The following NOP introduces a small delay that can prevent the wait
         * loop form iterating when running at the maximum speed. This gives
         * about 10% more speed, even if it seems counter-intuitive. At lower
         * speeds it is unnoticed.
         */
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = static_cast<byte>(opcode);
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = value;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
#else
    SPI.transfer(generateWriteOpcode(addr));
    SPI.transfer(static_cast<byte>(opcode));
    SPI.transfer(value);
#endif
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
    }
    template<IOExpanderAddress addr, bool standalone = true>
    static inline SplitWord16 readGPIO16() noexcept {
        return read16<addr, MCP23x17Registers::GPIO, standalone>();
    }
    template<IOExpanderAddress addr, bool standalone = true>
    static inline void writeGPIO16(uint16_t value) noexcept {
        write16<addr, MCP23x17Registers::GPIO, standalone>(value);
    }
    template<IOExpanderAddress addr, bool standalone = true>
    static inline void writeDirection(uint16_t value) noexcept {
        write16<addr, MCP23x17Registers::IODIR, standalone>(value);
    }
public:
    ProcessorInterface() = delete;
    ~ProcessorInterface() = delete;
    ProcessorInterface(const ProcessorInterface&) = delete;
    ProcessorInterface(ProcessorInterface&&) = delete;
    ProcessorInterface& operator=(const ProcessorInterface&) = delete;
    ProcessorInterface& operator=(ProcessorInterface&&) = delete;
public:
    static void begin() noexcept;
    [[nodiscard]] static constexpr Address getAddress() noexcept { return address_.getWholeValue(); }
    [[nodiscard]] static SplitWord16 getDataBits() noexcept {
        if constexpr (TargetBoard::onType1() || TargetBoard::onType2() || TargetBoard::onType3()) {
            return readGPIO16<ProcessorInterface::IOExpanderAddress::DataLines>();
        } else {
            // stub out
            return SplitWord16(0);
        }
    }
    static void setDataBits(uint16_t value) noexcept {
        if constexpr (TargetBoard::onType1() || TargetBoard::onType2() || TargetBoard::onType3()) {
            // the latch is preserved in between data line changes
            // okay we are still pointing as output values
            // check the latch and see if the output value is the same as what is latched
            if (latchedDataOutput.getWholeValue() != value) {
                latchedDataOutput.wholeValue_ = value;
                writeGPIO16<ProcessorInterface::IOExpanderAddress::DataLines>(latchedDataOutput.getWholeValue());
            }
        } else {
            // do nothing
        }
    }
    [[nodiscard]] static auto getStyle() noexcept {
#ifdef ARDUINO_AVR_ATmega1284
        return static_cast<LoadStoreStyle>((PINA & 0b11'0000));
#else
        auto lower = static_cast<byte>(DigitalPin<i960Pinout::BE0>::read()) << 4;
        auto upper = static_cast<byte>(DigitalPin<i960Pinout::BE1>::read()) << 5;
        return static_cast<LoadStoreStyle>(lower | upper);
#endif
    }
    [[nodiscard]] static bool isReadOperation() noexcept { return DigitalPin<i960Pinout::W_R_>::isAsserted(); }
    [[nodiscard]] static auto getCacheOffsetEntry() noexcept { return cacheOffsetEntry_; }
    inline static void setupDataLinesForWrite() noexcept {
        if constexpr (TargetBoard::onType1() || TargetBoard::onType2() || TargetBoard::onType3()) {
            if (!dataLinesDirection_) {
                dataLinesDirection_ = ~dataLinesDirection_;
                writeDirection<ProcessorInterface::IOExpanderAddress::DataLines>(0xFFFF);
            }
        } else {
            // do nothing
        }
    }
    inline static void setupDataLinesForRead() noexcept {
        if constexpr (TargetBoard::onType1() || TargetBoard::onType2() || TargetBoard::onType3()) {
            if (dataLinesDirection_) {
                dataLinesDirection_ = ~dataLinesDirection_;
                writeDirection<ProcessorInterface::IOExpanderAddress::DataLines>(0);
            }
        } else {
            // do nothing
        }
    }
private:
    template<bool useInterrupts = true>
    static byte getUpdateKind() noexcept {
        if constexpr (!useInterrupts) {
            return 0;
        } else {
#if defined(ARDUINO_AVR_ATmega1284)
            if constexpr (TargetBoard::onType1()) {
                switch (PIND & 0b1001'0000) {
                    case 0b0000'0000: return 0b0000;
                    case 0b0001'0000: return 0b0011;
                    case 0b1000'0000: return 0b1100;
                    case 0b1001'0000: return 0b1111;
                    default: return 0b0000;
                }
            } else if constexpr (TargetBoard::onType2()) {
                // even though three of the four pins are actually in use, I want to eventually diagnose the problem itself
                // so this code is ready for that day
                return PINA & 0b0000'1111;
            } else {
                return 0;
            }
#else
            auto a = static_cast<byte>(DigitalPin<i960Pinout::INT_EN0>::read());
            auto b = static_cast<byte>(DigitalPin<i960Pinout::INT_EN1>::read()) << 1;
            auto c = static_cast<byte>(DigitalPin<i960Pinout::INT_EN2>::read()) << 2;
            auto d = static_cast<byte>(DigitalPin<i960Pinout::INT_EN3>::read()) << 3;
            return a | b | c | d;
#endif
        }
    }
    template<byte offsetMask>
    inline static void full32BitUpdate() noexcept {
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
#if defined(ARDUINO_AVR_ATmega1284)
        constexpr auto Lower16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        constexpr auto Upper16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Upper16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIO);
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Lower16Opcode;
        /*
         * The following NOP introduces a small delay that can prevent the wait
         * loop form iterating when running at the maximum speed. This gives
         * about 10% more speed, even if it seems counter-intuitive. At lower
         * speeds it is unnoticed.
         */
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto lowest = SPDR;
        SPDR = 0;
        asm volatile("nop");
        {
            // inside of here we have access to 12 cycles to play with, so let's actually do some operations while we wait
            // put scope ticks to force the matter
            cacheOffsetEntry_ = (lowest >> 1) & offsetMask; // we want to make this quick to increment
            address_.bytes[0] = lowest;
        }
        while (!(SPSR & _BV(SPIF))); // wait
        auto lower = SPDR;
        DigitalPin<i960Pinout::GPIOSelect>::pulse<HIGH>(); // pulse high
        SPDR = Upper16Opcode;
        asm volatile("nop");
        {
            address_.bytes[1] = lower;
            // interleave this operation in, can't get more complex than this
        }
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto higher = SPDR;
        SPDR = 0;
        asm volatile("nop");
        {
            address_.bytes[2] = higher;
        }
        while (!(SPSR & _BV(SPIF))); // wait
        address_.bytes[3] = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
#else
        address_.setLowerHalf(readGPIO16<IOExpanderAddress::Lower16Lines>());
        address_.setUpperHalf(readGPIO16<IOExpanderAddress::Upper16Lines>());
        cacheOffsetEntry_ = (address_.bytes[0] >> 1) & offsetMask; // we want to make this quick to increment
#endif
    }
    template<byte offsetMask>
    static void lower16Update() noexcept {
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
#if defined(ARDUINO_AVR_ATmega1284)
        constexpr auto Lower16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIO);
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Lower16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto lowest = SPDR;
        SPDR = 0;
        asm volatile("nop");
        {
            // inside of here we have access to 12 cycles to play with, so let's actually do some operations while we wait
            // put scope ticks to force the matter
            cacheOffsetEntry_ = (lowest >> 1) & offsetMask; // we want to make this quick to increment
            address_.bytes[0] = lowest;
        }
        while (!(SPSR & _BV(SPIF))); // wait
        auto lower = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        address_.bytes[1] = lower;
#else
        address_.setLowerHalf(readGPIO16<IOExpanderAddress::Lower16Lines>());
        cacheOffsetEntry_ = (address_.bytes[0] >> 1) & offsetMask; // we want to make this quick to increment
#endif
    }
    static void upper16Update() noexcept {
        // only read the upper 16-bits
#if defined(ARDUINO_AVR_ATmega1284)
        constexpr auto Upper16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Upper16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIO);
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Upper16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto higher = SPDR;
        SPDR = 0;
        asm volatile("nop");
        {
            address_.bytes[2] = higher;
        }
        while (!(SPSR & _BV(SPIF))); // wait
        auto highest = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        address_.bytes[3] = highest;
#else
        address_.setUpperHalf(readGPIO16<IOExpanderAddress::Upper16Lines>());
#endif
    }
    static void updateHighest8() noexcept {
        // only read the upper 8 bits
#if defined(ARDUINO_AVR_ATmega1284)
        constexpr auto Upper16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Upper16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIOB);
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        SPDR = Upper16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto highest = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        address_.bytes[3] = highest;
#else
        address_.bytes[3] = read8<IOExpanderAddress::Upper16Lines, MCP23x17Registers::GPIOB>();
#endif
    }
    static void updateHigher8() noexcept {
        // only read the upper 8 bits
#if defined(ARDUINO_AVR_ATmega1284)
        constexpr auto Upper16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Upper16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIOA);
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        SPDR = Upper16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto highest = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        address_.bytes[2] = highest;
#else
        address_.bytes[2] = read8<IOExpanderAddress::Upper16Lines, MCP23x17Registers::GPIOA>();
#endif
    }
    template<byte offsetMask>
    static void updateLowest8() noexcept {
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
#if defined(ARDUINO_AVR_ATmega1284)
        constexpr auto Lower16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIOA);
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Lower16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto lowest = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        // inside of here we have access to 12 cycles to play with, so let's actually do some operations while we wait
        // put scope ticks to force the matter
        cacheOffsetEntry_ = (lowest >> 1) & offsetMask; // we want to make this quick to increment
        address_.bytes[0] = lowest;
#else
        address_.bytes[0] = read8<IOExpanderAddress::Lower16Lines, MCP23x17Registers::GPIOA>();
        cacheOffsetEntry_ = (address_.bytes[0] >> 1) & offsetMask; // we want to make this quick to increment
#endif
    }
    static void updateLower8() noexcept {
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
#if defined(ARDUINO_AVR_ATmega1284)
        static constexpr auto Lower16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        static constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIOB);
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Lower16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto lower = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        address_.bytes[1] = lower;
#else
        address_.bytes[1] = read8<IOExpanderAddress::Lower16Lines, MCP23x17Registers::GPIOB>();
#endif
    }
private:
    template<bool inDebugMode>
    inline static void updateTargetFunctions() noexcept {
        if constexpr (auto a = getBody<inDebugMode>(address_.bytes[3]); inDebugMode) {
            lastDebug_ = a;
        } else {
            last_ = a;
        }
    }
public:
    template<bool inDebugMode, byte offsetMask, bool useInterrupts = true, int debugLevel = 0>
    static void newDataCycle() noexcept {
        switch (getUpdateKind<useInterrupts>()) {
            case 0b0001:
                updateLower8();
                upper16Update();
                updateTargetFunctions<inDebugMode>();
                break;
            case 0b0010:
                updateLowest8<offsetMask>();
                upper16Update();
                updateTargetFunctions<inDebugMode>();
                break;
            case 0b0011:
                upper16Update();
                updateTargetFunctions<inDebugMode>();
                break;
            case 0b0100:
                lower16Update<offsetMask>();
                updateHighest8();
                updateTargetFunctions<inDebugMode>();
                break;
            case 0b0101:
                updateLower8();
                updateHighest8();
                updateTargetFunctions<inDebugMode>();
                break;
            case 0b0110:
                updateLowest8<offsetMask>();
                updateHighest8();
                updateTargetFunctions<inDebugMode>();
                break;
            case 0b0111:
                updateHighest8();
                updateTargetFunctions<inDebugMode>();
                break;
            case 0b1000:
                lower16Update<offsetMask>();
                updateHigher8();
                break;
            case 0b1001:
                updateHigher8();
                updateLower8();
                break;
            case 0b1010:
                updateHigher8();
                updateLowest8<offsetMask>();
                break;
            case 0b1011:
                updateHigher8();
                break;
            case 0b1100:
                lower16Update<offsetMask>();
                break;
            case 0b1101:
                updateLower8();
                break;
            case 0b1110:
                updateLowest8<offsetMask>();
                break;
            case 0b1111: break;
            default:
                full32BitUpdate<offsetMask>();
                updateTargetFunctions<inDebugMode>();
                if constexpr (inDebugMode && (debugLevel > 0)) {
                    Serial.print(F("ADDRESS0: 0x"));
                    Serial.println(address_.getWholeValue(), HEX);
                    // put in a delay so we can test the results
                    delayMicroseconds(2);
                    full32BitUpdate<offsetMask>();
                    updateTargetFunctions<inDebugMode>();
                    Serial.print(F("ADDRESS1: 0x"));
                    Serial.println(address_.getWholeValue(), HEX);
                }
                break;
        }
        if constexpr (inDebugMode) {
            lastDebug_();
        } else {
            last_();
        }
    }
    template<bool advanceAddress = true>
    static void burstNext() noexcept {
        if constexpr (advanceAddress) {
            if constexpr (TargetBoard::onAtmega1284p()) {
                // this is a subset of actions, we just need to read the byte enable bits continuously and advance the address by two to get to the
                // next 16-bit word
                // don't increment everything just the lowest byte since we will never actually span 16 byte segments in a single burst transaction
                address_.bytes[0] += 2;
            } else {
                // If we are not a 1284p then just increment the whole value, this is just going to be faster on arm in general
                address_.wholeValue_ += 2;
            }
        }
        if constexpr (TargetBoard::onType3()) {
            // force a delay of 2 in between transactions to make sure we don't go too fast
            // a better mechanism will be necessary in the future
            delayMicroseconds(2);
        }
    }
    /**
     * @brief Return the least significant byte of the address, useful for CoreChipsetFeatures
     * @return The LSB of the address
     */
    [[nodiscard]] static auto getPageOffset() noexcept { return address_.bytes[0]; }
    [[nodiscard]] static auto getPageIndex() noexcept { return address_.bytes[1]; }
private:
    static inline SplitWord32 address_{0};
    static inline SplitWord16 latchedDataOutput {0};
    static inline byte dataLinesDirection_ = 0xFF;
    static inline byte cacheOffsetEntry_ = 0;
    static inline bool initialized_ = false;
    static inline BodyFunction last_ = nullptr;
    static inline BodyFunction lastDebug_ = nullptr;
};
// 8 IOExpanders to a single enable line for SPI purposes
// 4 of them are reserved

#endif //ARDUINO_IOEXPANDERS_H
