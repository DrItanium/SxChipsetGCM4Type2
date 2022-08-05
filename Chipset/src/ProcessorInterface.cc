/*
SxChipset_ManagementEngine
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
#include "ProcessorInterface.h"
namespace
{
    union DataLines
    {
        uint32_t value;
        struct
        {
            uint32_t lowerPart: 8;
            uint32_t real89: 2;
            uint32_t upperPartRest: 6;
            uint32_t layout89: 2;
            uint32_t rest: 14;
        };
        uint16_t halves[2];
        [[nodiscard]] constexpr auto getValue32() const noexcept { return value; }
        [[nodiscard]] constexpr auto getLowerHalf() const noexcept { return halves[0]; }
        [[nodiscard]] constexpr auto getUpperHalf() const noexcept { return halves[1]; }
        explicit DataLines(uint32_t value) : value(value) {}
        explicit DataLines(uint16_t lower, uint16_t upper = 0) : halves{lower, upper} { }
    };
}
void
ProcessorInterface::setupDataLinesForWrite() noexcept {
    // writes are inputs since the processor is writing to the chipset
    DataLines directionContents( DigitalPin<i960Pinout::Data0>::readPortDir());
    directionContents.layout89 = 0;
    directionContents.lowerPart = 0;
    directionContents.upperPartRest = 0;
    DigitalPin<i960Pinout::Data0>::writePortDir(directionContents.getValue32());
}
void
ProcessorInterface::setupDataLinesForRead() noexcept {
    // reads are outputs since the processor is reading from the chipset
    DataLines directionContents( DigitalPin<i960Pinout::Data0>::readPortDir());
    directionContents.layout89 = 0b11;
    directionContents.lowerPart = 0xFF;
    directionContents.upperPartRest = 0b111'111;
    DigitalPin<i960Pinout::Data0>::writePortDir(directionContents.getValue32());
}
bool
ProcessorInterface::isReadOperation() noexcept {
    return !isWriteOperation_;
}
SplitWord16
ProcessorInterface::getDataBits() noexcept {
    DataLines tmp(DigitalPin<i960Pinout::Data0>::readInPort());
    tmp.real89 = tmp.layout89;
    Serial.print(F("\t GET DATA BITS: 0x"));
    Serial.println(tmp.getLowerHalf(), HEX);
    return SplitWord16{tmp.getLowerHalf()};
}
void
ProcessorInterface::setDataBits(uint16_t value) noexcept {
    Serial.print(F("\tSET DATA BITS: 0x"));
    Serial.println(value, HEX);
    volatile DataLines lineRepresentation(value, 0);
    DataLines outPort(DigitalPin<i960Pinout::Data0>::readOutPort());
    outPort.lowerPart = lineRepresentation.lowerPart;
    outPort.layout89 = lineRepresentation.real89;
    outPort.upperPartRest = lineRepresentation.upperPartRest;
    volatile uint32_t result = outPort.getValue32();
    DigitalPin<i960Pinout::Data0>::writeOutPort(result);
};
void
ProcessorInterface::begin() noexcept {
    //PORT->Group[PORTA].CTRL.reg = 0x00FF0000;
}
void
setMuxChannel(uint8_t pattern) noexcept {
    /// @todo use the OUTSET register to improve performance
    digitalWrite<i960Pinout::MA0>(pattern & 0b1 ? HIGH : LOW);
    digitalWrite<i960Pinout::MA1>(pattern & 0b10 ? HIGH : LOW);
}
volatile uint8_t
readMuxPort(uint8_t pattern) noexcept {
    setMuxChannel(pattern);
    return DigitalPin<i960Pinout::L0>::readInPort() >> 16;
}
struct TranslationTableEntry {
    constexpr TranslationTableEntry(uint8_t value) noexcept :
            result_(
                    ((value & 0b0000'0001) ? 0x00000001 : 0) |
                    ((value & 0b0000'0010) ? 0x00000010 : 0) |
                    ((value & 0b0000'0100) ? 0x00000100 : 0) |
                    ((value & 0b0000'1000) ? 0x00001000 : 0) |
                    ((value & 0b0001'0000) ? 0x00010000 : 0) |
                    ((value & 0b0010'0000) ? 0x00100000 : 0) |
                    ((value & 0b0100'0000) ? 0x01000000 : 0) |
                    ((value & 0b1000'0000) ? 0x10000000 : 0)) { }
    [[nodiscard]] constexpr auto getResult() const noexcept {return result_; }
    uint32_t result_;
};
static_assert(TranslationTableEntry{0xFF}.getResult() == 0x1111'1111);
static_assert(TranslationTableEntry{0xFF}.getResult() << 1 == 0x2222'2222);
static_assert(TranslationTableEntry{0xFF}.getResult() << 2 == 0x4444'4444);
static_assert(TranslationTableEntry{0xFF}.getResult() << 3 == 0x8888'8888);
constexpr uint32_t BaseTranslationTable[256] {
#define X(index) TranslationTableEntry(index).getResult()
#define Y(base) X((8 * base) + 0), X((8 * base) + 1), X( 8* base + 2), X(8*base + 3), X(8*base + 4), X(8*base + 5), X(8*base + 6), X(8*base + 7)
        Y(0), Y(1), Y(2), Y(3), Y(4), Y(5), Y(6), Y(7),
        Y(8), Y(9), Y(10), Y(11), Y(12), Y(13), Y(14), Y(15),
        Y(16), Y(17), Y(18), Y(19), Y(20), Y(21), Y(22), Y(23),
        Y(24), Y(25), Y(26), Y(27), Y(28), Y(29), Y(30), Y(31),
#undef Y
#undef X
};
constexpr uint32_t ProperTranslationTable[4][256] {
#define X(index, shift) BaseTranslationTable[index] << shift
#define Y(base, shift) X((8 * base) + 0, shift), X((8 * base) + 1, shift), X( 8* base + 2, shift), X(8*base + 3, shift), X(8*base + 4, shift), X(8*base + 5, shift), X(8*base + 6, shift), X(8*base + 7, shift)
        {
                Y(0, 0), Y(1, 0), Y(2, 0), Y(3, 0), Y(4, 0), Y(5, 0), Y(6, 0), Y(7, 0),
                Y(8, 0), Y(9, 0), Y(10, 0), Y(11, 0), Y(12, 0), Y(13, 0), Y(14, 0), Y(15, 0),
                Y(16, 0), Y(17, 0), Y(18, 0), Y(19, 0), Y(20, 0), Y(21, 0), Y(22, 0), Y(23, 0),
                Y(24, 0), Y(25, 0), Y(26, 0), Y(27, 0), Y(28, 0), Y(29, 0), Y(30, 0), Y(31, 0),
        },
        {
                Y(0, 1), Y(1, 1), Y(2, 1), Y(3, 1), Y(4, 1), Y(5, 1), Y(6, 1), Y(7, 1),
                Y(8, 1), Y(9, 1), Y(10, 1), Y(11, 1), Y(12, 1), Y(13, 1), Y(14, 1), Y(15, 1),
                Y(16, 1), Y(17, 1), Y(18, 1), Y(19, 1), Y(20, 1), Y(21, 1), Y(22, 1), Y(23, 1),
                Y(24, 1), Y(25, 1), Y(26, 1), Y(27, 1), Y(28, 1), Y(29, 1), Y(30, 1), Y(31, 1),
        },
        {
                Y(0, 2), Y(1, 2), Y(2, 2), Y(3, 2), Y(4, 2), Y(5, 2), Y(6, 2), Y(7, 2),
                Y(8, 2), Y(9, 2), Y(10, 2), Y(11, 2), Y(12, 2), Y(13, 2), Y(14, 2), Y(15, 2),
                Y(16, 2), Y(17, 2), Y(18, 2), Y(19, 2), Y(20, 2), Y(21, 2), Y(22, 2), Y(23, 2),
                Y(24, 2), Y(25, 2), Y(26, 2), Y(27, 2), Y(28, 2), Y(29, 2), Y(30, 2), Y(31, 2),
        },
        {
                Y(0, 3), Y(1, 3), Y(2, 3), Y(3, 3), Y(4, 3), Y(5, 3), Y(6, 3), Y(7, 3),
                Y(8, 3), Y(9, 3), Y(10, 3), Y(11, 3), Y(12, 3), Y(13, 3), Y(14, 3), Y(15, 3),
                Y(16, 3), Y(17, 3), Y(18, 3), Y(19, 3), Y(20, 3), Y(21, 3), Y(22, 3), Y(23, 3),
                Y(24, 3), Y(25, 3), Y(26, 3), Y(27, 3), Y(28, 3), Y(29, 3), Y(30, 3), Y(31, 3),
        },
#undef Y
#undef X
};
void
ProcessorInterface::newAddress() noexcept {
    // the mux data lines gives us data in the form of a compacted rectangle with each column being what you get
    // I:  00,  01,  10,  11
    // 0:  WR,  A1,  A2,  A3
    // 1:  A4,  A5,  A6,  A7
    // 2:  A8,  A9, A10, A11
    // 3: A12, A13, A14, A15
    // 4: A16, A17, A18, A19
    // 5: A20, A21, A22, A23,
    // 6: A24, A25, A26, A27,
    // 7: A28, A29, A30, A31
    //
    // So the goal is to expand each byte into a 32-bit number that can be quickly combined
    address_.wholeValue_ = 0;
    for (int i = 0; i < 4; ++i) {
        volatile auto m = readMuxPort(i);
        volatile uint32_t theAddress = ProperTranslationTable[i][m];
        Serial.print(F("\t0b"));
        Serial.print(m, BIN);
        Serial.print(F(" -> A"));
        Serial.print(i, DEC);
        Serial.print(F(": 0b"));
        Serial.println(theAddress, BIN);
        address_.wholeValue_ |= theAddress;
    }
    isWriteOperation_ = (address_.wholeValue_ & 0b1) != 0;
    address_.wholeValue_ &= 0xFFFF'FFFE;
    Serial.print(F("ADDRESS: 0x"));
    Serial.print(address_.wholeValue_ , HEX);
    Serial.print(F(", OP: "));
    Serial.println(isWriteOperation_ ? "WRITE" : "READ");
}
LoadStoreStyle
ProcessorInterface::getStyle() noexcept {
    if (DigitalPin<i960Pinout::BE0>::isAsserted()) {
        if (DigitalPin<i960Pinout::BE1>::isAsserted()) {
            return LoadStoreStyle::Full16;
        } else {
            return LoadStoreStyle::Lower8;
        }
    } else {
        if (DigitalPin<i960Pinout::BE1>::isAsserted()) {
            return LoadStoreStyle::Upper8;
        } else {
            return LoadStoreStyle::None;
        }
    }
}
