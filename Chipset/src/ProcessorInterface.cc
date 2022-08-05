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
    union MuxLines {
        uint32_t value;
        struct {
            uint32_t unused0 : 16;
            uint32_t data : 8;
            uint32_t unused1 : 8;
        };
        explicit MuxLines(uint32_t value) : value(value) { }

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
    return SplitWord16{tmp.getLowerHalf()};
}
void
ProcessorInterface::setDataBits(uint16_t value) noexcept {
    DataLines lineRepresentation(value, 0);
    DataLines outPort(DigitalPin<i960Pinout::Data0>::readOutPort());
    outPort.lowerPart = lineRepresentation.lowerPart;
    outPort.layout89 = lineRepresentation.real89;
    outPort.upperPartRest = lineRepresentation.upperPartRest;
    DigitalPin<i960Pinout::Data0>::writeOutPort(outPort.getValue32());
};
void
ProcessorInterface::begin() noexcept {
    PORT->Group[PORTA].CTRL.reg = 0x00FF0000;
}
void
setMuxChannel(uint8_t pattern) noexcept {
    /// @todo use the OUTSET register to improve performance
    digitalWrite<i960Pinout::MA0>(pattern & 0b1);
    digitalWrite<i960Pinout::MA1>(pattern & 0b10);
}
volatile uint8_t
readMuxPort(uint8_t pattern) noexcept {
    setMuxChannel(pattern);
    return DigitalPin<i960Pinout::L0>::readInPort() >> 16;
}
struct TranslationTableEntry {
    constexpr TranslationTableEntry(uint8_t value) noexcept :
            result_(
                    ((value & 0b0000'0001) ? (0b0001 << 0) : 0) |
                    ((value & 0b0000'0010) ? (0b0001 << 4) : 0) |
                    ((value & 0b0000'0100) ? (0b0001 << 8) : 0) |
                    ((value & 0b0000'1000) ? (0b0001 << 12) : 0) |
                    ((value & 0b0001'0000) ? (0b0001 << 16) : 0) |
                    ((value & 0b0010'0000) ? (0b0001 << 20) : 0) |
                    ((value & 0b0100'0000) ? (0b0001 << 24) : 0) |
                    ((value & 0b1000'0000) ? (0b0001 << 28) : 0)) { }
    [[nodiscard]] constexpr auto getResult() const noexcept {return result_; }
    uint32_t result_;
};
static_assert(TranslationTableEntry{0xFF}.getResult() == 0x1111'1111);
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
    static constexpr uint32_t TranslationTable[256] {

    };
    Serial.println(F("NEW ADDRESS!!!"));
    volatile SplitWord32 theAddress(0);
    theAddress.bytes[0] = readMuxPort(0b00);
    theAddress.bytes[1] = readMuxPort(0b01);
    theAddress.bytes[2] = readMuxPort(0b10);
    theAddress.bytes[3] = readMuxPort(0b11);
    isWriteOperation_ = theAddress.bytes[0] & 0b1;
    theAddress.bytes[0] &= 0b1111'1110;
    Serial.print(F("ADDRESS: 0x"));
    Serial.println(theAddress.wholeValue_, HEX);
    address_.wholeValue_ = theAddress.wholeValue_;
    volatile MuxLines linesDir2(PORT->Group[PORTA].DIR.reg);
    linesDir2.data = 0xFF;
    PORT->Group[PORTA].DIR.reg = linesDir2.value;
}
LoadStoreStyle
ProcessorInterface::getStyle() noexcept {
    return static_cast<LoadStoreStyle>((readMuxPort(4) >> 4) & 0b11);
}
