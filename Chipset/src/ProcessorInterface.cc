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
            uint32_t unused0 : 12;
            uint32_t muxIndex : 3;
            uint32_t enable : 1;
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
    MuxLines config (DigitalPin<i960Pinout::MUXSel0>::readOutPort());
    config.muxIndex = pattern;
    DigitalPin<i960Pinout::MUXSel0>::writeOutPort(config.value);
    // introduce a forced delay of a few cycles to make sure we switch at the appropriate speed, otherwise we read way too quickly after the update
    asm volatile ("nop");
}
volatile uint8_t
readMuxPort(uint8_t pattern) noexcept {
    setMuxChannel(pattern);
    return DigitalPin<i960Pinout::MUXADR0>::readInPort() >> 16;
}
void
ProcessorInterface::newAddress() noexcept {
    Serial.println(F("NEW ADDRESS!!!"));
    volatile SplitWord32 theAddress(0);

    volatile MuxLines linesDir(PORT->Group[PORTA].DIR.reg);
    linesDir.data = 0;
    PORT->Group[PORTA].DIR.reg = linesDir.value;
    for (int i = 0; i < 8; ++i) {
        Serial.print(F("PATTERN: 0b"));
        Serial.println(i, HEX);
        {
            volatile MuxLines lines(PORT->Group[PORTA].OUT.reg);
            lines.muxIndex = i;
            PORT->Group[PORTA].OUT.reg = lines.value;
            for (volatile int i = 0; i < 32; ++i) {
                asm volatile ("nop");
            }
        }
        for (int j = 0; j < 16; ++j) {
            volatile uint8_t value = 0;
            value |= digitalRead(i960Pinout::MUXADR0) != LOW ? 0b00000001 : 0;
            value |= digitalRead(i960Pinout::MUXADR1) != LOW ? 0b00000010 : 0;
            value |= digitalRead(i960Pinout::MUXADR2) != LOW ? 0b00000100 : 0;
            value |= digitalRead(i960Pinout::MUXADR3) != LOW ? 0b00001000 : 0;
            value |= digitalRead(i960Pinout::MUXADR4) != LOW ? 0b00010000 : 0;
            value |= digitalRead(i960Pinout::MUXADR5) != LOW ? 0b00100000 : 0;
            value |= digitalRead(i960Pinout::MUXADR6) != LOW ? 0b01000000 : 0;
            value |= digitalRead(i960Pinout::MUXADR7) != LOW ? 0b10000000 : 0;
            if (i < 4) {
                theAddress.bytes[i] = value;
            }
            Serial.print(F("\t")); Serial.print(j, HEX); Serial.print(F(": 0b")); Serial.println(value, BIN);
        }
    }
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
