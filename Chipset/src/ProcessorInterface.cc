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
}
template<uint8_t pattern>
uint8_t
readMuxPort() noexcept {
    digitalWrite<i960Pinout::MUX_EN, LOW>();
    digitalWrite<i960Pinout::MUXSel0, pattern & 0b001 ? HIGH : LOW>();
    digitalWrite<i960Pinout::MUXSel1, pattern & 0b010 ? HIGH : LOW>();
    digitalWrite<i960Pinout::MUXSel2, pattern & 0b100 ? HIGH : LOW>();
    volatile auto result = DigitalPin<i960Pinout::MUXADR0>::readInPort();
    digitalWrite<i960Pinout::MUX_EN, HIGH>();
    return static_cast<uint8_t>(result >> 16);
}
namespace {
    void
    displayPortContents() noexcept {
        digitalWrite<i960Pinout::MUX_EN, LOW>();
        Serial.print(F("\tIN: 0x"));
        Serial.print(DigitalPin<i960Pinout::MUXSel0>::readInPort(), HEX);
        Serial.print(F(", OUT: 0x"));
        Serial.println(DigitalPin<i960Pinout::MUXSel0>::readOutPort(), HEX);
        digitalWrite<i960Pinout::MUX_EN, HIGH>();
    }
}
void
ProcessorInterface::newAddress() noexcept {
    displayPortContents();
    // update the address here
    auto lowest = readMuxPort<0>();
    Serial.print(F("ind 0: 0x")); Serial.println(lowest, HEX);
    displayPortContents();
    isWriteOperation_ = lowest & 0b1;
    lowest &= 0b1111'1110; // clear the W/R bit out
    auto lower = readMuxPort<1>();
    Serial.print(F("ind 1: 0x")); Serial.println(lower, HEX);
    displayPortContents();
    auto higher = readMuxPort<2>();
    Serial.print(F("ind 2: 0x")); Serial.println(higher, HEX);
    displayPortContents();
    auto highest = readMuxPort<3>();
    Serial.print(F("ind 3: 0x")); Serial.println(highest, HEX);
    displayPortContents();
    address_ = SplitWord32{lowest, lower, higher, highest};
    Serial.print(F("ind 4: 0x")); Serial.println(readMuxPort<4>(), HEX);
    displayPortContents();
    Serial.print(F("ind 5: 0x")); Serial.println(readMuxPort<5>(), HEX);
    displayPortContents();
    Serial.print(F("ind 6: 0x")); Serial.println(readMuxPort<6>(), HEX);
    displayPortContents();
    Serial.print(F("ind 7: 0x")); Serial.println(readMuxPort<7>(), HEX);
    displayPortContents();
    Serial.print(F("ADDRESS: 0x"));
    Serial.println(address_.getWholeValue(), HEX);
}
LoadStoreStyle
ProcessorInterface::getStyle() noexcept {
    return static_cast<LoadStoreStyle>((readMuxPort<4>() >> 4) & 0b11);
}
