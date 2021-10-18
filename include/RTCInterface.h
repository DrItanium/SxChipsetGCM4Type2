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

#ifndef SXCHIPSET_RTCINTERFACE_H
#define SXCHIPSET_RTCINTERFACE_H
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SI5351.h>
#include <RTClib.h>

#include "Pinout.h"

template<Address baseAddress>
class RTCInterface {
public:
public:
    RTCInterface() = delete;
    ~RTCInterface() = delete;
    static constexpr bool respondsTo(byte targetPage) noexcept {
        return targetPage >= StartPage && targetPage < EndPage;
    }
    static void begin() noexcept {
        rtcUp = rtc.begin();
        if (!rtcUp) {
            Serial.println(F("NO RTC FOUND...DISABLING"));
        } else {
            Serial.println(F("RTC FOUND... CHECKING"));
            if (!rtc.initialized() || rtc.lostPower()) {
                Serial.println(F("RTC is NOT initialized, setting time from sketch compile"));
                // not the most accurate but good enough
                rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
            }
            DateTime now = rtc.now();
            Serial.print(now.year(), DEC);
            Serial.print(F("/"));
            Serial.print(now.month(), DEC);
            Serial.print(F("/"));
            Serial.print(now.day(), DEC);
            Serial.print(F(" ("));
            Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
            Serial.print(F(") "));
            Serial.print(now.hour(), DEC);
            Serial.print(F(":"));
            Serial.print(now.minute(), DEC);
            Serial.print(F(":"));
            Serial.print(now.second(), DEC);
            Serial.println();

            rtc.start();
        }
    }
    static uint16_t read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept {
        return 0;
    }
    static void write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
    }
private:
    static inline RTC_PCF8523 rtc_;
    static inline bool rtcUp_ = false;
    static inline char daysOfTheWeek_[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
};

#endif //SXCHIPSET_RTCINTERFACE_H
