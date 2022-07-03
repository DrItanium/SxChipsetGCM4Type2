/*
SxChipsetGCM4Type2
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
//
// Created by jwscoggins on 7/2/22.
//

#ifndef MANAGEMENTENGINE_PINOUT_H
#define MANAGEMENTENGINE_PINOUT_H
#include <Arduino.h>

enum class PinStyle {
    Input,
    Output,
    InputPullup,
    Any,
};

template<PinStyle style>
constexpr decltype(OUTPUT) PinDirection_v = INPUT; // default to input
template<> constexpr auto PinDirection_v<PinStyle::Input> = INPUT;
template<> constexpr auto PinDirection_v<PinStyle::InputPullup> = INPUT_PULLUP;
template<> constexpr auto PinDirection_v<PinStyle::Output> = OUTPUT;

template<auto pin, PinStyle style, decltype(HIGH) asserted, decltype(HIGH) deasserted>
struct DigitalPin {
    static_assert(asserted != deasserted, "asserted cannot be equal to deasserted");
    DigitalPin() = delete;
    ~DigitalPin() = delete;
    DigitalPin(const DigitalPin&) = delete;
    DigitalPin(DigitalPin&&) = delete;
    DigitalPin& operator=(const DigitalPin&) = delete;
    DigitalPin& operator=(DigitalPin&&) = delete;
    static constexpr bool isBidirectionalPin() noexcept { return style == PinStyle::Any; }
    static constexpr bool isInputPin() noexcept { return style == PinStyle::Input || isBidirectionalPin(); }
    static constexpr bool isOutputPin() noexcept { return style == PinStyle::Output || isBidirectionalPin(); }
    static constexpr bool isInputPullupPin() noexcept { return style == PinStyle::InputPullup || isBidirectionalPin(); }
    static constexpr auto getDirection() noexcept { return PinDirection_v<style>; }
    static constexpr auto getPin() noexcept { return pin; }
    static void configure(decltype(OUTPUT) pinDirection = getDirection()) noexcept {
        ::pinMode(static_cast<int>(getPin()), pinDirection);
    }
    static auto read() noexcept {
        return digitalReadFast(static_cast<int>(pin));
    }
    static bool inputAsserted() noexcept { return read() == asserted; }
    static bool inputDeasserted() noexcept { return read() == deasserted; }
    static bool inputLow() noexcept { return read() == LOW; }
    static bool inputHigh() noexcept { return read() == HIGH; }
    static void write(decltype(HIGH) value) noexcept {
        digitalWriteFast(static_cast<int>(pin), value);
    }
    static void assertPin() noexcept { write(asserted); }
    static void deassertPin() noexcept { write(deasserted); }
    static void pulse() noexcept {
        assertPin();
        __builtin_avr_nops(2);
        deassertPin();
    }
    static void waitUntilPinIsAsserted() noexcept {
        /// @todo should this only be allowed on input pins?
        while (inputDeasserted());
    }
    static void waitUntilPinIsDeasserted() noexcept {
        while (inputAsserted());
    }
};

template<typename T>
[[gnu::always_inline]]
inline void digitalWrite(T ip, decltype(HIGH) value) {
    digitalWriteFast(static_cast<int>(ip), value);
}

template<typename T>
[[gnu::always_inline]]
inline auto digitalRead(T ip) {
    return digitalRead<T>(ip);
}

template<typename ... pins>
[[gnu::always_inline]]
inline void configurePins() noexcept {
    (pins::configure(), ...);
}
template<typename ... pins>
[[gnu::always_inline]]
inline void deassertPins() noexcept {
    (pins::deassertPin(), ...);
}
template<auto pin, decltype(HIGH) value>
[[gnu::always_inline]]
inline void digitalWrite() noexcept {
    digitalWrite(pin, value);
}
template<auto pin>
[[gnu::always_inline]]
inline void digitalWrite(decltype(HIGH) value) noexcept {
    digitalWriteFast(static_cast<int>(pin), value);
}
template<auto pin>
[[gnu::always_inline]] inline void digitalWrite(bool level) noexcept {
    digitalWriteFast(static_cast<int>(pin), level ? HIGH : LOW);
}

template<auto pin, decltype(HIGH) asserted = LOW>
[[gnu::always_inline]]
inline void pulse() noexcept {
    static constexpr auto deasserted = asserted == LOW ? HIGH : LOW;
    // use the switch to value to compute what to revert to
    digitalWrite<pin, asserted>();
    __builtin_avr_nops(4);
    digitalWrite<pin, deasserted>();
}

template<auto pin>
[[gnu::always_inline]]
inline auto digitalRead() noexcept {
    return digitalReadFast(static_cast<int>(pin));
}




template<typename T>
class PinAsserter {
public:
    static_assert(T::isOutputPin());
    PinAsserter() { T::assertPin(); }
    ~PinAsserter() { T::deassertPin(); }
};

template<auto pin, decltype(HIGH) asserted, decltype(HIGH) deasserted>
using OutputPin = DigitalPin<pin, PinStyle::Output, asserted, deasserted>;

template<auto pin, decltype(HIGH) asserted, decltype(HIGH) deasserted>
using InputPin = DigitalPin<pin, PinStyle::Input, asserted, deasserted>;
template<auto pin, decltype(HIGH) asserted, decltype(HIGH) deasserted>
using InputPullupPin = DigitalPin<pin, PinStyle::InputPullup, asserted, deasserted>;

template<auto pin> using ActiveHighOutputPin = OutputPin<pin, HIGH, LOW>;
template<auto pin> using ActiveLowOutputPin = OutputPin<pin, LOW, HIGH>;
template<auto pin> using ActiveHighInputPin = InputPin<pin, HIGH, LOW>;
template<auto pin> using ActiveLowInputPin = InputPin<pin, LOW, HIGH>;
template<auto pin> using ActiveHighInputPullupPin = InputPullupPin<pin, HIGH, LOW>;
template<auto pin> using ActiveLowInputPullupPin = InputPullupPin<pin, LOW, HIGH>;
#endif //MANAGEMENTENGINE_PINOUT_H
