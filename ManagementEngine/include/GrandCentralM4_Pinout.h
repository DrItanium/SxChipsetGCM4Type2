/*
i960SxChipset
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

#ifndef SXCHIPSET_GRAND_CENTRAL_M4_PINOUT_H
#define SXCHIPSET_GRAND_CENTRAL_M4_PINOUT_H
#if defined(CHIPSET_TYPE3)
#include "Pinout.h"

class InterruptDisabler final {
public:
    InterruptDisabler() noexcept { noInterrupts(); }
    ~InterruptDisabler() noexcept { interrupts(); }
};

template<i960Pinout pin>
[[gnu::always_inline]]
inline const PinDescription& getPinDescription() noexcept {
    return g_APinDescription[static_cast<int>(pin)];
}
template<i960Pinout pin>
[[gnu::always_inline]]
inline PortGroup & getPortGroup() noexcept {
    return PORT->Group[getPinDescription<pin>().ulPort];
}
static constexpr uint32_t bitMasks[32] {
#define X(index) (1ul << (index))
#define Y(baseIndex) X(baseIndex + 0), X(baseIndex + 1), X(baseIndex + 2), X(baseIndex + 3), X(baseIndex + 4), X(baseIndex + 5), X(baseIndex + 6), X(baseIndex + 7)
        Y(0),
        Y(8),
        Y(16),
        Y(24),
#undef Y
#undef X
};
template<i960Pinout pin>
[[gnu::always_inline]]
inline auto digitalRead() noexcept {
    static_assert(isValidPin960_v<pin>, "Invalid pin provided for digitalRead");
    return (getPortGroup<pin>().IN.reg & (bitMasks[getPinDescription<pin>().ulPin])) != 0 ? HIGH : LOW;
}

template<i960Pinout pin, decltype(HIGH) value>
[[gnu::always_inline]]
inline void digitalWrite() noexcept {
    static_assert(isValidPin960_v<pin>, "Invalid pin provided for digitalWrite");
    // assume that we know what we are doing! so don't support setting the pull-up on input pins
    if constexpr (value == LOW) {
        getPortGroup<pin>().OUTCLR.reg = bitMasks[getPinDescription<pin>().ulPin];
    } else {
        getPortGroup<pin>().OUTSET.reg = bitMasks[getPinDescription<pin>().ulPin];
    }
}
template<i960Pinout pin>
[[gnu::always_inline]]
inline void digitalWrite(decltype(HIGH) value) noexcept {
    static_assert(isValidPin960_v<pin>, "Invalid pin provided for digitalWrite");
    // assume that we know what we are doing! so don't support setting the pull-up on input pins
    if (value == LOW) {
        getPortGroup<pin>().OUTCLR.reg = bitMasks[getPinDescription<pin>().ulPin];
    } else {
        getPortGroup<pin>().OUTSET.reg = bitMasks[getPinDescription<pin>().ulPin];
    }
}
#endif
#endif //SXCHIPSET_GRAND_CENTRAL_M4_PINOUT_H
