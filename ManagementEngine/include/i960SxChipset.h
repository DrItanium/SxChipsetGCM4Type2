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
// Created by jwscoggins on 10/31/21.
//

#ifndef SXCHIPSET_I960SXCHIPSET_H
#define SXCHIPSET_I960SXCHIPSET_H
#include <string>
#include <tuple>
using BodyFunction = void (*)();
using SplitBodyFunction = std::tuple<BodyFunction, BodyFunction>;
BodyFunction getNonDebugBody(byte index) noexcept;
BodyFunction getDebugBody(byte index) noexcept;

template<bool inDebugMode>
BodyFunction getBody(byte index) noexcept {
    if constexpr (inDebugMode) {
        return getDebugBody(index);
    } else {
        return getNonDebugBody(index);
    }
}
SplitBodyFunction getSplitNonDebugBody(byte index) noexcept;
SplitBodyFunction getSplitDebugBody(byte index) noexcept;

template<bool inDebugMode>
SplitBodyFunction getSplitBody(byte index) noexcept {
    if constexpr (inDebugMode) {
        return getSplitDebugBody(index);
    } else {
        return getSplitNonDebugBody(index);
    }
}

[[noreturn]] void signalHaltState(const __FlashStringHelper* msg) noexcept;
[[noreturn]] void signalHaltState(const char* msg) noexcept;
#ifdef __arm__
[[noreturn]] void signalHaltState(const std::string& msg) noexcept;
#endif

#endif //SXCHIPSET_I960SXCHIPSET_H
