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
//
// Created by jwscoggins on 6/22/22.
//

#ifndef SXCHIPSETGCM4TYPE2_SINGLEPAGEMEMORYSPACE_H
#define SXCHIPSETGCM4TYPE2_SINGLEPAGEMEMORYSPACE_H
#include <experimental/memory>
#include "MemorySpace.h"
/**
 * @brief A simple memory space that takes up a single 256 byte page, this is basically a simple 64 entry lookup table
 */
class SinglePageMemorySpace : public SizedMemorySpace {
public:
    using Self = SinglePageMemorySpace;
    using Parent = SizedMemorySpace;
public:
    SinglePageMemorySpace() : Parent(1) { }
    ~SinglePageMemorySpace() override = default;
    constexpr auto& operator[](uint8_t index) noexcept { return getEntry(index); }
    constexpr const auto& operator[](uint8_t index) const noexcept { return getEntry(index); }
    constexpr auto& operator[](uint32_t index) noexcept { return getEntry(index); }
    constexpr const auto& operator[](uint32_t index) const noexcept { return getEntry(index); }
private:
    constexpr SplitWord16& getEntry(uint8_t index) noexcept { return entries_[index & 0b0111'1111]; }
    constexpr const SplitWord16& getEntry(uint8_t index) const noexcept { return entries_[index & 0b0111'1111]; }
    constexpr SplitWord16& getEntry(uint32_t index) noexcept { return getEntry(static_cast<uint8_t>(index >> 1)); }
    constexpr const SplitWord16& getEntry(uint32_t index) const noexcept { return getEntry(static_cast<uint8_t>(index >> 1)); }
public:
    void
    write(uint32_t address, SplitWord16 value, LoadStoreStyle lss) noexcept override {
        switch (auto& result = getEntry(address); lss) {
            case LoadStoreStyle::Full16:
                result = value;
                break;
            case LoadStoreStyle::Upper8:
                result.bytes[1] = value.bytes[1];
                break;
            case LoadStoreStyle::Lower8:
                result.bytes[0] = value.bytes[0];
                break;
            default:
                break;
        }
    }
    [[nodiscard]]
    uint16_t
    read(uint32_t address, LoadStoreStyle lss) const noexcept override {
        return getEntry(address).getWholeValue();
    }
private:
    SplitWord16 entries_[128];
};

template<auto ... Types>
class Foo {

};
#endif //SXCHIPSETGCM4TYPE2_SINGLEPAGEMEMORYSPACE_H
