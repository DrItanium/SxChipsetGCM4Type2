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
// Created by jwscoggins on 6/21/21.
//

#ifndef I960SXCHIPSET_CORECHIPSETFEATURES_H
#define I960SXCHIPSET_CORECHIPSETFEATURES_H
#include "ProcessorSerializer.h"
#include "SDCardInterface.h"
#include "Serial0Interface.h"
#include "MemorySpace.h"
class CoreChipsetFeatures : public SizedMemorySpace {
public:
    static constexpr Address IOBaseAddress = 0xFFFF'0000;
    using Parent = SizedMemorySpace;
    using Self = CoreChipsetFeatures;
    using Ptr = std::shared_ptr<Self>;
public:
    CoreChipsetFeatures() : Parent(32) { }
    ~CoreChipsetFeatures() override = default;
    constexpr auto numPopulated() const noexcept { return numberOfItems_;  }
    constexpr auto maxSize() const noexcept { return 1024; }
    constexpr auto empty() const noexcept { return numberOfItems_ == 0; }
    constexpr auto full() const noexcept { return numberOfItems_ == maxSize(); }
public:
    const auto& getConfigurationDevice(uint16_t deviceId) const noexcept { return entries_[(deviceId >> 5) & 0b11111][deviceId & 0b11111]; }
    auto& getConfigurationDevice(uint16_t deviceId) noexcept { return entries_[(deviceId >> 5) & 0b11111][deviceId & 0b11111]; }
    bool
    addDevice(const MemorySpace::Ptr& ptr, uint32_t flags = 0) noexcept {
        return addDevice(ptr->getBaseAddress(), 0x8000'0000 | flags);
    }
    bool
    addDevice(uint32_t address, uint32_t flags) noexcept {
        if (!full()) {
            auto& configDevice = getConfigurationDevice(numberOfItems_);
            ++numberOfItems_;
            configDevice.baseAddress_.setWholeValue(address);
            configDevice.flags_.setWholeValue(flags);
            return true;
        }
        return false;
    }

private:
    static constexpr auto computeTargetPage(uint32_t address) noexcept {
        return static_cast<uint8_t>((address >> 8) & 0x1F);
    }
    static constexpr auto computeTargetOffset(uint32_t address) noexcept {
        return static_cast<uint8_t>(address);
    }
    static constexpr auto makeRelativeAddress(uint32_t address) noexcept {
        return address & 0x1FFF;
    }
    static constexpr auto computeNumberOfBytesToWalk(uint32_t baseAddress, uint32_t count) noexcept {
        if (auto numAvailableBytes = EndRelativeAddress - makeRelativeAddress(baseAddress); numAvailableBytes < count) {
            return numAvailableBytes;
        } else {
            return count;
        }
    }
    static constexpr std::tuple<uint8_t, uint8_t> computeTarget(uint32_t address) noexcept {
        return std::make_tuple<uint8_t, uint8_t>(computeTargetPage(address), computeTargetOffset(address));
    }
public:
    [[nodiscard]] uint16_t read16(uint32_t address) const noexcept override {
        auto [targetPage, targetOffset] = computeTarget(address);
        return entries_[targetPage & 0x1f][(targetOffset >> 3) & 0x1F].read16(targetOffset & 0b111);
    }
private:
    struct ConfigurationEntry {
        SplitWord32 baseAddress_ {0};
        SplitWord32 flags_ {0};
        explicit ConfigurationEntry(uint32_t baseAddress, uint32_t flags = 0) noexcept : baseAddress_(baseAddress), flags_(flags) {}
        ConfigurationEntry() : ConfigurationEntry(0, 0) { }
        void clear() noexcept {
            baseAddress_.setWholeValue(0);
            flags_.setWholeValue(0);
        }
        [[nodiscard]] constexpr uint8_t read8(uint8_t offset) const noexcept {
            switch (offset & 0b111) {
                case 0: return baseAddress_.getLowestByte();
                case 1: return baseAddress_.getLowerByte();
                case 2: return baseAddress_.getHigherByte();
                case 3: return baseAddress_.getHighestByte();
                case 4: return flags_.getLowestByte();
                case 5: return flags_.getLowerByte();
                case 6: return flags_.getHigherByte();
                case 7: return flags_.getHighestByte();
                default: return 0;
            }
        }
        [[nodiscard]] constexpr uint16_t read16(uint8_t offset) const noexcept {
            switch ((offset & 0b111) >> 1) {
                case 0: return baseAddress_.getLowerHalf();
                case 1: return baseAddress_.getUpperHalf();
                case 2: return flags_.getLowerHalf();
                case 3: return flags_.getUpperHalf();
                default: return 0;
            }
        }
        [[nodiscard]] constexpr uint32_t read32(uint8_t offset) const noexcept {
            switch ((offset & 0b111) >> 2) {
                case 0: return baseAddress_.getWholeValue();
                case 1: return flags_.getWholeValue();
                default: return 0;
            }
        }
    };
    static inline constexpr uint32_t EndRelativeAddress = 256*32;
    /**
     * @brief The 32 pages with 32 entries each
     */
    ConfigurationEntry entries_[32][32];
    static_assert(sizeof(entries_) == 256*32, "Entry block must be 256 bytes * 32 pages in size!");
    uint16_t numberOfItems_ = 0;
};
#endif //I960SXCHIPSET_CORECHIPSETFEATURES_H
