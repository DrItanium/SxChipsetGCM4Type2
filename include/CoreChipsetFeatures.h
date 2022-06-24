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
        if (!full()) {
            auto& configDevice = getConfigurationDevice(numberOfItems_);
            ++numberOfItems_;
            configDevice.baseAddress_ = ptr->getBaseAddress();
            configDevice.flags_ = 0x8000'0000 | flags;
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
    [[nodiscard]] uint16_t read(uint32_t address, LoadStoreStyle lss) const noexcept override {
        auto [targetPage, targetOffset] = computeTarget(address);
        return readGeneric(targetPage, targetOffset, lss);
    }
private:
    [[nodiscard]] uint16_t readGeneric(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) const noexcept {
        return entries_[targetPage & 0x1F][(offset >> 3) & 0x1F].read(offset & 0b111);
    }
private:
    struct ConfigurationEntry {
        uint32_t baseAddress_ = 0;
        uint32_t flags_ = 0;
        explicit ConfigurationEntry(uint32_t baseAddress, uint32_t flags = 0) noexcept : baseAddress_(baseAddress), flags_(flags) {}
        ConfigurationEntry() : ConfigurationEntry(0, 0) { }
        void clear() noexcept {
            flags_ = 0;
            baseAddress_ = 0;
        }
        [[nodiscard]] uint16_t read(uint8_t offset) const noexcept {
            switch (offset & 0b00000'111) {
                case 0: return baseAddress_ ;
                case 2: return static_cast<uint16_t>(baseAddress_ >> 16) ;
                case 4: return flags_;
                case 6: return static_cast<uint16_t>(flags_ >> 16);
                default:
                    return 0;
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
