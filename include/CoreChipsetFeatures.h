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
class CoreChipsetFeatures : public MemorySpace {
public:
    static constexpr Address IOBaseAddress = 0xFFFF'0000;
    using Parent = MemorySpace;
public:
    CoreChipsetFeatures() : Parent(0xFFFF'0000, 33) { }
    ~CoreChipsetFeatures() override = default;
    void
    setAddress(uint8_t page, uint8_t offset, uint32_t address, uint32_t flags) {
        auto targetPage = page & 0b11111;
        auto targetOffset = offset & 0b11111;
        activeConfigurationPages_ |= (1 << targetPage);
        enabledDevices_[targetPage] |= (1 << targetOffset);
        entries_[targetPage][targetOffset] = ConfigurationEntry{address, flags};
    }

private:
    uint16_t
    readFromFirstPage(uint8_t offset) const noexcept {
        switch (offset) {
            case 0: return static_cast<uint16_t>(activeConfigurationPages_);
            case 2: return static_cast<uint16_t>(activeConfigurationPages_ >> 16);
#define X(ind) case ind: return static_cast<uint16_t>(enabledDevices_[ind - 4]); \
              case ind + 2 : return static_cast<uint16_t>(enabledDevices_[ind - 4] >> 16)
            X(4); X(8); X(12); X(16); X(20); X(24); X(28); X(32);
            X(36); X(40); X(44); X(48); X(52); X(56); X(60); X(64);
            X(68); X(72); X(76); X(80); X(84); X(88); X(92); X(96);
            X(100); X(104); X(108); X(112); X(116); X(120); X(124); X(128);
#undef X
            default:
                return 0;


        }
    }
public:
    [[nodiscard]] uint16_t read(uint32_t address, LoadStoreStyle lss) const noexcept override {
        auto targetPage = static_cast<uint8_t>((address >> 8) & 0x1F);
        auto targetOffset = static_cast<uint8_t>(address);
        return readGeneric(targetPage, targetOffset, lss);
    }
private:
    [[nodiscard]] uint16_t readGeneric(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) const noexcept {
        switch (targetPage) {
            case 0:
                return readFromFirstPage(offset);
            case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8:
            case 9: case 10: case 11: case 12: case 13: case 14: case 15: case 16:
            case 17: case 18: case 19: case 20: case 21: case 22: case 23: case 24:
            case 25: case 26: case 27: case 28: case 29: case 30: case 31: case 32:
                return entries_[targetPage - 1][(offset & 0b11111'000) >> 3].read(offset & 0b111);
            default:
                return 0;
        }
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
    /**
     * @brief Describes which of the 32 pages contain active entries, a 1 means there are available entries in there
     */
    uint32_t activeConfigurationPages_ = 0;
    /**
     * @brief A series of 32-bit values which describe which entries in a given table are available for querying. A 1 means that the given item is available
     */
    uint32_t enabledDevices_[32] { 0 };
    /**
     * @brief The 32 pages with 32 entries each
     */
    ConfigurationEntry entries_[32][32];
};
#endif //I960SXCHIPSET_CORECHIPSETFEATURES_H
