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
class CoreChipsetFeatures {
public:
    static constexpr Address IOBaseAddress = 0xFFFF'0000;
public:
    CoreChipsetFeatures() = delete;
    ~CoreChipsetFeatures() = delete;
    CoreChipsetFeatures(const CoreChipsetFeatures&) = delete;
    CoreChipsetFeatures(CoreChipsetFeatures&&) = delete;
    CoreChipsetFeatures& operator=(const CoreChipsetFeatures&) = delete;
    CoreChipsetFeatures& operator=(CoreChipsetFeatures&&) = delete;
    static void setAddress(uint8_t page, uint8_t offset, uint32_t address, uint32_t flags) {
        auto targetPage = page & 0b11111;
        auto targetOffset = offset & 0b11111;
        activeConfigurationPages_ |= (1 << targetPage);
        enabledDevices_[targetPage] |= (1 << targetOffset);
        entries_[targetPage][targetOffset] = ConfigurationEntry{address, flags};
    }
    static void begin() {
        // clear out the table to start
        activeConfigurationPages_ = 0;
        for (int i = 0;i < 32; ++i) {
            enabledDevices_[i] = 0;
            for (int j = 0; j < 32; ++j) {
                entries_[i][j].clear();
            }
        }
    }
private:
    static uint16_t readFromFirstPage(uint8_t offset, LoadStoreStyle lss) noexcept {
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
    [[nodiscard]] static uint16_t read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept {
        switch (targetPage) {
            case 0:
                return readFromFirstPage(offset, lss);
            default:
                return 0;
        }
    }
    static void write(uint8_t, uint8_t, LoadStoreStyle, SplitWord16) noexcept { }
private:
    struct ConfigurationEntry {
        uint32_t baseAddress_ = 0;
        uint32_t flags_ = 0;
        explicit ConfigurationEntry(uint32_t baseAddress, uint32_t flags = 0) noexcept : baseAddress_(baseAddress), flags_(flags) {}
        void clear() noexcept {
            flags_ = 0;
            baseAddress_ = 0;
        }
    };
    /**
     * @brief Describes which of the 32 pages contain active entries, a 1 means there are available entries in there
     */
    static inline uint32_t activeConfigurationPages_;
    /**
     * @brief A series of 32-bit values which describe which entries in a given table are available for querying. A 1 means that the given item is available
     */
    static inline uint32_t enabledDevices_[32];
    /**
     * @brief The 32 pages with 32 entries each
     */
    static ConfigurationEntry entries_[32][32];
};
#endif //I960SXCHIPSET_CORECHIPSETFEATURES_H
