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
//
// Created by jwscoggins on 6/21/22.
//

#ifndef SXCHIPSETGCM4TYPE2_RAM_H
#define SXCHIPSETGCM4TYPE2_RAM_H
#include "MCUPlatform.h"
#include "MemorySpace.h"
#include "SetAssociativeRandPLRUCacheSets.h"
#include "SinglePoolCache.h"
#include "Pinout.h"
template<typename S>
class RAM : public MemorySpace {
public:
    using Self = RAM;
    using Parent = MemorySpace;
    using BackingMemoryStorage_t = S;
    static constexpr auto CacheLineSize = TargetBoard::getCacheLineSizeInBits();
    static constexpr auto CompileInAddressDebuggingSupport = TargetBoard::compileInAddressDebuggingSupport();
    static constexpr auto AddressDebuggingEnabledOnStartup = TargetBoard::addressDebuggingEnabledOnStartup();
    static constexpr auto CompileInCacheSystemDebuggingSupport = TargetBoard::compileInCacheSystemDebuggingSupport();
    static constexpr auto CompileInExtendedDebugInformation = TargetBoard::compileInExtendedDebugInformation();
    static constexpr auto ValidateTransferDuringInstall = TargetBoard::validateTransferDuringInstall();
    using Cache_t = Cache2Instance_t<TenWayRandPLRUCacheWay, 256, CacheLineSize, BackingMemoryStorage_t, CompileInAddressDebuggingSupport>;
    static constexpr auto BaseAddress = 0;
    static constexpr auto NumPages = (512 * 1024 * 1024) / 256;
    using TaggedAddress = typename Cache_t::TaggedAddress;
public:
    RAM() : Parent(BaseAddress, NumPages) {
        theCache_.begin();
    }
    ~RAM() override = default;
    void write(uint32_t address, SplitWord16 value, LoadStoreStyle lss) noexcept override {
        auto& line = theCache_.getLine(TaggedAddress{address});
        line.set(value, lss);
    }
    [[nodiscard]]
    uint16_t
    read(uint32_t address, LoadStoreStyle lss) const noexcept override {
        auto& line = theCache_.getLine(TaggedAddress{address});
        return theCache_.getLine(TaggedAddress{address}).get(ProcessorInterface::getCacheOffsetEntry<Cache_t::CacheEntryMask>(SplitWord32{address}));
    }
    uint32_t write(uint32_t baseAddress, uint8_t *data, uint32_t count) noexcept override {
        /// @todo implement
        return 0;
    }
    uint32_t read(uint32_t baseAddress, uint8_t *data, uint32_t count) noexcept override {
        /// @todo implement
        return 0;
    }
    void handleReadRequest() noexcept override {
        MemorySpace::handleReadRequest();
    }
    void handleWriteRequest() noexcept override {
        MemorySpace::handleWriteRequest();
    }
private:
    Cache_t theCache_;
};
#endif //SXCHIPSETGCM4TYPE2_RAM_H
