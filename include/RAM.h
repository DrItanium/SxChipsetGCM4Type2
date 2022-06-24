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
class RAM : public SizedMemorySpace {
public:
    using Self = RAM;
    using Ptr = std::shared_ptr<Self>;
    using Parent = SizedMemorySpace;
    using BackingMemoryStorage_t = S;
    static constexpr auto CacheLineSize = TargetBoard::getCacheLineSizeInBits();
    static constexpr auto CompileInAddressDebuggingSupport = TargetBoard::compileInAddressDebuggingSupport();
    static constexpr auto AddressDebuggingEnabledOnStartup = TargetBoard::addressDebuggingEnabledOnStartup();
    static constexpr auto CompileInCacheSystemDebuggingSupport = TargetBoard::compileInCacheSystemDebuggingSupport();
    static constexpr auto CompileInExtendedDebugInformation = TargetBoard::compileInExtendedDebugInformation();
    static constexpr auto ValidateTransferDuringInstall = TargetBoard::validateTransferDuringInstall();
    using Cache_t = Cache2Instance_t<TenWayRandPLRUCacheWay, 256, CacheLineSize, BackingMemoryStorage_t, CompileInAddressDebuggingSupport>;
    static constexpr auto NumPages = (512 * 1024 * 1024) / 256;
    using TaggedAddress = typename Cache_t::TaggedAddress;
public:
    RAM() : Parent(NumPages) {
        theCache_.begin();
    }
    ~RAM() override = default;
    void write(uint32_t address, SplitWord16 value, LoadStoreStyle lss) noexcept override {
        auto& line = theCache_.getLine(TaggedAddress{address});
        line.set(ProcessorInterface::getCacheOffsetEntry<Cache_t::CacheEntryMask>(SplitWord32{address}),
                 lss,
                 value);
    }
    [[nodiscard]]
    uint16_t
    read(uint32_t address, LoadStoreStyle lss) const noexcept override {
        return theCache_.getLine(TaggedAddress{address}).get(ProcessorInterface::getCacheOffsetEntry<Cache_t::CacheEntryMask>(SplitWord32{address}));
    }
    void
    handleReadRequest(uint32_t baseAddress) noexcept override {
        auto start = ProcessorInterface::getCacheOffsetEntry<decltype(theCache_)::CacheEntryMask>(SplitWord32{baseAddress});
        auto end = start + 8;
        auto& theEntry = theCache_.getLine(TaggedAddress{baseAddress});
        // when dealing with read operations, we can actually easily unroll the do while by starting at the cache offset entry and walking
        // forward until we either hit the end of the cache line or blast is asserted first (both are valid states)
        for (auto i = start; i < end; ++i) {
            // start working on getting the given value way ahead of cycle unlock happening
            ManagementEngine::waitForCycleUnlock();
            ProcessorInterface::setDataBits(theEntry.get(i));
            // Only pay for what we need even if it is slower
            if (ManagementEngine::informCPU()) {
                break;
            }
            // so if I don't increment the address, I think we run too fast xD based on some experimentation
            ProcessorInterface::burstNext<false>();
        }
    }
    void handleWriteRequest(uint32_t baseAddress) noexcept override {
        auto start = ProcessorInterface::getCacheOffsetEntry<decltype(theCache_)::CacheEntryMask>(SplitWord32{baseAddress});
        auto end = start + 8;

        auto& theEntry = theCache_.getLine(TaggedAddress{baseAddress});
        // when dealing with writes to the cache line we are safe in just looping through from the start to at most 8 because that is as
        // far as we can go with how the Sx works!

        // Also the manual states that the processor cannot burst across 16-byte boundaries so :D.
        for (auto i = start; i < end; ++i) {
            ManagementEngine::waitForCycleUnlock();
            theEntry.set(i, ProcessorInterface::getStyle(),
                         ProcessorInterface::getDataBits());
            if (ManagementEngine::informCPU()) {
                break;
            }
            // the manual doesn't state that the burst transaction will always have BE0 and BE1 pulled low and this is very true, you must
            // check the pins because it will do unaligned burst transactions but even that will never span multiple 16-byte entries
            // so if I don't increment the address, I think we run too fast xD based on some experimentation
            ProcessorInterface::burstNext<false>();
        }
    }
    byte* viewCacheAsStorage() noexcept { return theCache_.viewAsStorage(); }
    void clear() noexcept { theCache_.clear(); }
    constexpr auto getCacheSize() const noexcept { return theCache_.getCacheSize(); }
private:
    Cache_t theCache_;
};
#endif //SXCHIPSETGCM4TYPE2_RAM_H
