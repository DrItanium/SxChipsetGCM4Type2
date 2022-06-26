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
protected:
public:
    /// @todo implement the read/write 8,16,32 routines at some point
    /// @todo implement custom routines for read and write block
public:
    void
    handleReadRequest(uint32_t baseAddress) noexcept override {
        auto start = ProcessorInterface::getCacheOffsetEntry<decltype(theCache_)::CacheEntryMask>(SplitWord32{baseAddress});
        auto end = start + 8;
        auto& theEntry = theCache_.getLine(TaggedAddress{baseAddress});
        // when dealing with read operations, we can actually easily unroll the do while by starting at the cache offset entry and walking
        // forward until we either hit the end of the cache line or blast is asserted first (both are valid states)
        for (auto i = start; i < end; i+=2) {
            // start working on getting the given value way ahead of cycle unlock happening
            ManagementEngine::waitForCycleUnlock();
            // load 32-bits at a time from the cache
            /// @todo should we care if we are the last word in a cache line which is always a multiple of 16 so also aligned to 16-byte boundaries? Is corruption of the other word okay
            SplitWord32 theWord{theEntry.get(i), theEntry.get(i+1)};
            ProcessorInterface::setDataBits(theWord.getLowerHalf());
            // Only pay for what we need even if it is slower
            if (ManagementEngine::informCPU()) {
                break;
            }
            ManagementEngine::waitForCycleUnlock();
            ProcessorInterface::setDataBits(theWord.getUpperHalf());
            if (ManagementEngine::informCPU()) {
                break;
            }
        }
    }
    void handleWriteRequest(uint32_t baseAddress) noexcept override {
        auto start = ProcessorInterface::getCacheOffsetEntry<decltype(theCache_)::CacheEntryMask>(SplitWord32{baseAddress});
        auto end = start + 8;

        auto& theEntry = theCache_.getLine(TaggedAddress{baseAddress});
        // when dealing with writes to the cache line we are safe in just looping through from the start to at most 8 because that is as
        // far as we can go with how the Sx works!

        // Also the manual states that the processor cannot burst across 16-byte boundaries so :D.
        for (auto i = start; i < end; i+=2) {
            ManagementEngine::waitForCycleUnlock();
            SplitWord32 theWord{0};
            theWord.setLowerWord(ProcessorInterface::getDataBits());
            auto style0 = ProcessorInterface::getStyle();
            if (ManagementEngine::informCPU()) {
                theEntry.set(i, style0, theWord.getLowerWord());
                break;
            }
            ManagementEngine::waitForCycleUnlock();
            theWord.setUpperWord(ProcessorInterface::getDataBits());
            auto style1 = ProcessorInterface::getStyle();
            theEntry.set32(i + 1, convert16To32(style0, style1), theWord);
            if (ManagementEngine::informCPU()) {
                break;
            }
        }
    }
    byte* viewCacheAsStorage() noexcept { return theCache_.viewAsStorage(); }
    void clear() noexcept { theCache_.clear(); }
    constexpr auto getCacheSize() const noexcept { return theCache_.getCacheSize(); }
private:
    Cache_t theCache_;
};
#endif //SXCHIPSETGCM4TYPE2_RAM_H
