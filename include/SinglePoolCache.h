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
// Created by jwscoggins on 11/19/21.
//

#ifndef SXCHIPSET_SINGLEPOOLCACHE_H
#define SXCHIPSET_SINGLEPOOLCACHE_H
#include "MCUPlatform.h"
#include "Pinout.h"
#include "TaggedCacheAddress.h"
#include "CacheEntry.h"
#include "ProcessorSerializer.h"

template<template<auto, auto, auto, typename, bool> typename C, uint32_t numEntries, byte numAddressBits, byte numOffsetBits, typename T, bool debugMode = false, bool directlyUseEntryCount = false>
class SinglePoolCache {
private:
    using FakeCacheType = C<getNumberOfBitsForNumberOfEntries(numEntries), numAddressBits, numOffsetBits, T, debugMode>;
public:
    static constexpr auto NumCacheWays = FakeCacheType::NumberOfWays;
    static constexpr auto NumberOfSets = numEntries / (directlyUseEntryCount ? 1 : NumCacheWays);
    static constexpr auto NumberOfBitsForGivenSet = getNumberOfBitsForNumberOfEntries(NumberOfSets);
    static_assert(NumberOfBitsForGivenSet > 0, "Illegal number of sets detected!");
    using CacheWay = C<NumberOfBitsForGivenSet, numAddressBits, numOffsetBits, T, debugMode>;
    static constexpr auto MaximumNumberOfEntries = numEntries;
    static constexpr auto ActualNumberOfEntries = directlyUseEntryCount ? MaximumNumberOfEntries : (MaximumNumberOfEntries / CacheWay :: NumberOfWays);
    using CacheEntry = typename CacheWay::CacheEntry;
    using TaggedAddress = typename CacheWay::TaggedAddress;
    static constexpr auto NumBytesCached = CacheEntry::NumBytesCached;
    static constexpr auto NumWordsCached = CacheEntry::NumWordsCached;
    static constexpr auto CacheEntryMask = CacheEntry::CacheEntryMask;
public:
    [[nodiscard]] CacheEntry& getLine() noexcept {
        // only align if we need to reset the chip
        return getLine(TaggedAddress(ProcessorInterface::getAddress()));
    }
    [[nodiscard]] CacheEntry& getLine(const TaggedAddress& theAddress) noexcept {
        // only align if we need to reset the chip
        if constexpr (debugMode) {
            Serial.print(F("Address Decomp: 0x"));
            Serial.print(theAddress.getRest(), HEX);
            Serial.print(F(", 0x"));
            Serial.print(theAddress.getTagIndex(), HEX);
            Serial.print(F(", 0x"));
            Serial.print(theAddress.getLowest(), HEX);
            Serial.print(F(" -> 0x"));
            Serial.println(theAddress.getAddress(), HEX);
        }
        return entries_[theAddress.getTagIndex()].getLine(theAddress);
    }
    [[nodiscard]] CacheEntry& reset(const TaggedAddress& theAddress) noexcept {
        return entries_[theAddress.getTagIndex()].reset(theAddress);
    }
    void clear() {
        // then clear both the way and underlying entries
        for (auto& a : entries_) {
            a.clear();
        }
    }
    byte* viewAsStorage() noexcept {
        return reinterpret_cast<byte*>(entries_);
    }

    void begin() noexcept {
        clear();
    }
    constexpr auto getCacheSize() const noexcept { return sizeof(entries_); }
private:
    CacheWay entries_[ActualNumberOfEntries];
};

/**
 * @brief A wrapper around a single cache pool to make chaining possible
 * @tparam C The type of the cache line
 * @tparam backingStoreSize The number of bytes that this cache will use as storage (does not include tag bits only the underlying core storage)
 * @tparam numAddressBits  the number of address bits that make up the entire tag address
 * @tparam numOffsetBits  The number of bits that make up the storage within the bytes itself
 * @tparam T A static class that the cache data is read from and written to
 */
template<template<auto, auto, auto, typename, bool> typename C,
         uint32_t backingStoreSize,
         byte numAddressBits,
         byte numOffsetBits,
         typename T,
         bool debugMode = false>
struct Cache {
public:
    static constexpr auto NumOffsetBits = numOffsetBits;
    static constexpr auto NumBackingStoreBytes = backingStoreSize;
    static constexpr auto TotalEntryCount = NumBackingStoreBytes/ pow2(NumOffsetBits);
    using UnderlyingCache_t = SinglePoolCache<C, TotalEntryCount, numAddressBits, numOffsetBits, T, debugMode, false>;
    using CacheLine = typename UnderlyingCache_t::CacheEntry;
    static constexpr auto CacheEntryMask = CacheLine::CacheEntryMask;
    static constexpr auto NumWordsCached = CacheLine::NumWordsCached;
public:
    Cache() = delete;
    ~Cache() = delete;
    Cache(const Cache&) = delete;
    Cache(Cache&&) = delete;
    /// @todo delete more of the default methods
    [[nodiscard]] static CacheLine& getLine() noexcept { return theCache_.getLine(); }
    static void begin() noexcept { theCache_.begin(); }
    static void clear() noexcept { theCache_.clear(); }
    [[nodiscard]] static constexpr auto getCacheSize() noexcept { return theCache_.getCacheSize(); }
    [[nodiscard]] static auto viewAsStorage() noexcept { return theCache_.viewAsStorage(); }
private:
    static inline UnderlyingCache_t theCache_;
};

/**
 * @brief A different kind of cache where the number of entries is defined instead of the total size.
 * @tparam C The cache set type
 * @tparam numberOfEntries The number of sets to hold in this cache
 * @tparam numAddressBits The total number of bits that make up a cache address
 * @tparam numOffsetBits The number of bits for the backing store in each cache line (6 -> 64 bytes, etc)
 * @tparam T The backing storage of this cache
 * @tparam useSpecificTypeSizes When true use the smallest type available for each bit field (will barf out in most cases)
 */
template<template<auto, auto, auto, typename, bool> typename C,
        uint32_t numberOfEntries,
        byte numAddressBits,
        byte numOffsetBits,
        typename T,
        bool debugMode = false>
struct Cache2 {
public:
    using UnderlyingCache_t = SinglePoolCache<C, numberOfEntries, numAddressBits, numOffsetBits, T, debugMode, true>;
    using CacheLine = typename UnderlyingCache_t::CacheEntry;
    static constexpr auto CacheEntryMask = CacheLine::CacheEntryMask;
    static constexpr auto NumWordsCached = CacheLine::NumWordsCached;
public:
    Cache2() = delete;
    ~Cache2() = delete;
    Cache2(const Cache2&) = delete;
    Cache2(Cache2&&) = delete;
    /// @todo delete more of the default methods
    [[nodiscard]] static CacheLine& getLine() noexcept { return theCache_.getLine(); }
    static void begin() noexcept { theCache_.begin(); }
    static void clear() noexcept { theCache_.clear(); }
    [[nodiscard]] static constexpr auto getCacheSize() noexcept { return theCache_.getCacheSize(); }
    [[nodiscard]] static auto viewAsStorage() noexcept { return theCache_.viewAsStorage(); }
private:
    static inline UnderlyingCache_t theCache_;
};

template<template<auto, auto, auto, typename, bool> typename C, uint32_t backingStoreSize, byte numAddressBits, byte numOffsetBits, typename T, bool debugMode = false>
using Cache_t = Cache<C, backingStoreSize, numAddressBits, numOffsetBits, T, debugMode>;

template<template<auto, auto, auto, typename, bool> typename C, uint32_t backingStoreSize, byte numAddressBits, byte numOffsetBits, typename T, bool debugMode = false>
using CacheInstance_t = typename Cache_t<C, backingStoreSize, numAddressBits, numOffsetBits, T, debugMode>::UnderlyingCache_t;

template<template<auto, auto, auto, typename, bool> typename C, uint32_t numEntries, byte numAddressBits, byte numOffsetBits, typename T, bool debugMode = false>
using Cache2_t = Cache2<C, numEntries, numAddressBits, numOffsetBits, T, debugMode>;

template<template<auto, auto, auto, typename, bool> typename C, uint32_t numEntries, byte numAddressBits, byte numOffsetBits, typename T, bool debugMode = false>
using Cache2Instance_t = typename Cache2_t<C, numEntries, numAddressBits, numOffsetBits, T, debugMode>::UnderlyingCache_t;

#endif //SXCHIPSET_SINGLEPOOLCACHE_H
