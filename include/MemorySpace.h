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

#ifndef SXCHIPSETGCM4TYPE2_MEMORYSPACE_H
#define SXCHIPSETGCM4TYPE2_MEMORYSPACE_H
#include "MCUPlatform.h"
#include "Pinout.h"
#include <vector>
#include <memory>
#include <experimental/memory>
/**
 * @brief Abstract representation of a memory space that can be accessed in a generic fashion; It has a base address (256-byte aligned) and consumes 1 or more pages!
 */
class MemorySpace {
public:
    using Self = MemorySpace;
    using Ptr = std::shared_ptr<Self>;
    using ObserverPtr = std::experimental::observer_ptr<Self>;
    static constexpr uint32_t fixBaseAddress(uint32_t value) noexcept { return value & 0xFFFFFF00; }
    static constexpr uint32_t correctPageCount(uint32_t value) noexcept { return value < 1 ? 1 : value; }
public:
    /**
     * @brief Construct a memory space with a start and length (in 256 byte pages)
     * @param baseAddress The starting address (the lowest 8 bits will be cleared out)
     * @param numPages The number of 256-byte pages consumed by this memory space (at least 1)
     */
    MemorySpace(uint32_t baseAddress, uint32_t numPages) :
    numberOfPages_(correctPageCount(numPages)),
    baseAddress_(fixBaseAddress(baseAddress)),
    endAddress_(baseAddress_ + (numberOfPages_ << 8)),
    internalMask_(endAddress_ - 1)
    {

    }
    virtual ~MemorySpace() = default;
    /**
     * @brief Write a given value to memory
     * @param address The address that we want to read from relative to the space's base address
     * @param value The value to write
     * @param lss The size of the value
     */
    virtual void write(uint32_t address, SplitWord16 value, LoadStoreStyle lss) noexcept { }
    void write(uint32_t address, uint16_t value, LoadStoreStyle style) noexcept { write(address, SplitWord16(value), style); }
    /**
     * @brief Read a 16-bit value from this space relative to the base address
     * @param address The address that we want to read from relative to this space's base address
     * @param lss The size of the value to read
     * @return The 16-bit value
     */
    [[nodiscard]] virtual uint16_t read(uint32_t address, LoadStoreStyle lss) const noexcept { return 0; }

    /**
     * @brief Used to determine if this memory space responds to a given memory request
     * @param address The physical address to check for matching
     * @return a boolean value signifying if this memory space responds to the given address or not
     */
    virtual bool respondsTo(uint32_t address) const noexcept {
        return address >= baseAddress_ && address < endAddress_;
    }

    virtual uint32_t write(uint32_t baseAddress, uint8_t* data, uint32_t count) noexcept = 0;
    virtual uint32_t read(uint32_t baseAddress, uint8_t* data, uint32_t count) noexcept = 0;
    [[nodiscard]] constexpr auto getNumberOfPages() const noexcept { return numberOfPages_; }
    [[nodiscard]] constexpr auto getBaseAddress() const noexcept { return baseAddress_; }
    [[nodiscard]] constexpr auto getEndAddress() const noexcept { return endAddress_; }
    [[nodiscard]] constexpr auto getInternalMask() const noexcept { return internalMask_; }

    virtual void handleReadRequest() noexcept;
    virtual void handleWriteRequest() noexcept;
private:
    uint32_t numberOfPages_;
    uint32_t baseAddress_;
    uint32_t endAddress_;
    uint32_t internalMask_;
};

/**
 * @brief A class which holds onto a set of sub memory spaces
 */
class ContainerSpace : public MemorySpace {
public:
    using Parent = MemorySpace;
    using Self = ContainerSpace;
public:
    ContainerSpace(uint32_t baseAddress, uint32_t numPages) : Parent(baseAddress, numPages) { }
    ~ContainerSpace() override = default;

    void
    write(uint32_t address, SplitWord16 value, LoadStoreStyle lss) noexcept override {
        if (lastMatch_) {
            lastMatch_->write(address, value, lss);
        } else {
            if (auto result = find(address); result) {
                result->write(address, value, lss) ;
                lastMatch_ = result;
            }
        }
    }
    uint16_t
    read(uint32_t address, LoadStoreStyle lss) const noexcept override {
        // at this point we need to use lastMatch_ as a matching criteria
        // generally, if we are at this point then we found a successful match!
        if (lastMatch_) {
            return lastMatch_->read(address, lss);
        } else {
            if (auto result = find(address); result) {
                lastMatch_ = result;
                return lastMatch_->read(address, lss) ;
            } else {
                return 0;
            }

        }
    }
protected:
    MemorySpace::ObserverPtr
    find(uint32_t address) const noexcept {
        for (const auto& subSpace : subSpaces_) {
            if (subSpace->respondsTo(address)) {
                return subSpace;
            }
        }
        return nullptr;
    }
    bool
    updateLastMatch(uint32_t address) const noexcept {
        if (lastMatch_ && lastMatch_->respondsTo(address)) {
            return true;
        } else {
            if (auto subSpace = find(address); subSpace) {
                lastMatch_ = subSpace;
                return true;
            } else {
                return false;
            }
        }
    }
public:
    bool
    respondsTo(uint32_t address) const noexcept override {
        if (Parent::respondsTo(address)) {
            return updateLastMatch(address);
        } else {
            return false;
        }
    }
    bool empty() const noexcept { return subSpaces_.empty(); }
    auto size() const noexcept { return subSpaces_.size(); }
    void emplace_back(MemorySpace::ObserverPtr targetPtr) noexcept { subSpaces_.emplace_back(targetPtr); }
    template<typename T>
    void emplace_back(T& targetPtr) noexcept { emplace_back(std::experimental::make_observer(&targetPtr)); }

    uint32_t
    write(uint32_t baseAddress, uint8_t* data, uint32_t count) noexcept override {
        // bypass lastMatch to make sure we always do the right thing
        if (auto result = find(baseAddress); result) {
            return result->write(baseAddress, data, count);
        } else {
            return 0;
        }

    }
    uint32_t
    read(uint32_t baseAddress, uint8_t* data, uint32_t count) noexcept override {
        if (auto result = find(baseAddress); result) {
            return result->read(baseAddress, data, count);
        } else {
            return 0;
        }
    }
    void handleReadRequest() noexcept override;
    void handleWriteRequest() noexcept override;
private:
    // yes mutable is gross but I have an interface to satisfy
    mutable ObserverPtr lastMatch_ = nullptr;
    std::vector<ObserverPtr> subSpaces_;
};
class CompleteMemorySpace : public ContainerSpace {
public:
    using Self = CompleteMemorySpace;
    using Parent = ContainerSpace;
    using Ptr = std::shared_ptr<Self>;
public:
    CompleteMemorySpace() : Parent(0, 0x00FFFFFF) { }
    bool respondsTo(uint32_t address) const noexcept override { return true; }
    void handleReadRequest() noexcept override;
    void handleWriteRequest() noexcept override;
};
#endif //SXCHIPSETGCM4TYPE2_MEMORYSPACE_H
