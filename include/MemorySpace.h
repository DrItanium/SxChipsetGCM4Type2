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
    baseAddress_(fixBaseAddress(baseAddress)),
    endAddress_(baseAddress_ + (correctPageCount(numPages) << 8))
    {

    }
    struct TreatAsAbsoluteAddress { };
    struct TreatAsRelativeAddress { };
    virtual ~MemorySpace() = default;
    /**
     * @brief Write a given value to memory
     * @param address The address that we want to read from relative to the space's base address
     * @param value The value to write
     * @param lss The size of the value
     */
    inline void write(uint32_t address, SplitWord16 value, LoadStoreStyle lss, TreatAsAbsoluteAddress) noexcept {
        write(makeAddressRelative(address), value, lss, TreatAsRelativeAddress{});
    }
    virtual void write(uint32_t address, SplitWord16 value, LoadStoreStyle lss, TreatAsRelativeAddress) noexcept {
        // always operate on relative addresses
    }
    void write(uint32_t address, uint16_t value, LoadStoreStyle style) noexcept { write(address, SplitWord16(value), style, TreatAsAbsoluteAddress{}); }
    /**
     * @brief Read a 16-bit value from this space relative to the base address
     * @param address The address that we want to read from relative to this space's base address
     * @param lss The size of the value to read
     * @return The 16-bit value
     */
    [[nodiscard]] virtual uint16_t read(uint32_t address, LoadStoreStyle lss, TreatAsRelativeAddress) const noexcept {
        return 0;
    }

    [[nodiscard]] inline uint16_t read(uint32_t address, LoadStoreStyle lss, TreatAsAbsoluteAddress) const noexcept { return read(makeAddressRelative(address), lss, TreatAsRelativeAddress{}); }
    [[nodiscard]] inline uint16_t read(uint32_t address, LoadStoreStyle lss) const noexcept { return read(address, lss, TreatAsAbsoluteAddress{}); }
    /**
     * @brief Used to determine if this memory space responds to a given memory request
     * @param address The physical address to check for matching
     * @return a boolean value signifying if this memory space responds to the given address or not
     */
    virtual bool respondsTo(uint32_t address) const noexcept {
        return address >= baseAddress_ && address < endAddress_;
    }

    [[nodiscard]] constexpr auto getNumberOfPages() const noexcept { return (endAddress_ - baseAddress_) >> 8; }
    [[nodiscard]] constexpr auto getBaseAddress() const noexcept { return baseAddress_; }
    [[nodiscard]] constexpr auto getEndAddress() const noexcept { return endAddress_; }
    virtual void handleReadRequest() noexcept;
    virtual void handleWriteRequest() noexcept;
    /**
     * @brief Convert an absolute address into one that is relative to the current memory space
     * @param absoluteAddress The absolute address
     * @return The address relative to the starting position
     */
    virtual uint32_t makeAddressRelative(uint32_t absoluteAddress) const noexcept {
        return absoluteAddress - baseAddress_;
    }
    virtual uint32_t read(uint32_t address, uint16_t* value, uint32_t count) noexcept;
    virtual uint32_t write(uint32_t address, uint16_t* value, uint32_t count) noexcept;
    /// @todo implement block read/write support
private:
    uint32_t baseAddress_;
    uint32_t endAddress_;
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
    write(uint32_t address, SplitWord16 value, LoadStoreStyle lss, TreatAsRelativeAddress) noexcept override {
        if (auto result = find(address); result) {
            // the address is a relative address according to this memory space, but to children it is an absolute address
            result->write(address, value, lss, TreatAsAbsoluteAddress{}) ;
        }
    }
    uint16_t
    read(uint32_t address, LoadStoreStyle lss, TreatAsRelativeAddress) const noexcept override {
        // generally, if we are at this point then we found a successful match!
        if (auto result = find(address); result) {
            // the address is relative to the parent but is an absolute address inside this memory space according to its children
            return result->read(address, lss, TreatAsAbsoluteAddress{});
        } else {
            return 0;
        }
    }
    uint32_t read(uint32_t address, uint16_t *value, uint32_t count) noexcept override;
    uint32_t write(uint32_t address, uint16_t *value, uint32_t count) noexcept override;
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
public:
    bool empty() const noexcept { return subSpaces_.empty(); }
    auto size() const noexcept { return subSpaces_.size(); }
    /// @todo implement support for constructing child memory spaces relative to zero instead of absolute 32-bit addresses
    void emplace_back(MemorySpace::ObserverPtr targetPtr) noexcept { subSpaces_.emplace_back(targetPtr); }
    template<typename T>
    void emplace_back(T& targetPtr) noexcept { emplace_back(std::experimental::make_observer(&targetPtr)); }
private:
    // yes mutable is gross but I have an interface to satisfy
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
};
#endif //SXCHIPSETGCM4TYPE2_MEMORYSPACE_H
