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
#include <optional>
/**
 * @brief Abstract representation of a memory space that can be accessed in a generic fashion; It has a base address (256-byte aligned) and consumes 1 or more pages!
 * It assumes it is the only thing in its own 32-bit memory space, use mapping objects to connect it into a memory space. It assumes a base address of zero at all times!
 */
class MemorySpace {
public:
    using Self = MemorySpace;
    using Ptr = std::shared_ptr<Self>;
public:
    virtual ~MemorySpace() = default;
    /**
     * @brief Write a given value to memory
     * @param address The address that we want to read from relative to the space's base address
     * @param value The value to write
     * @param lss The size of the value
     */
    virtual void write(uint32_t address, SplitWord16 value, LoadStoreStyle lss) noexcept = 0;
    /**
     * @brief Read a 16-bit value from this space relative to the base address
     * @param address The address that we want to read from relative to this space's base address
     * @param lss The size of the value to read
     * @return The 16-bit value
     */
    [[nodiscard]] virtual uint16_t read(uint32_t address, LoadStoreStyle lss) const noexcept = 0;
    /**
     * @brief Used to determine if this memory space responds to a given memory request
     * @param address The physical address to check for matching
     * @return a boolean value signifying if this memory space responds to the given address or not
     */
    virtual bool respondsTo(uint32_t address) const noexcept = 0;
    virtual void handleReadRequest(uint32_t baseAddress) noexcept;
    virtual void handleWriteRequest(uint32_t baseAddress) noexcept;
    void handleReadRequest() noexcept;
    void handleWriteRequest() noexcept;
    virtual uint32_t read(uint32_t address, uint16_t* value, uint32_t count) noexcept = 0;
    virtual uint32_t write(uint32_t address, uint16_t* value, uint32_t count) noexcept = 0;
    virtual uint32_t getBaseAddress() const noexcept { return 0; }
    virtual uint32_t getEndAddress() const noexcept = 0;
};
/**
 * @brief Abstract representation of a memory space that can be accessed in a generic fashion; It has a base address (256-byte aligned) and consumes 1 or more pages!
 * It assumes it is the only thing in its own 32-bit memory space, use mapping objects to connect it into a memory space. It assumes a base address of zero at all times!
 */
class SizedMemorySpace : public MemorySpace {
public:
    using Self = SizedMemorySpace;
    using Ptr = std::shared_ptr<Self>;
    static constexpr uint32_t fixBaseAddress(uint32_t value) noexcept { return value & 0xFFFFFF00; }
    static constexpr uint32_t correctPageCount(uint32_t value) noexcept { return value < 1 ? 1 : value; }
public:
    /**
     * @brief Construct a memory space with a start and length (in 256 byte pages)
     * @param baseAddress The starting address (the lowest 8 bits will be cleared out)
     * @param numPages The number of 256-byte pages consumed by this memory space (at least 1)
     */
    SizedMemorySpace(uint32_t numPages) : numPages_(correctPageCount(numPages)), endAddress_((numPages_ << 8)) { }
    ~SizedMemorySpace() override = default;
    /**
     * @brief Write a given value to memory
     * @param address The address that we want to read from relative to the space's base address
     * @param value The value to write
     * @param lss The size of the value
     */
    void write(uint32_t address, SplitWord16 value, LoadStoreStyle lss) noexcept override {
        // always operate on relative addresses
    }
    /**
     * @brief Read a 16-bit value from this space relative to the base address
     * @param address The address that we want to read from relative to this space's base address
     * @param lss The size of the value to read
     * @return The 16-bit value
     */
    [[nodiscard]] uint16_t read(uint32_t address, LoadStoreStyle lss) const noexcept override {
        return 0;
    }

    /**
     * @brief Used to determine if this memory space responds to a given memory request
     * @param address The physical address to check for matching
     * @return a boolean value signifying if this memory space responds to the given address or not
     */
    bool respondsTo(uint32_t address) const noexcept override {
        return address < endAddress_;
    }

    [[nodiscard]] constexpr auto getNumberOfPages() const noexcept { return endAddress_ >> 8; }
    [[nodiscard]] uint32_t getEndAddress() const noexcept override { return endAddress_; }
    uint32_t read(uint32_t address, uint16_t* value, uint32_t count) noexcept override;
    uint32_t write(uint32_t address, uint16_t* value, uint32_t count) noexcept override;
private:
    uint32_t numPages_;
    uint32_t endAddress_;
};

class MappedMemorySpace : public MemorySpace {
public:
    using Self = MappedMemorySpace;
    using Ptr = std::shared_ptr<Self>;
    explicit MappedMemorySpace(uint32_t baseAddress, MemorySpace& ptr) : baseAddress_(baseAddress), ptr_(ptr) { }
    ~MappedMemorySpace() override = default;
    void write(uint32_t address, SplitWord16 value, LoadStoreStyle lss) noexcept override;
    uint16_t read(uint32_t address, LoadStoreStyle lss) const noexcept override;
    bool respondsTo(uint32_t address) const noexcept override;
    void handleReadRequest(uint32_t baseAddress) noexcept override;
    void handleWriteRequest(uint32_t baseAddress) noexcept override;
    uint32_t read(uint32_t address, uint16_t *value, uint32_t count) noexcept override;
    uint32_t write(uint32_t address, uint16_t *value, uint32_t count) noexcept override;
    uint32_t getBaseAddress() const noexcept override { return baseAddress_; }
    uint32_t getEndAddress() const noexcept override { return getBaseAddress() + ptr_.getEndAddress(); }
private:
    [[nodiscard]] constexpr uint32_t makeAddressRelative(uint32_t absoluteAddress) const noexcept { return absoluteAddress - baseAddress_; }
private:
    uint32_t baseAddress_;
    MemorySpace& ptr_;
};
/**
 * @brief M
 */
class ContainerMemorySpace : public MemorySpace {
public:
    using Parent = MemorySpace;
    using Self = ContainerMemorySpace;
public:
    ~ContainerMemorySpace() override = default;
    void write(uint32_t address, SplitWord16 value, LoadStoreStyle lss) noexcept override;
    uint16_t read(uint32_t address, LoadStoreStyle lss) const noexcept override;
    bool respondsTo(uint32_t address) const noexcept override;
    void handleReadRequest(uint32_t baseAddress) noexcept override;
    void handleWriteRequest(uint32_t baseAddress) noexcept override;
    uint32_t read(uint32_t address, uint16_t *value, uint32_t count) noexcept override;
    uint32_t write(uint32_t address, uint16_t *value, uint32_t count) noexcept override;
    void emplace_back(const Parent::Ptr& target) noexcept { children_.emplace_back(target); }
private:
    Parent::Ptr find(uint32_t address) noexcept;
    const Parent::Ptr find(uint32_t address) const noexcept;
private:
    std::vector<Parent::Ptr> children_;
};

class CompleteMemorySpace : public ContainerMemorySpace {
public:
    using Self = CompleteMemorySpace;
    using Parent = ContainerMemorySpace;
    using Ptr = std::shared_ptr<Self>;
public:
    [[nodiscard]] bool respondsTo(uint32_t address) const noexcept override { return true; }
    [[nodiscard]] uint32_t getEndAddress() const noexcept override { return 0xFFFF'FFFF; }
};

/**
 * @brief Map the given memory space to the target base address
 * @param baseAddress The address to base the memory space at
 * @param space The space to be mapped
 * @return A memory mapped memory space that wraps the provided space
 */
MappedMemorySpace::Ptr map(uint32_t baseAddress, MemorySpace& space) noexcept;
/**
 * @brief Map the given memory space immediately following another memory space (generally the previous space should be mapped)
 * @param previousSpace The previous space that this new space will follow
 * @param space The space to map immediately following the previous space
 * @return A mapped memory space that follows the previous memory space
 */
MappedMemorySpace::Ptr map(const MemorySpace::Ptr& previousSpace, MemorySpace& space) noexcept;


#endif //SXCHIPSETGCM4TYPE2_MEMORYSPACE_H
