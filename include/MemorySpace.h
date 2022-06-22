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
/**
 * @brief Abstract representation of a memory space that can be accessed in a generic fashion
 */
class MemorySpace {
public:
    using Self = MemorySpace;
    using Ptr = std::shared_ptr<Self>;
public:
    MemorySpace() = default;
    virtual ~MemorySpace() = default;
    /**
     * @brief Write a given value to memory
     * @param address The address that we want to read from relative to the space's base address
     * @param value The value to write
     * @param lss The size of the value
     */
    virtual void write(uint32_t address, uint16_t value, LoadStoreStyle lss) noexcept { }
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
    virtual bool respondsTo(uint32_t address) const noexcept = 0;
};

/**
 * @brief A class which holds onto a set of sub memory spaces
 */
class ContainerSpace : public MemorySpace {

public:
    using MemorySpace::MemorySpace;
    ~ContainerSpace() override = default;
    void write(uint32_t address, uint16_t value, LoadStoreStyle lss) noexcept override {

    }
    uint16_t
    read(uint32_t address, LoadStoreStyle lss) const noexcept override {
        // at this point we need to use lastMatch_ as a matching criteria
        // generally, if we are at this point then we found a successful match!
        if (lastMatch_) {
            return lastMatch_->read(address, lss);
        } else {
            if (auto result = find(address); lastMatch_) {
                return lastMatch_->read(address, lss) ;
            } else {
                return 0;
            }

        }
    }
private:
    MemorySpace::Ptr
    find(uint32_t address) const noexcept {
        for (const auto& subSpace : subSpaces_) {
            if (subSpace->respondsTo(address)) {
                return subSpace;
            }
        }
        return nullptr;
    }
public:
    bool
    respondsTo(uint32_t address) const noexcept override {
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
    bool empty() const noexcept { return subSpaces_.empty(); }
    auto size() const noexcept { return subSpaces_.size(); }
    /**
     * @brief Adds the provided memory space shared_ptr to the current list, allows for duplicates!
     * @param targetPtr The pointer to add
     */
    void emplace_back(MemorySpace::Ptr targetPtr) noexcept { subSpaces_.emplace_back(targetPtr); }
private:
    // yes mutable is gross but I have an interface to satisfy
    mutable MemorySpace::Ptr lastMatch_ = nullptr;
    std::vector<MemorySpace::Ptr> subSpaces_;
};

#endif //SXCHIPSETGCM4TYPE2_MEMORYSPACE_H
