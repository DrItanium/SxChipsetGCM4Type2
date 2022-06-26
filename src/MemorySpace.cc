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
//
// Created by jwscoggins on 6/21/22.
//
#include "MemorySpace.h"
#include "ManagementEngine.h"
#include "ProcessorSerializer.h"
#include <functional>

void
MemorySpace::handleReadRequest(uint32_t baseAddress) noexcept {
    for (auto address = baseAddress; ;address += 2) {
        // wait for
        ManagementEngine::waitForCycleUnlock();
        SplitWord32 container{0};
        if (ManagementEngine::isLastCycleOfTransaction()) {
            container.setLowerHalf(read16(address));
        } else {
            container.setWholeValue(read32(address));
        }
        ProcessorInterface::setDataBits(container.getLowerHalf());
        if (ManagementEngine::informCPU()) {
            break;
        }
        ManagementEngine::waitForCycleUnlock();
        ProcessorInterface::setDataBits(container.getUpperHalf());
        if (ManagementEngine::informCPU()) {
            break;
        }
    }
}
void
MemorySpace::dispatchWriteRequest16(uint32_t address, LoadStoreStyle style, const SplitWord16& value) noexcept {
    switch (style) {
        case LoadStoreStyle::Full16:
            write16(address, value.getWholeValue());
            break;
        case LoadStoreStyle::Upper8:
            write8(address + 1, value.getUpperHalf());
            break;
        case LoadStoreStyle::Lower8:
            write8(address, value.getLowerHalf());
            break;
        default:
            break;
    }
}
void
MemorySpace::dispatchWriteRequest32(uint32_t address, LoadStoreStyle style, LoadStoreStyle style2, const SplitWord32 &value) noexcept {
    switch (convert16To32(style, style2)) {
        case LoadStoreStyle32::Full32:
            write32(address, value.getWholeValue());
            break;
        case LoadStoreStyle32::Upper16_Lower8:
            write8(address + 1, value.getLowerByte());
            write16(address + 2, value.getUpperHalf());
            break;
        case LoadStoreStyle32::Upper16_Lowest8:
            write8(address, value.getLowestByte());
            write16(address + 2, value.getUpperHalf());
            break;
        case LoadStoreStyle32::Upper16:
            write16(address + 2, value.getUpperHalf());
            break;
        case LoadStoreStyle32::Highest8:
            write8(address + 3, value.getHighestByte());
            break;
        case LoadStoreStyle32::Highest8_Lower16:
            write16(address, value.getLowerHalf());
            write8(address + 3, value.getHighestByte());
            break;
        case LoadStoreStyle32::Highest8_Lower8:
            write8(address + 1, value.getLowerByte());
            write8(address + 3, value.getHighestByte());
            break;
        case LoadStoreStyle32::Highest8_Lowest8:
            write8(address, value.getLowestByte());
            write8(address + 3, value.getHighestByte());
            break;
        case LoadStoreStyle32::Higher8:
            write8(address + 2, value.getHigherByte());
            break;
        case LoadStoreStyle32::Higher8_Lower16:
            write16(address, value.getLowerHalf());
            write8(address + 2, value.getHigherByte());
            break;
        case LoadStoreStyle32::Higher8_Lower8:
            write8(address + 1, value.getLowerByte());
            write8(address + 2, value.getHigherByte());
            break;
        case LoadStoreStyle32::Higher8_Lowest8:
            write8(address + 0, value.getLowerByte());
            write8(address + 2, value.getHigherByte());
            break;
        case LoadStoreStyle32::Lower16:
            write16(address, value.getLowerHalf());
            break;
        case LoadStoreStyle32::Lowest8:
            write8(address, value.getLowestByte());
            break;
        case LoadStoreStyle32::Lower8:
            write8(address+1, value.getLowerByte());
            break;
        default:
            break;
    }
}

void
MemorySpace::handleWriteRequest(uint32_t baseAddress) noexcept {
    for (auto address = baseAddress; ; address += 2) {
        ManagementEngine::waitForCycleUnlock();
        SplitWord32 container{0};
        container.setLowerWord(ProcessorInterface::getDataBits());
        auto style0 = ProcessorInterface::getStyle();
        if (ManagementEngine::informCPU()) {
            dispatchWriteRequest16(address, style0, container.getLowerWord()));
            break;
        }
        ManagementEngine::waitForCycleUnlock();
        container.setUpperWord(ProcessorInterface::getDataBits());
        auto style1 = ProcessorInterface::getStyle();
        dispatchWriteRequest32(address, style0, style1, container);
        if (ManagementEngine::informCPU()) {
            break;
        }

    }
}
#if 0

uint32_t
SizedMemorySpace::read(uint32_t address, uint16_t* value, uint32_t count) noexcept {
    auto relativeStartAddress = address;
    auto relativeTotalEndAddress = endAddress_;
    // okay so now that we have a relative address, we need to know how many bytes to walk through
    auto relativeEndAddress = relativeStartAddress + count;
    if (relativeTotalEndAddress < relativeEndAddress) {
        relativeEndAddress = relativeTotalEndAddress;
    }
    uint32_t numRead = 0;
    for (auto i = relativeStartAddress; i < relativeEndAddress; i+=sizeof(uint16_t), ++numRead, ++value) {
        *value = read(i, LoadStoreStyle::Full16);
    }
    return numRead;
}
uint32_t
SizedMemorySpace::write(uint32_t address, uint16_t* value, uint32_t count) noexcept {
    auto relativeStartAddress = address;
    auto relativeTotalEndAddress = endAddress_;
    // okay so now that we have a relative address, we need to know how many bytes to walk through
    auto relativeEndAddress = relativeStartAddress + count;
    if (relativeTotalEndAddress < relativeEndAddress) {
        relativeEndAddress = relativeTotalEndAddress;
    }
    uint32_t numWritten = 0;
    for (auto i = relativeStartAddress; i < relativeEndAddress; i+=sizeof(uint16_t), ++numWritten, ++value) {
        write(i, SplitWord16{*value}, LoadStoreStyle::Full16);
    }
    return numWritten;
}
void
MappedMemorySpace::write(uint32_t address, SplitWord16 value, LoadStoreStyle lss) noexcept {
    ptr_.write(makeAddressRelative(address), value, lss);
}
uint16_t
MappedMemorySpace::read(uint32_t address, LoadStoreStyle lss) const noexcept {
    return ptr_.read(makeAddressRelative(address), lss);
}
#endif
bool
MappedMemorySpace::respondsTo(uint32_t address) const noexcept {
    return address >= baseAddress_ && ptr_.respondsTo(address);
}
void
MappedMemorySpace::handleReadRequest(uint32_t addr) noexcept {
    ptr_.handleReadRequest(makeAddressRelative(addr));
}
void
MappedMemorySpace::handleWriteRequest(uint32_t addr) noexcept {
    ptr_.handleWriteRequest(makeAddressRelative(addr));
}
void
MappedMemorySpace::write8(uint32_t address, uint8_t value) noexcept {
    MemorySpace::write8(address, value);
}
void
MappedMemorySpace::write16(uint32_t address, uint16_t value) noexcept {
    MemorySpace::write16(address, value);
}
void
MappedMemorySpace::write32(uint32_t address, uint32_t value) noexcept {
    MemorySpace::write32(address, value);
}
uint16_t
MappedMemorySpace::read16(uint32_t address) noexcept {
    return MemorySpace::read16(address);
}
uint32_t
MappedMemorySpace::read32(uint32_t address) noexcept {
    return MemorySpace::read32(address);
}
uint32_t
MappedMemorySpace::read(uint32_t address, uint8_t *value, uint32_t count) noexcept {
    return 0;
}
uint32_t
MappedMemorySpace::write(uint32_t address, uint8_t *value, uint32_t count) noexcept {
    return 0;
}
#if 0
uint32_t
MappedMemorySpace::read(uint32_t address, uint16_t *value, uint32_t count) noexcept {
    return ptr_.read(makeAddressRelative(address), value, count);
}
uint32_t
MappedMemorySpace::write(uint32_t address, uint16_t *value, uint32_t count) noexcept {
    return ptr_.write(makeAddressRelative(address), value, count);
}
#endif

void
MemorySpace::handleWriteRequest() noexcept {
    handleWriteRequest(ProcessorInterface::getAddress());
}
void
MemorySpace::handleReadRequest() noexcept {
    handleReadRequest(ProcessorInterface::getAddress());
}
#if 0
void
ContainerMemorySpace::write(uint32_t address, SplitWord16 value, LoadStoreStyle lss) noexcept {
    if (auto result = find(address); result) {
        result->write(address, value, lss);
    }
}
uint16_t
ContainerMemorySpace::read(uint32_t address, LoadStoreStyle lss) const noexcept {
    if (auto result = find(address); result) {
        return result->read(address, lss);
    }
    return 0;
}
#endif
bool
ContainerMemorySpace::respondsTo(uint32_t address) const noexcept {
    return find(address) != nullptr;
}

void
ContainerMemorySpace::handleReadRequest(uint32_t baseAddress) noexcept {
    if (auto result = find(baseAddress); result) {
        result->handleReadRequest(baseAddress);
    } else {
        Parent::handleReadRequest(baseAddress);
    }
}
void
ContainerMemorySpace::handleWriteRequest(uint32_t baseAddress) noexcept {
    if (auto result = find(baseAddress); result) {
        result->handleWriteRequest(baseAddress);
    } else {
        Parent::handleWriteRequest(baseAddress);
    }
}
#if 0
uint32_t
ContainerMemorySpace::read(uint32_t address, uint16_t *value, uint32_t count) noexcept {
    if (auto result = find(address); result) {
        return result->read(address, value, count);
    } else {
        return 0;
    }
}
uint32_t
ContainerMemorySpace::write(uint32_t address, uint16_t *value, uint32_t count) noexcept {
    if (auto result = find(address); result) {
        return result->write(address, value, count);
    } else {
        return 0;
    }
}
#endif
MemorySpace::Ptr
ContainerMemorySpace::find(uint32_t address) noexcept {
    for (auto& a : children_) {
        if (a->respondsTo(address))  {
            return a;
        }
    }
    return nullptr;
}

const MemorySpace::Ptr
ContainerMemorySpace::find(uint32_t address) const noexcept {
    for (const auto& a : children_) {
        if (a->respondsTo(address))  {
            return a;
        }
    }
    return nullptr;
}
MappedMemorySpace::Ptr
map(uint32_t baseAddress, MemorySpace& space) noexcept {
    return std::make_shared<MappedMemorySpace>(baseAddress, space);
}

MappedMemorySpace::Ptr
map(const MemorySpace::Ptr& previousSpace, MemorySpace& space) noexcept {
    return map(previousSpace->getEndAddress(), space);
}

