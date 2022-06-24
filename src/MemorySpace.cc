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

void
MemorySpace::handleReadRequest() noexcept {
    do {
        // wait for
        ManagementEngine::waitForCycleUnlock();
        ProcessorInterface::setDataBits(read(ProcessorInterface::getAddress(),
                                             ProcessorInterface::getStyle()));
        if (ManagementEngine::informCPU()) {
            break;
        }
        ProcessorInterface::burstNext<true>(); // advance the address
    } while (true);
}

void
MemorySpace::handleWriteRequest() noexcept {
    do {
        // wait for
        ManagementEngine::waitForCycleUnlock();
        // get the data bits but do nothing with it just to delay things
        write(ProcessorInterface::getAddress(),
              ProcessorInterface::getDataBits(),
              ProcessorInterface::getStyle());
        if (ManagementEngine::informCPU()) {
            break;
        }
        ProcessorInterface::burstNext<true>();
    } while (true);
}

uint32_t
SizedMemorySpace::read(uint32_t address, uint16_t* value, uint32_t count) noexcept {
    auto relativeStartAddress = makeAddressRelative(address);
    auto relativeTotalEndAddress = makeAddressRelative(endAddress_);
    // okay so now that we have a relative address, we need to know how many bytes to walk through
    auto relativeEndAddress = relativeStartAddress + count;
    if (relativeTotalEndAddress < relativeEndAddress) {
        relativeEndAddress = relativeTotalEndAddress;
    }
    uint32_t numRead = 0;
    for (auto i = relativeStartAddress; i < relativeEndAddress; i+=sizeof(uint16_t), ++numRead, ++value) {
        *value = read(i, LoadStoreStyle::Full16, TreatAsRelativeAddress{});
    }
    return numRead;
}
uint32_t
SizedMemorySpace::write(uint32_t address, uint16_t* value, uint32_t count) noexcept {
    auto relativeStartAddress = makeAddressRelative(address);
    auto relativeTotalEndAddress = makeAddressRelative(endAddress_);
    // okay so now that we have a relative address, we need to know how many bytes to walk through
    auto relativeEndAddress = relativeStartAddress + count;
    if (relativeTotalEndAddress < relativeEndAddress) {
        relativeEndAddress = relativeTotalEndAddress;
    }
    uint32_t numWritten = 0;
    for (auto i = relativeStartAddress; i < relativeEndAddress; i+=sizeof(uint16_t), ++numWritten, ++value) {
        write(i, SplitWord16{*value}, LoadStoreStyle::Full16, TreatAsRelativeAddress{});
    }
    return numWritten;
}
uint32_t
ContainerSpace::read(uint32_t address, uint16_t *value, uint32_t count) noexcept {
    /// @todo reimplement so that we can support spanning multiple sub memory spaces
    return MemorySpace::read(address, value, count);
}
uint32_t
ContainerSpace::write(uint32_t address, uint16_t *value, uint32_t count) noexcept {
    return MemorySpace::write(address, value, count);
}
void
MappedMemorySpace::write(uint32_t address, SplitWord16 value, LoadStoreStyle lss) noexcept {
    ptr_->write(makeAddressRelative(address), value, lss);
}
uint16_t
MappedMemorySpace::read(uint32_t address, LoadStoreStyle lss) const noexcept {
    return ptr_->read(makeAddressRelative(address), lss);
}
bool
MappedMemorySpace::respondsTo(uint32_t address) const noexcept {
    return address >= baseAddress_ && ptr_->respondsTo(address);
}
void
MappedMemorySpace::handleReadRequest(uint32_t addr) noexcept {
    ptr_->handleReadRequest(makeAddressRelative(addr));
}
void
MappedMemorySpace::handleWriteRequest(uint32_t addr) noexcept {
    ptr_->handleWriteRequest(makeAddressRelative(addr));
}
uint32_t
MappedMemorySpace::read(uint32_t address, uint16_t *value, uint32_t count) noexcept {
    return ptr_->read(makeAddressRelative(address), value, count);
}
uint32_t
MappedMemorySpace::write(uint32_t address, uint16_t *value, uint32_t count) noexcept {
    return ptr_->write(makeAddressRelative(address), value, count);
}
