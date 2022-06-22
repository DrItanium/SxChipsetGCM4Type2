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
    ProcessorInterface::setupDataLinesForRead();
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
    ProcessorInterface::setupDataLinesForWrite();
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

void
ContainerSpace::handleWriteRequest() noexcept {
   if (lastMatch_) {
       lastMatch_->handleWriteRequest();
   } else {
       // do a fallback to the parent implementation because something has been done out of order
       Parent::handleWriteRequest();
   }
}

void
ContainerSpace::handleReadRequest() noexcept {
    if (lastMatch_) {
        lastMatch_->handleReadRequest();
    } else {
        Parent::handleReadRequest();
    }
}

void
CompleteMemorySpace::handleReadRequest() noexcept {
    (void)updateLastMatch(ProcessorInterface::getAddress());
    Parent::handleReadRequest();
}

void
CompleteMemorySpace::handleWriteRequest() noexcept {
    (void)updateLastMatch(ProcessorInterface::getAddress());
    Parent::handleWriteRequest();
}