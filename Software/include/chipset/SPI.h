/*
hitagimon
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
#ifndef HITAGIMON_DMA_H__
#define HITAGIMON_DMA_H__
#include <stdint.h>
#include "IODevice.h"
#include "lang/noexcept.h"
#include "ChipsetInteract.h"

/**
 * Simple wrapper around the SPI interface provided by the Chipset. It does not fiddle with GPIOs, that is the job of the GPIO class!
 */
class SPIEngine {
public:
    typedef uint8_t* Buffer;
    /**
     * @brief Raw view of the SPI control registers
     */
    struct Request {
        Buffer src;
        Buffer dest;
        uint32_t transferRate;
        union Flags {
            uint32_t full;
            struct {
                uint32_t overwriteSrc : 1;
                uint32_t mode : 2;
                uint32_t dataOrder : 1;
                uint32_t count : 8;
            };
        } flags;
        uint8_t size() const { return flags.count; }
        bool overwriteSource() const { return flags.overwriteSrc; }
        uint32_t fullConfigurationRegister() const { return flags.full; }
        void clear();
    } __attribute__((packed));
    struct RawView {
        volatile uint32_t valid;
        volatile Request* requestBaseAddress;
        volatile uint32_t maximumSpeed;
        volatile uint32_t ready;
    } __attribute__((packed));
    SPIEngine();
    ~SPIEngine();
    void begin();
    void transfer(Buffer src, uint8_t count) { transfer(src, 0, count, true); }
    void transfer(Buffer src, Buffer dest, uint8_t count) { transfer(src, dest, count, false); }
    uint8_t transfer(uint8_t value);
    uint32_t getMaximumTransferRate() const { return raw_.maximumSpeed; }
    uint32_t getCurrentTransferRate() const { return currentTransferRate_; }
    void setCurrentTransferRate(uint32_t value);
    bool available() const { return raw_.valid; }
    operator bool() const { return available(); }
    void setMode(int value) { mode_ = value; }
    void setByteOrder(bool littleEndian) { dataOrder_ = littleEndian; }
    inline void setByteOrderLSB() { setByteOrder(true); }
    inline void setByteOrderMSB() { setByteOrder(false); }
private:
    void transfer(Buffer src, Buffer dest, uint8_t count, bool overwriteSource);
private:
    volatile RawView& raw_;
    uint32_t currentTransferRate_;
    uint32_t mode_ : 2;
    uint32_t dataOrder_ : 1;
    Request internalRequest_;
};

SPIEngine& getSPIEngine();
#endif // end HITAGIMON_DMA_H__
