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
// Created by jwscoggins on 10/3/21.
//

#ifndef SXCHIPSET_OPENFILEHANDLE_H
#define SXCHIPSET_OPENFILEHANDLE_H
#include "Pinout.h"
#include <SdFat.h>
#include "MemorySpace.h"
#include "SinglePageMemorySpace.h"
extern SdFat SD;

/**
 * @brief A wrapper class around an sdfat file that exposes further functionality for the chipset to expose to the i960 via memory mapping
 */
class OpenFileHandle : public SinglePageMemorySpace {
public:
    enum class Registers : uint8_t {

#define TwoByteEntry(Prefix) Prefix ## 0, Prefix ## 1
#define FourByteEntry(Prefix) \
        TwoByteEntry(Prefix ## 0), \
        TwoByteEntry(Prefix ## 1)
#define EightByteEntry(Prefix) \
        FourByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define SixteenByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        EightByteEntry(Prefix ## 1)
        TwoByteEntry(IOPort),
        TwoByteEntry(Flush),

        TwoByteEntry(Sync),
        TwoByteEntry(IsOpen),

        TwoByteEntry(SeekEnd),
        TwoByteEntry(SeekBeginning),

        FourByteEntry(SeekAbsolute),
        FourByteEntry(SeekRelative),
        FourByteEntry(Size),

        TwoByteEntry(Permissions),
        TwoByteEntry(WriteError),

        TwoByteEntry(ErrorCode),
        TwoByteEntry(Close),

#undef SixteenByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
        IOPort = IOPort0,
        Flush = Flush0,
        Sync = Sync0,
        IsOpen = IsOpen0,
        SeekEnd = SeekEnd0,
        SeekBeginning = SeekBeginning0,
        SeekAbsoluteLower = SeekAbsolute00,
        SeekAbsoluteUpper = SeekAbsolute10,
        SeekRelativeLower = SeekRelative00,
        SeekRelativeUpper = SeekRelative10,
        SizeLower = Size00,
        SizeUpper = Size10,
        Permissions = Permissions0,
        WriteError = WriteError0,
        ErrorCode = ErrorCode0,
        Close = Close0,
        End,
    };
public:
    bool open(const char* path, uint16_t permissions) noexcept {
        if (!backingStore_) {
            permissions_ = permissions;
            backingStore_ = SD.open(path, static_cast<byte>(permissions));
            return backingStore_;
        } else {
            return false;
        }
    }
    bool close() noexcept {
        if (backingStore_) {
            return backingStore_.close();
        } else {
            return false;
        }
    }

    [[nodiscard]] bool isOpen() const noexcept { return backingStore_.isOpen(); }
    explicit operator bool() const noexcept { return backingStore_.operator bool(); }
    [[nodiscard]] auto size() const noexcept { return backingStore_.fileSize(); }
    [[nodiscard]] auto position() const noexcept { return backingStore_.curPosition(); }
    bool setAbsolutePosition(uint32_t pos) noexcept { return backingStore_.seekSet(pos); }
    bool setRelativePosition(int32_t pos) noexcept { return backingStore_.seekCur(pos); }
    bool seekToEnd() noexcept { return backingStore_.seekEnd(); }
    bool seekToBeginning() noexcept { return backingStore_.seekSet(0); }
    [[nodiscard]] bool getWriteError() const noexcept { return backingStore_.getWriteError(); }
    [[nodiscard]] auto getError() const noexcept { return backingStore_.getError(); }
    void flush() noexcept { backingStore_.flush(); }
    void sync() noexcept { backingStore_.sync(); }
    bool isBusy() noexcept { return backingStore_.isBusy(); }
    bool isDirectory() noexcept { return backingStore_.isDirectory(); }
    uint16_t getChar() const noexcept {
        if (backingStore_) {
            return static_cast<uint16_t>(backingStore_.read());
        } else {
            return -1;
        }
    }
    void putChar(uint16_t value) noexcept {
        if (backingStore_) {
            backingStore_.write(static_cast<byte>(value));
        }
    }
    [[nodiscard]] size_t read(void* buf, size_t count) noexcept { return backingStore_.read(buf, count); }
    [[nodiscard]] size_t write(void* buf, size_t count) noexcept { return backingStore_.write(buf, count); }
    void
    write16(uint32_t address, uint16_t value) noexcept override {
        using T = Registers;
        switch(static_cast<T>(static_cast<uint8_t>(address))) {
            case T::Close: close(); break;
            case T::IOPort: putChar(value); break;
            case T::Sync: sync(); break;
            case T::Flush: flush(); break;
            case T::SeekBeginning: seekToBeginning(); break;
            case T::SeekEnd: seekToEnd(); break;
            default: break;
        }
    }
    void
    write32(uint32_t address, uint32_t value) noexcept override {
        using T = Registers;
        switch(static_cast<T>(static_cast<uint8_t>(address))) {
            case T::SeekAbsoluteLower:
                setAbsolutePosition(value);
                break;
            case T::SeekRelativeLower:
                setRelativePosition(static_cast<int32_t>(value));
                break;
            default:
                break;
        }
    }
    uint16_t
    read16(uint32_t address) const noexcept override {
        using T = Registers;
        switch (static_cast<T>(static_cast<uint8_t>(address))) {
            case T::IOPort: return getChar();
            case T::IsOpen: return isOpen() ? 0xFFFF : 0;
            case T::Permissions: return permissions_;
            case T::ErrorCode: return getError();
            case T::WriteError: return getWriteError();
            default: return 0;
        }
    }
    uint32_t
    read32(uint32_t address) const noexcept override {
        using T = Registers;
        switch (static_cast<T>(static_cast<uint8_t>(address))) {
            case T::SizeLower: return size();
            default: return 0;
        }
    }
private:
    mutable File backingStore_;
    uint16_t permissions_ = 0;
};

#endif //SXCHIPSET_OPENFILEHANDLE_H
