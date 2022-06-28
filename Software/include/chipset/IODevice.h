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
// Created by jwscoggins on 5/3/21.
//

#ifndef I960SXCHIPSET_IODEVICE_H
#define I960SXCHIPSET_IODEVICE_H
#include <stdint.h>
#include <unistd.h>
#include <string>
#include "ChipsetInteract.h"

class BuiltinIOBaseDevice {
public:
    BuiltinIOBaseDevice(uint32_t offset);
    uint32_t getBaseAddress() const { return baseAddress_; }
    uint32_t getOffset() const { return offset_; }
protected:
    uint32_t offset_;
    uint32_t baseAddress_;
};
/**
 * @brief Manages the builtin led provided by the chipset
 */
class ChipsetBasicFunctions : public BuiltinIOBaseDevice {
public:
    class SDFile {
    private:
#define TwoByteEntry(name) volatile uint16_t name
#define FourByteEntry(name) volatile uint32_t name
        struct FileInterfaceRaw {
            TwoByteEntry(ioPort);
            TwoByteEntry(flushPort);
            TwoByteEntry(syncPort);
            TwoByteEntry(isOpenPort);
            TwoByteEntry(seekEndPort);
            TwoByteEntry(seekBeginningPort);
            FourByteEntry(seekAbsolutePort);
            volatile int32_t seekRelativePort;
            FourByteEntry(sizePort);
            TwoByteEntry(permissionsPort);
            TwoByteEntry(writeErrorPort);
            TwoByteEntry(errorCodePort);
            TwoByteEntry(closePort);
        } __attribute__((packed));
#undef TwoByteEntry
#undef FourByteEntry
    public:
        SDFile(uint32_t baseAddress);
        void flush() { raw.flushPort = 1; }
        void sync() { raw.syncPort = 1; }
        void seekEnd() { raw.seekEndPort = 1; }
        void seekBeginning() { raw.seekBeginningPort = 1; }
        void seekRelative(int32_t value) { raw.seekRelativePort = value; }
        void seekAbsolute(uint32_t value) { raw.seekAbsolutePort = value; }
        bool isOpen() const { return raw.isOpenPort; }
        uint32_t getPermissions() const { return raw.permissionsPort; }
        bool getWriteError() const { return raw.writeErrorPort; }
        uint16_t getErrorCode() const { return raw.errorCodePort; }
        void putChar(char value) { raw.ioPort = value; }
        uint16_t getChar() { return raw.ioPort; }
        void close() { raw.closePort = 1; }
    private:
        volatile FileInterfaceRaw& raw;
    };
public:
    ChipsetBasicFunctions(uint32_t offset = 0);
    ~ChipsetBasicFunctions();
    void flush();
    uint16_t read() const;
    void write(uint16_t value);
    void write(char c);
    void write(const char* ptr);
    void writeLine();
    void writeLine(const char* ptr);
    /**
     * @brief sequential read from the console into the provided buffer
     * @param buffer the buffer to save to
     * @param nbyte the maximum number of bytes to read
     * @return number of bytes read
     */
    ssize_t read(char* buffer, size_t nbyte) ;
    /**
     * @brief Sequential write to the console into the provided buffer
     * @param buffer the buffer to write into
     * @param nbyte the maximum number of bytes to write
     * @return the number of bytes written
     */
    ssize_t write(char* buffer, size_t nbyte);
    void triggerInt0();
    /**
     * @brief Has the chipset been told to enable console debugging?
     * @return True if the chipset has console debugging enabled
     */
    bool addressDebuggingEnabled() const;
    /**
     * @brief Turn chipset address debugging on (VERY EXPENSIVE!!!)
     */
    void enableAddressDebugging();
    /**
     * @brief Turn chipset address debugging off
     */
    void disableAddressDebugging();

    int openFile(const char* path, int flags, int mode);

    bool readFile(int fd, void *buf, size_t sz, int &nread);
    bool writeFile(int fd, const void *buf, size_t sz, int &nwrite);
    bool closeFile(int fd);
    uint16_t getBacklightIntensity() const;
    void setBacklightIntensity(uint16_t value);
    uint32_t getButtonsRaw() const;
    uint32_t timesince2000() const;
    uint32_t unixtime() const;
    void now() const;
private:
    uint16_t waitForLegalCharacter();
private:
#define TwoByteEntry(name) volatile uint16_t name
#define FourByteEntry(name) volatile uint32_t name
#define EightByteEntry(name) volatile uint64_t name
    struct ChipsetRegistersRaw {
        volatile uint16_t consoleIOPort;
        volatile uint16_t consoleFlushPort;
        volatile uint16_t triggerInt0Port;
        volatile uint32_t addressDebuggingFlag;
    } __attribute__((packed));
    struct SDCardBaseInterfaceRaw {
        volatile char path[80];
        TwoByteEntry(openPort);
        TwoByteEntry(makeDirectoryPort);
        TwoByteEntry(existsPort);
        TwoByteEntry(removePort);
        FourByteEntry(sdCardClusterCountPort);
        FourByteEntry(sdVolumeSectorCountPort);
        TwoByteEntry(sdbytesPerSectorPort);
        TwoByteEntry(numberOfOpenFilesPort);
        TwoByteEntry(maximumNumberOfOpenFilesPort);
        TwoByteEntry(errorCodePort);
        TwoByteEntry(makeMissingParentDirectoriesPort);
        TwoByteEntry(filePermissions);
        TwoByteEntry(openReadWrite);
        TwoByteEntry(openReadOnly);
        TwoByteEntry(openWriteOnly);
        TwoByteEntry(createFileIfMissing);
        TwoByteEntry(clearFileContentsOnOpen);
        TwoByteEntry(mountCTL);
    } __attribute__((packed));
    struct RTCInterface {
        TwoByteEntry(available);
        TwoByteEntry(nowRequest);
        TwoByteEntry(seconds);
        TwoByteEntry(minutes);
        TwoByteEntry(hours);
        TwoByteEntry(day);
        TwoByteEntry(month);
        TwoByteEntry(year);
        TwoByteEntry(dayOfTheWeek);
        TwoByteEntry(reserved0);
        FourByteEntry(secondsSince2000);
        FourByteEntry(unixtime);

    } __attribute__((packed));
    struct SeesawRegisters {
        TwoByteEntry(backlight);
    } __attribute__((packed));
    struct RawDisplayInstruction {
        RawDisplayInstruction() {
            quadFields_[0] = 0;
            quadFields_[1] = 0;
        }
        RawDisplayInstruction(uint64_t a, uint64_t b) {
            quadFields_[0] = a;
            quadFields_[1] = b;
        }
        RawDisplayInstruction(uint16_t a, uint16_t b, uint16_t c, uint16_t d,
                              uint16_t e, uint16_t f, uint16_t g, uint16_t h) {
            fields_[0] = a;
            fields_[1] = b;
            fields_[2] = c;
            fields_[3] = d;
            fields_[4] = e;
            fields_[5] = f;
            fields_[6] = g;
            fields_[7] = h;
        }
        RawDisplayInstruction(uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
            doubleFields_[0] = a;
            doubleFields_[1] = b;
            doubleFields_[2] = c;
            doubleFields_[3] = d;
        }
        RawDisplayInstruction(const RawDisplayInstruction& other) {
            quadFields_[0] = other.quadFields_[0];
            quadFields_[1] = other.quadFields_[1];
        }
        union {
            uint16_t fields_[8];
            uint32_t doubleFields_[4];
            uint64_t quadFields_[2];
            struct
            {
                uint16_t iField0_;
                uint16_t iField1_;
                uint16_t iField2_;
                uint16_t iField3_;
                uint16_t iField4_;
                uint16_t iField5_;
                uint16_t iField6_;
                uint16_t iField7_;
            };
        };
    } __attribute__((packed));
    struct DisplayRegisters {
        volatile uint64_t instructionLower0;
        volatile uint64_t instructionUpper0;
        volatile uint16_t invoke0;
    }__attribute__((packed));
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
    enum Shape {
        ShapePixel = 0x00,
        ShapeLine = 0x01,
        ShapeFastVLine = 0x02,
        ShapeFastHLine = 0x03,
        ShapeRect = 0x04,
        ShapeTriangle = 0x05,
        ShapeRoundRect = 0x06,
        ShapeCircle = 0x07,
    };
    enum InvokeClass {
        InvokeClass_Draw = 0x00,
        InvokeClass_Fill = 0x10,
    };
    enum Opcodes {
        InvokeOpcodes_DrawPixel = InvokeClass_Draw | ShapePixel,
        InvokeOpcodes_DrawLine = InvokeClass_Draw | ShapeLine,
        InvokeOpcodes_DrawFastVLine = InvokeClass_Draw | ShapeFastVLine,
        InvokeOpcodes_DrawFastHLine = InvokeClass_Draw | ShapeFastHLine,
        InvokeOpcodes_DrawRect = InvokeClass_Draw | ShapeRect,
        InvokeOpcodes_DrawTriangle = InvokeClass_Draw | ShapeTriangle,
        InvokeOpcodes_DrawRoundRect = InvokeClass_Draw | ShapeRoundRect,
        InvokeOpcodes_DrawCircle = InvokeClass_Draw | ShapeCircle,

        InvokeOpcodes_FillScreen = InvokeClass_Fill | ShapePixel,
        InvokeOpcodes_FillRect = InvokeClass_Fill | ShapeRect,
        InvokeOpcodes_FillTriangle = InvokeClass_Fill | ShapeTriangle,
        InvokeOpcodes_FillRoundRect = InvokeClass_Fill | ShapeRoundRect,
        InvokeOpcodes_FillCircle = InvokeClass_Fill | ShapeCircle,
    };
public:
    void drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t fgColor);
    void drawVerticalLine(uint16_t x, uint16_t y, uint16_t height, uint16_t fgColor);
    void drawHorizontalLine(uint16_t x, uint16_t y, uint16_t width, uint16_t fgColor);
    void drawRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t fgColor, bool fill = false);
    void drawCircle(uint16_t x, uint16_t y, uint16_t radius, uint16_t fgColor, bool fill = false);
    void drawRoundedRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t radius, uint16_t fgColor, bool fill = false);
    void drawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t fgColor, bool fill = false);
    uint16_t color565(uint32_t color);
    uint16_t color565(uint8_t r, uint8_t g, uint8_t b);
    void clearScreen();
    void fillScreen(uint16_t value);
    void drawPixel(uint16_t x, uint16_t y, uint16_t color);
private:
    void installInstruction0(const RawDisplayInstruction& instruction, Opcodes opcode);
    void installInstruction0_FirstQuadField(const RawDisplayInstruction& instruction, Opcodes opcode);
private:
    volatile ChipsetRegistersRaw& _memory;
    volatile SDCardBaseInterfaceRaw& _sdbase;
    volatile SeesawRegisters& _displayAux;
    volatile DisplayRegisters& _displayItself;
    volatile RTCInterface& _rtcBase;
    SDFile** openFiles;
    uint16_t colorBlack_;
};

ChipsetBasicFunctions& getBasicChipsetInterface();
void displayIOMemoryMap();
inline uint32_t makeOrdinal(uint16_t lower, uint16_t upper) {
    return static_cast<uint32_t>(lower) | (static_cast<uint32_t>(upper) << 16);
}
inline uint64_t makeLongOrdinal(uint32_t lower, uint32_t upper) {
    return static_cast<uint64_t>(lower) | (static_cast<uint64_t>(upper) << 32);
}
inline uint64_t makeLongOrdinal(uint16_t a, uint16_t b, uint16_t c, uint16_t d) {
    return makeLongOrdinal(makeOrdinal(a, b),
                           makeOrdinal(c, d));
}

uint32_t getSPIEngineBaseAddress();
uint32_t getGPIOEngineBaseAddress();
uint32_t getTWIEngineBaseAddress();

#endif //I960SXCHIPSET_IODEVICE_H
