//
// Created by jwscoggins on 5/3/21.
//

#include "chipset/IODevice.h"
#include "chipset/ChipsetInteract.h"
#include <string.h>
#include <errno.h>
#include <fcntl.h>

namespace {
    struct IOConfigurationSpaceView {
        volatile uint32_t serial0Addr;
        volatile uint32_t sdCardCtlAddr;
        volatile uint32_t sdCardFileBegin;
        volatile uint32_t auxDisplayStart;
        volatile uint32_t displayFunctionsStart;
        volatile uint32_t rtcStart;
        volatile uint32_t spiEngineStart;
        volatile uint32_t gpioEngineStart;
        volatile uint32_t twiEngineStart;
    } __attribute__((packed));
    volatile IOConfigurationSpaceView& getConfiguration0() {
        return memory<IOConfigurationSpaceView>(getIOBase0Address(0));
    }
    uint32_t getChipsetRegistersBase() {
        return getConfiguration0().serial0Addr;
    }
    uint32_t getSDCardRegisterBase() {
        return getConfiguration0().sdCardCtlAddr;
    }

    uint32_t getSDCardFileBase() {
        return getConfiguration0().sdCardFileBegin;
    }
    uint32_t getAuxDisplayFunctionsBase() {
        return getConfiguration0().auxDisplayStart;
    }
    uint32_t getDisplayFunctionsBase() {
        return getConfiguration0().displayFunctionsStart;
    }
    uint32_t getRTCBase() {
        return getConfiguration0().rtcStart;
    }
}
uint32_t
getSPIEngineBaseAddress() {
    return getConfiguration0().spiEngineStart;
}
uint32_t
getGPIOEngineBaseAddress() {
    return getConfiguration0().gpioEngineStart;
}
uint32_t
getTWIEngineBaseAddress() {
    return getConfiguration0().twiEngineStart;
}
uint16_t
computeColor565(uint8_t r, uint8_t g, uint8_t b) {
    // don't actually ask the chipset for this
    return (static_cast<uint16_t>(r & 0xF8) << 8) |
           (static_cast<uint16_t>(g & 0xFC) << 3) |
           (static_cast<uint16_t>(b>>3));
}
uint16_t
ChipsetBasicFunctions::color565(uint8_t r, uint8_t g, uint8_t b) {
    // don't actually ask the chipset for this
    return computeColor565(r, g, b);
}
BuiltinIOBaseDevice::BuiltinIOBaseDevice(uint32_t offset) : offset_(offset), baseAddress_(getIOBase0Address(offset)) { }
ChipsetBasicFunctions::SDFile::SDFile(uint32_t baseAddress) : raw(memory<FileInterfaceRaw>(baseAddress)) {}
ChipsetBasicFunctions::ChipsetBasicFunctions(uint32_t offset) : BuiltinIOBaseDevice(offset),
_memory(memory<ChipsetRegistersRaw>(getChipsetRegistersBase())),
_sdbase(memory<SDCardBaseInterfaceRaw>(getSDCardRegisterBase())),
_displayAux(memory<SeesawRegisters>(getAuxDisplayFunctionsBase())),
_displayItself(memory<DisplayRegisters>(getDisplayFunctionsBase())),
_rtcBase(memory<RTCInterface>(getRTCBase())),
openFiles(new SDFile*[_sdbase.maximumNumberOfOpenFilesPort]),
colorBlack_(computeColor565(0,0,0)){
    uint32_t sdCardFileBase = getSDCardFileBase();
    for (int i = 0; i < _sdbase.maximumNumberOfOpenFilesPort; ++i, sdCardFileBase += 0x100) {
        openFiles[i] = new SDFile(sdCardFileBase);
    }
    now();
}


uint16_t
ChipsetBasicFunctions::read() const {
    return _memory.consoleIOPort;
}

void
ChipsetBasicFunctions::write(uint16_t c) {
    _memory.consoleIOPort = c;
}

void
ChipsetBasicFunctions::write(char c) {
    write(static_cast<uint16_t>(c));
}
void
ChipsetBasicFunctions::flush() {
    // doesn't matter what you write as long as you write it
    _memory.consoleFlushPort = 1;
}
void
ChipsetBasicFunctions::write(const char* ptr) {
    write(const_cast<char*>(ptr), strlen(ptr));
}

void
ChipsetBasicFunctions::writeLine() {
    write('\n');
}
void
ChipsetBasicFunctions::writeLine(const char* ptr) {
    write(ptr);
    writeLine();
}


ChipsetBasicFunctions&
getBasicChipsetInterface() {
    static ChipsetBasicFunctions theLed;
    return theLed;
}

ssize_t
ChipsetBasicFunctions::write(char *buffer, size_t nbyte) {
    // unlike reading, we must be sequential in writing
    ssize_t numWritten = 0;
    for (size_t i = 0; i < nbyte; ++i) {
        _memory.consoleIOPort = static_cast<uint16_t>(static_cast<unsigned char>(buffer[i]));
        ++numWritten;
    }
    flush();
    return numWritten;
}
uint16_t
ChipsetBasicFunctions::waitForLegalCharacter() {
    uint16_t rawConsoleValue = _memory.consoleIOPort;
    while (rawConsoleValue == 0xFFFF) {
        rawConsoleValue = _memory.consoleIOPort;
    }
    return rawConsoleValue;
}
ssize_t
ChipsetBasicFunctions::read(char *buffer, size_t nbyte) {
    ssize_t numRead = 0;
    for (size_t i = 0; i < nbyte; ++i) {
        buffer[i] = static_cast<char>(waitForLegalCharacter());
        ++numRead;
        if ((buffer[i] == '\n') || (buffer[i] == '\r')) {
            return numRead;
        }
    }
    return numRead;
}




void
ChipsetBasicFunctions::triggerInt0() {
    // force the chipset to trigger an interrupt
    _memory.triggerInt0Port = 1;
}
bool
ChipsetBasicFunctions::addressDebuggingEnabled() const {
    return _memory.addressDebuggingFlag != 0;
}
void
ChipsetBasicFunctions::enableAddressDebugging() {
    // upper 16-bits are ignored on the chipset side but who cares
    _memory.addressDebuggingFlag = 0xFFFFFFFF;
}
void
ChipsetBasicFunctions::disableAddressDebugging() {
    _memory.addressDebuggingFlag = 0;
}

ChipsetBasicFunctions::~ChipsetBasicFunctions() {
    for (int i = 0; i < _sdbase.maximumNumberOfOpenFilesPort; ++i) {
        if (openFiles[i]->isOpen()) {
            openFiles[i]->close();
        }
        delete openFiles[i];
    }
    delete [] openFiles;
}

int
ChipsetBasicFunctions::openFile(const char *path, int flags, int mode) {
    int len = strlen(path);
    if (len > 80) {
        errno = EBADF;
        return -1;
    } else {
        // transfer the path
        int i = 0;
        for (; i < len; ++i) {
            _sdbase.path[i] = path[i];
        }
        // zero out the rest of the path
        for (; i < 80; ++i) {
            _sdbase.path[i] = 0;
        }
        // decode i960 permissions into the appropriate registers for translation sake
        _sdbase.filePermissions = 0; // clear previous permissions out
        _sdbase.openWriteOnly = mode & O_WRONLY;
        _sdbase.openReadOnly = mode & O_RDONLY;
        _sdbase.openReadWrite = mode & O_RDWR;
        _sdbase.clearFileContentsOnOpen = flags & O_TRUNC;
        _sdbase.createFileIfMissing = flags & O_CREAT;
        // make the chipset look for an open file handle that we can use
        return _sdbase.openPort;
    }
}
bool
ChipsetBasicFunctions::readFile(int fd, void *buf, size_t sz, int &nread) {
    SDFile* file = openFiles[fd];
    if (!file->isOpen()) {
        return false;
    } else {
        /// @todo implement
        char* buffer = reinterpret_cast<char*>(buf) ;
        for (size_t i = 0; i < sz; ++i) {
            uint16_t result = file->getChar();
            if (result == 0xFFFF) {
                // EOF reached
                break;
            } else {
                ++nread;
                buffer[i] = static_cast<char>(result);
            }
        }
        return true;
    }
}

bool
ChipsetBasicFunctions::writeFile(int fd, const void *buf, size_t sz, int &nwrite) {
    SDFile* file = openFiles[fd];
    if (!file->isOpen()) {
        return false;
    } else {
        /// @todo implement
        const char* buffer = reinterpret_cast<const char*>(buf) ;
        for (size_t i = 0; i < sz; ++i) {
            file->putChar(buffer[i]);
            if (file->getWriteError()) {
                break;
            } else {
                ++nwrite;
            }
        }
        return true;
    }
}
bool
ChipsetBasicFunctions::closeFile(int fd) {
    SDFile* file = openFiles[fd];
    if (!file->isOpen()) {
        return false;
    } else {
        file->close();
        return true;
    }
}

void
displayIOMemoryMap() {
    printf("Serial 0 base address 0x%lx\n", getChipsetRegistersBase());
    printf("SD ctl base address 0x%lx\n", getSDCardRegisterBase());
    printf("SD file begin address 0x%lx\n", getSDCardFileBase());
    printf("Display Aux begin address 0x%lx\n", getAuxDisplayFunctionsBase());
    printf("Display primary begin address 0x%lx\n", getDisplayFunctionsBase());
    printf("RTC begin address 0x%lx\n", getRTCBase());
}

void
ChipsetBasicFunctions::setBacklightIntensity(uint16_t value) {
    _displayAux.backlight = value;
}

uint16_t
ChipsetBasicFunctions::getBacklightIntensity() const {
    return _displayAux.backlight;
}


uint32_t
ChipsetBasicFunctions::unixtime() const {
    // request a time update
    now();
    return _rtcBase.unixtime;
}

uint32_t
ChipsetBasicFunctions::timesince2000() const {
    now();
    return _rtcBase.secondsSince2000;
}

void
ChipsetBasicFunctions::now() const {
    _rtcBase.nowRequest = 0;
}

uint16_t
ChipsetBasicFunctions::color565(uint32_t color) {
    return color565(static_cast<uint8_t>(color),
                    static_cast<uint8_t>(color >> 8),
                    static_cast<uint8_t>(color >> 16)) ;
}
void
ChipsetBasicFunctions::installInstruction0(const RawDisplayInstruction &instruction, Opcodes opcode) {
    _displayItself.instructionLower0 = instruction.quadFields_[0];
    _displayItself.instructionUpper0 = instruction.quadFields_[1];
    _displayItself.invoke0 = static_cast<uint16_t>(opcode);
}

void
ChipsetBasicFunctions::installInstruction0_FirstQuadField(const RawDisplayInstruction &instruction, Opcodes opcode) {
    _displayItself.instructionLower0 = instruction.quadFields_[0];
    _displayItself.invoke0 = static_cast<uint16_t>(opcode);
}

void
ChipsetBasicFunctions::fillScreen(uint16_t value) {
    RawDisplayInstruction rdi(value, 0, 0, 0, 0, 0, 0, 0);
    installInstruction0_FirstQuadField(rdi, InvokeOpcodes_FillScreen);
}
void
ChipsetBasicFunctions::clearScreen() {
    fillScreen(colorBlack_);
}




void
ChipsetBasicFunctions::drawRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t fgColor, bool fill) {
    RawDisplayInstruction rdi(x, y, width, height, fgColor, 0, 0, 0);
    installInstruction0(rdi, fill ? InvokeOpcodes_FillRect : InvokeOpcodes_DrawRect) ;
}
void
ChipsetBasicFunctions::drawCircle(uint16_t x, uint16_t y, uint16_t radius, uint16_t fgColor, bool fill) {
    RawDisplayInstruction rdi(x, y, radius, fgColor, 0, 0, 0, 0);
    installInstruction0_FirstQuadField(rdi, fill ? InvokeOpcodes_FillCircle : InvokeOpcodes_DrawCircle) ;
}
void
ChipsetBasicFunctions::drawRoundedRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t radius, uint16_t fgColor, bool fill ) {
    RawDisplayInstruction rdi(x, y, width, height, radius, fgColor, 0, 0);
    installInstruction0(rdi, fill ? InvokeOpcodes_FillRoundRect : InvokeOpcodes_DrawRoundRect) ;

}
void ChipsetBasicFunctions::drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t fgColor) {
    RawDisplayInstruction rdi(x0, y0, x1, y1, fgColor, 0, 0, 0);
    installInstruction0(rdi, InvokeOpcodes_DrawLine);
}
void ChipsetBasicFunctions::drawVerticalLine(uint16_t x, uint16_t y, uint16_t height, uint16_t fgColor) {
    RawDisplayInstruction rdi(x, y, height, fgColor, 0, 0, 0, 0);
    installInstruction0_FirstQuadField(rdi, InvokeOpcodes_DrawFastVLine);
}
void ChipsetBasicFunctions::drawHorizontalLine(uint16_t x, uint16_t y, uint16_t width, uint16_t fgColor) {
    RawDisplayInstruction rdi(x, y, width, fgColor, 0, 0, 0, 0);
    installInstruction0_FirstQuadField(rdi, InvokeOpcodes_DrawFastHLine);
}

void
ChipsetBasicFunctions::drawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t fgColor, bool fill) {
    RawDisplayInstruction rdi(x0, y0, x1, y1, x2, y2, fgColor, 0);
    installInstruction0(rdi, fill ? InvokeOpcodes_FillTriangle : InvokeOpcodes_DrawTriangle);
}

void
ChipsetBasicFunctions::drawPixel(uint16_t x, uint16_t y, uint16_t color) {
    RawDisplayInstruction rdi(x, y, color, 0, 0, 0, 0, 0);
    installInstruction0_FirstQuadField(rdi, InvokeOpcodes_DrawPixel);
}
