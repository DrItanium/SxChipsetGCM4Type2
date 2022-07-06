/*
SxChipset_ManagementEngine
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

/// i960Sx chipset mcu, based on atmega1284p with fuses set for:
/// - 20Mhz crystal
/// - D1 acts as CLKO
/// Language options:
/// - C++17
/// Board Platform: MightyCore
#include <SPI.h>
#include <Wire.h>
#include <SdFat.h>
#include <tuple>
#include "Pinout.h"
#include <memory>

#include "CacheEntry.h"
#include "SetAssociativeRandPLRUCacheSets.h"
#include "SinglePoolCache.h"

#include "ProcessorInterface.h"
#include "CoreChipsetFeatures.h"
#include "SDCardAsRam.h"
#include "TaggedCacheAddress.h"
#include "i960SxChipset.h"
#include <type_traits>
#include "ManagementEngine.h"
#include "RAM.h"
#include "SinglePageMemorySpace.h"
#include "SPIMemorySpace.h"
constexpr auto RTCBaseAddress = 0xFA00'0000;
constexpr auto Serial0BaseAddress = 0xFB00'0000;
constexpr auto SDBaseAddress = 0xFD00'0000;
constexpr auto MaximumNumberOfOpenFiles = 256;
constexpr auto CompileInAddressDebuggingSupport = TargetBoard::compileInAddressDebuggingSupport();
constexpr auto AddressDebuggingEnabledOnStartup = TargetBoard::addressDebuggingEnabledOnStartup();
constexpr auto CompileInCacheSystemDebuggingSupport = TargetBoard::compileInCacheSystemDebuggingSupport();
constexpr auto CompileInExtendedDebugInformation = TargetBoard::compileInExtendedDebugInformation();
constexpr auto ValidateTransferDuringInstall = TargetBoard::validateTransferDuringInstall();
/**
 * @brief When set to true, the interrupt lines the mcp23s17 provides are used to determine which bytes to read
 */
constexpr auto UseIOExpanderAddressLineInterrupts = TargetBoard::useIOExpanderAddressLineInterrupts();
//using TheDisplayInterface = DisplayInterface<DisplayBaseAddress>;
using TheSDInterface = SDCardInterface<MaximumNumberOfOpenFiles>;
//using TheConsoleInterface = Serial0Interface<Serial0BaseAddress, CompileInAddressDebuggingSupport, AddressDebuggingEnabledOnStartup>;
using UART0Interface = Serial0Interface;
using ConfigurationSpace = CoreChipsetFeatures;
// define the backing memory storage classes via template specialization
// at this point in time, if no specialization is performed, use SDCard as ram backend
using FallbackMemory = SDCardAsRam<TheSDInterface >;
template<TargetMCU mcu> struct BackingMemoryStorage final { using Type = FallbackMemory; };

using BackingMemoryStorage_t = BackingMemoryStorage<TargetBoard::getMCUTarget()>::Type;
constexpr auto CacheLineSize = TargetBoard::getCacheLineSizeInBits();
constexpr auto CacheSize = TargetBoard::getCacheSize();
using SystemRam_t = RAM<BackingMemoryStorage_t>;
CompleteMemorySpace fullSpace;
SystemRam_t theRAM;
CoreChipsetFeatures configurationSpace;
UART0Interface uart0;
SPIMemorySpace spi0;
TheSDInterface sdcard_(SD);
Uart Serial2(&SERCOM_SERIAL2, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);
Uart Serial3(&SERCOM_SERIAL3, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX, PAD_SERIAL3_TX);
Uart Serial4(&SERCOM_SERIAL4, PIN_SERIAL4_RX, PIN_SERIAL4_TX, PAD_SERIAL4_RX, PAD_SERIAL4_TX);
auto& UnoUart = Serial1;
auto& PicUart = Serial4;
auto& Feather0Uart = Serial2;
auto& Feather1Uart = Serial3;
//NullMemorySpace null_;

template<bool inDebugMode>
void invocationBody() noexcept {
    // wait for the management engine to give the go ahead
    while (DigitalPin<i960Pinout::InTransaction>::isDeasserted());

    // keep processing data requests until we
    // when we do the transition, record the information we need
    // there are only two parts to this code, either we map into ram or chipset functions
    // we can just check if we are in ram, otherwise it is considered to be chipset. This means that everything not ram is chipset
    // and so we are actually continually mirroring the mapping for the sake of simplicity
    ProcessorInterface::newDataCycle<inDebugMode>();
}
void doInvocationBody() noexcept {
    invocationBody<false>();
}

void installBootImage() noexcept {

    // okay now we need to actually open boot.system and copy it into the ramBlock
    if (!SD.exists(const_cast<char*>("boot.sys"))) {
        // delete the file and start a new
        signalHaltState(F("Could not find file \"boot.sys\"!"));
    }
    if (auto theFile = SD.open("boot.sys", FILE_READ); !theFile) {
        signalHaltState(F("Could not open \"boot.sys\"! SD CARD may be corrupt?")) ;
    } else {
            // okay we were successful in opening the file, now copy the image into psram
        Address size = theFile.size();
        Serial.println(F("TRANSFERRING BOOT.SYS TO RAM"));
        constexpr auto cacheSize = theRAM.getCacheSize();
        //static constexpr auto CacheSize = ::CacheSize;
        auto *storage = theRAM.viewCacheAsStorage();
        if constexpr (ValidateTransferDuringInstall) {
            auto realCacheSize = cacheSize / 2;
            byte* storage0 = storage;
            byte* storage1 = storage + (realCacheSize);
            for (Address addr = 0; addr < size; addr += realCacheSize) {
                // do a linear read from the start to the end of storage
                // wait around to make sure we don't run afoul of the sdcard itself
                while (theFile.isBusy());
                auto numRead = theFile.read(storage0, realCacheSize);
                if (numRead < 0) {
                    // something wen't wrong so halt at this point
                    SD.errorHalt();
                }
                (void) BackingMemoryStorage_t::write(addr, storage0, numRead);
                (void) BackingMemoryStorage_t::read(addr, storage1, numRead);
                // now read back the contents into the second buffer
                for (auto i = 0; i < numRead; ++i) {
                    auto a = storage0[i];
                    auto b = storage1[i];
                    if (a != b) {
                        Serial.print(F("MISMATCH WANTED 0x"));
                        Serial.print(a, HEX);
                        Serial.print(F(" BUT GOT 0x"));
                        Serial.println(b, HEX);
                    }
                }

                Serial.print(F("."));
            }
        } else {
            // use the cache as a buffer since it won't be in use at this point in time
            for (Address addr = 0; addr < size; addr += CacheSize) {
                // do a linear read from the start to the end of storage
                // wait around to make sure we don't run afoul of the sdcard itself
                while (theFile.isBusy());
                auto numRead = theFile.read(storage, CacheSize);
                if (numRead < 0) {
                    // something wen't wrong so halt at this point
                    SD.errorHalt();
                }
                (void) BackingMemoryStorage_t::write(addr, storage, numRead);
                // now read back the contents into the upper half
                Serial.print(F("."));
            }
        }
        Serial.println();
        Serial.println(F("Transfer complete!"));
        // make sure we close the file before destruction
        theFile.close();
        // clear both caches to be on the safe side
        theRAM.clear();
    }
}


// the setup routine runs once when you press reset:
void setupMemoryMap();
void
setupDataLines() {
    DigitalPin<i960Pinout::Data0>::configure();
    DigitalPin<i960Pinout::Data1>::configure();
    DigitalPin<i960Pinout::Data2>::configure();
    DigitalPin<i960Pinout::Data3>::configure();
    DigitalPin<i960Pinout::Data4>::configure();
    DigitalPin<i960Pinout::Data5>::configure();
    DigitalPin<i960Pinout::Data6>::configure();
    DigitalPin<i960Pinout::Data7>::configure();
    DigitalPin<i960Pinout::Data8>::configure();
    DigitalPin<i960Pinout::Data9>::configure();
    DigitalPin<i960Pinout::Data10>::configure();
    DigitalPin<i960Pinout::Data11>::configure();
    DigitalPin<i960Pinout::Data12>::configure();
    DigitalPin<i960Pinout::Data13>::configure();
    DigitalPin<i960Pinout::Data14>::configure();
    DigitalPin<i960Pinout::Data15>::configure();
}
void setupMux() noexcept {
    DigitalPin<i960Pinout::MUXADR0>::configure();
    DigitalPin<i960Pinout::MUXADR1>::configure();
    DigitalPin<i960Pinout::MUXADR2>::configure();
    DigitalPin<i960Pinout::MUXADR3>::configure();
    DigitalPin<i960Pinout::MUXADR4>::configure();
    DigitalPin<i960Pinout::MUXADR5>::configure();
    DigitalPin<i960Pinout::MUXADR6>::configure();
    DigitalPin<i960Pinout::MUXADR7>::configure();
    DigitalPin<i960Pinout::MUXSel0>::configure(); DigitalPin<i960Pinout::MUXSel0>::deassertPin();
    DigitalPin<i960Pinout::MUXSel1>::configure(); DigitalPin<i960Pinout::MUXSel1>::deassertPin();
    DigitalPin<i960Pinout::MUXSel2>::configure(); DigitalPin<i960Pinout::MUXSel2>::deassertPin();
    DigitalPin<i960Pinout::MUX_EN>::configure(); DigitalPin<i960Pinout::MUX_EN>::assertPin();
}
void setup() {
    ManagementEngine::configure();
    ManagementEngine::holdInReset();
    ManagementEngine::chipsetIsInSetup();
    // always do this first to make sure that we put the i960 into reset regardless of target
    // make sure that the 4809 has enough time and also make sure that the i960 has enough time to undegrade itself!
    // setup random
    // seed random on startup to be on the safe side from analog pin A0, A1, A2, and A3
    // before we do anything else, configure as many pins as possible and then
    // pull the i960 into a reset state, it will remain this for the entire
    // duration of the setup function
    // get SPI setup ahead of time
    randomSeed(analogRead(A0) + analogRead(A1) + analogRead(A2) + analogRead(A3));
    // put in a 1-millisecond delay to be on the safe side
    delay(1);
    ManagementEngine::allowBoot();
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    UnoUart.begin(115200);
    Feather0Uart.begin(115200);
    Feather1Uart.begin(115200);
    PicUart.begin(115200);

    SPI.begin();
    Wire.begin();
    DigitalPin<i960Pinout::SD_EN>::configure(); DigitalPin<i960Pinout::SD_EN>::deassertPin();
    DigitalPin<i960Pinout::Ready>::configure(); DigitalPin<i960Pinout::Ready>::deassertPin();
    DigitalPin<i960Pinout::GPIOSelect>::configure(); DigitalPin<i960Pinout::GPIOSelect>::deassertPin();
    DigitalPin<i960Pinout::INT_EN0>::configure(); DigitalPin<i960Pinout::INT_EN0>::deassertPin();
    DigitalPin<i960Pinout::Feather0_INT>::configure();
    DigitalPin<i960Pinout::Feather1_INT>::configure();
    DigitalPin<i960Pinout::BusHold>::configure(); DigitalPin<i960Pinout::BusHold>::deassertPin();
    DigitalPin<i960Pinout::BusHold_Acknowledge>::configure();
    setupMux();
    setupDataLines();
    // all of these pins need to be pulled high
    // setup the pins that could be attached to an io expander separately
    //theCache.begin();
    // purge the cache pages
    Serial.println(F("i960Sx chipset bringup"));
    sdcard_.begin();
    ProcessorInterface::begin();
    BackingMemoryStorage_t::begin();
    setupMemoryMap();
    //installBootImage();
    delay(100);
    Serial.println(F("i960Sx chipset brought up fully!"));
    ManagementEngine::chipsetReady();
    ProcessorInterface::setupDataLinesForRead();
    Serial.println(F("Waiting for boot signal"));
    ManagementEngine::waitForBootSignal();
    Serial.println(F("System Booted!"));
}

void loop() {
    doInvocationBody();
}

[[noreturn]]
void
signalHaltState(const __FlashStringHelper* haltMsg) noexcept {
    Serial.print(F("CHIPSET HALT: "));
    Serial.println(haltMsg);
    while(true) {
        delay(1000);
    }
}
[[noreturn]]
void
signalHaltState(const char* haltMsg) noexcept {
    Serial.print("CHIPSET HALT: ");
    Serial.println(haltMsg);
    while(true) {
        delay(1000);
    }
}
[[noreturn]]
void
signalHaltState(const std::string& haltMsg) noexcept {
    signalHaltState(haltMsg.c_str());
}

void
setupMemoryMap() {
    Serial.println(F("Setting up memory map"));
    static constexpr uint32_t mmioBaseAddress = 0xFF00'0000;
    static constexpr uint32_t chipsetDevicesStart = mmioBaseAddress + 0xF0'0000;
    static constexpr uint32_t ramStart = 0x0000'0000;
    auto memory = map(ramStart, theRAM);
    /// @todo define more items here
    auto sdcardInterface = map(0xFE00'0000, sdcard_);
    // originally the sdcard file interface was designed to be placed independently of the ctl register
    // however, now it is placed contiguously
    auto sdcardFileInterface = map(0xFE00'0100, sdcard_);
    auto configSpaceMapping = map(chipsetDevicesStart + 0xF'0000, configurationSpace);
    auto serial = map(chipsetDevicesStart + 0xE'FF00, uart0);
    auto spi = map(chipsetDevicesStart + 0xE'FE00, spi0);
    // null_ is a fake device so just define it
    //auto auxDisplayInterface = map(chipsetDevicesStart + 0xE'FD00, null_);
    //auto displayInterface = map(chipsetDevicesStart + 0xE'FC00, null_);
    //auto rtcInterface = map(chipsetDevicesStart + 0xE'FB00, null_);
    //auto gpioInterface = map(chipsetDevicesStart + 0xA'0000, null_);
    //auto twiInterface = map(chipsetDevicesStart + 0xA'0000, null_);
    // make sure that the layout
    configurationSpace.addDevice(serial);
    configurationSpace.addDevice(sdcardInterface);
    // this is a bit of a hack because previously I was assuming that the file interface could be anywhere!
    // but in reality it is not important
    configurationSpace.addDevice(sdcardFileInterface);
    configurationSpace.addDevice(0, 0) ; // Aux display
    configurationSpace.addDevice(0, 0); // Display
    configurationSpace.addDevice(0, 0); //RTC
    configurationSpace.addDevice(0, 0); // SPI
    configurationSpace.addDevice(0, 0); // GPIO
    configurationSpace.addDevice(0, 0); // TWI
    fullSpace.emplace_back(memory);
    fullSpace.emplace_back(configSpaceMapping);
    fullSpace.emplace_back(serial);
    fullSpace.emplace_back(sdcardInterface);
    /// @todo add more items to full space here
    Serial.println(F("Done setting up memory map!"));
}
MemorySpace&
getMemory() noexcept {
    return fullSpace;
}
SdFat SD;
