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
#include <SdFat.h>
#include <tuple>
#include "Pinout.h"

#include "CacheEntry.h"
#include "SetAssociativeRandPLRUCacheSets.h"
#include "SinglePoolCache.h"

#include "ProcessorSerializer.h"
#include "CoreChipsetFeatures.h"
#include "SDCardAsRam.h"
#include "TaggedCacheAddress.h"
#include "RTCInterface.h"
#include "i960SxChipset.h"
#include <type_traits>
#include "ManagementEngine.h"
#include "RAM.h"
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
using TheSDInterface = SDCardInterface<MaximumNumberOfOpenFiles, SDBaseAddress>;
//using TheConsoleInterface = Serial0Interface<Serial0BaseAddress, CompileInAddressDebuggingSupport, AddressDebuggingEnabledOnStartup>;
using UART0Interface = Serial0Interface;
using TheRTCInterface = RTCInterface<RTCBaseAddress>;
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
UART0Interface uart0{0xFFFE'FF00};

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
constexpr auto TestReadyPinSignal = false;
void setupMemoryMap();
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
    SPI.begin();
    Wire.begin();
    configurePins<
            i960Pinout::SD_EN,
            i960Pinout::Ready,
            i960Pinout::GPIOSelect,
            i960Pinout::BE0,
            i960Pinout::BE1,
            i960Pinout::W_R_,
            i960Pinout::SuccessfulBoot,
            i960Pinout::INT_EN0,
            i960Pinout::InTransaction,
            i960Pinout::DoCycle,
            i960Pinout::BurstNext,
            i960Pinout::Data0,
            i960Pinout::Data1,
            i960Pinout::Data2,
            i960Pinout::Data3,
            i960Pinout::Data4,
            i960Pinout::Data5,
            i960Pinout::Data6,
            i960Pinout::Data7,
            i960Pinout::Data8,
            i960Pinout::Data9,
            i960Pinout::Data10,
            i960Pinout::Data11,
            i960Pinout::Data12,
            i960Pinout::Data13,
            i960Pinout::Data14,
            i960Pinout::Data15,
            i960Pinout::MUXADR0,
            i960Pinout::MUXADR1,
            i960Pinout::MUXADR2,
            i960Pinout::MUXADR3,
            i960Pinout::MUXADR4,
            i960Pinout::MUXADR5,
            i960Pinout::MUXADR6,
            i960Pinout::MUXADR7,
            i960Pinout::MUXSel1,
            i960Pinout::MUXSel2,
            i960Pinout::MUX_EN,
            i960Pinout::MUXSel0
            >();
    // all of these pins need to be pulled high
    DigitalPin<i960Pinout::SD_EN>::deassertPin();
    DigitalPin<i960Pinout::Ready>::deassertPin();
    DigitalPin<i960Pinout::GPIOSelect>::deassertPin();
    DigitalPin<i960Pinout::MUXSel0>::deassertPin();
    if constexpr (TestReadyPinSignal) {
        Serial.println("TEST READY SIGNAL PIN MODE");
        while (true) {
            Serial.println("ASSERT READY!");
            DigitalPin<i960Pinout::Ready>::assertPin();
            delay(1000);
            Serial.println("DEASSERT READY!");
            DigitalPin<i960Pinout::Ready>::deassertPin();
            delay(1000);
        }
    }
    // setup the pins that could be attached to an io expander separately
    //theCache.begin();
    // purge the cache pages
    Serial.println(F("i960Sx chipset bringup"));
    ProcessorInterface::begin();
    BackingMemoryStorage_t::begin();
    setupMemoryMap();
    installBootImage();
    delay(100);
    Serial.println(F("i960Sx chipset brought up fully!"));
    ManagementEngine::chipsetReady();
    ProcessorInterface::setupDataLinesForRead();
    ManagementEngine::waitForBootSignal();
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
    fullSpace.emplace_back(theRAM);
    fullSpace.emplace_back(configurationSpace);
    fullSpace.emplace_back(uart0);
    configurationSpace.addDevice(uart0);
    /// @todo implement
}
MemorySpace::ObserverPtr
getMemory() noexcept {
    static MemorySpace::ObserverPtr space;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        space = std::experimental::make_observer(&fullSpace);
    }
    return space;
}
SdFat SD;
