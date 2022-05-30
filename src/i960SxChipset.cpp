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
#include "DisplayInterface.h"
#include "CoreChipsetFeatures.h"
#include "SDCardAsRam.h"
#include "TaggedCacheAddress.h"
#include "RTCInterface.h"
#include "i960SxChipset.h"
#include "type_traits.h"

constexpr auto RTCBaseAddress = 0xFA00'0000;
constexpr auto Serial0BaseAddress = 0xFB00'0000;
constexpr auto DisplayBaseAddress = 0xFC00'0000;
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
using TheDisplayInterface = DisplayInterface<DisplayBaseAddress>;
using TheSDInterface = SDCardInterface<MaximumNumberOfOpenFiles, SDBaseAddress>;
using TheConsoleInterface = Serial0Interface<Serial0BaseAddress, CompileInAddressDebuggingSupport, AddressDebuggingEnabledOnStartup>;
using TheRTCInterface = RTCInterface<RTCBaseAddress>;
using ConfigurationSpace = CoreChipsetFeatures<TheConsoleInterface,
        TheSDInterface,
        TheDisplayInterface,
        TheRTCInterface>;
// define the backing memory storage classes via template specialization
// at this point in time, if no specialization is performed, use SDCard as ram backend
using FallbackMemory = SDCardAsRam<TheSDInterface >;
template<TargetMCU mcu> struct BackingMemoryStorage final { using Type = FallbackMemory; };

using BackingMemoryStorage_t = BackingMemoryStorage<TargetBoard::getMCUTarget()>::Type;
constexpr auto CacheLineSize = TargetBoard::getCacheLineSizeInBits();
constexpr auto CacheSize = TargetBoard::getCacheSize();

//using L1Cache = CacheInstance_t<EightWayRandPLRUCacheSet, CacheSize, CacheLineSize, BackingMemoryStorage_t, CompileInCacheSystemDebuggingSupport>;
//using L1Cache = CacheInstance_t<SixteenWayRandPLRUCacheWay, CacheSize, CacheLineSize, BackingMemoryStorage_t, CompileInCacheSystemDebuggingSupport>;
//using L1Cache = Cache2Instance_t<FourteenWayRandPLRUCacheWay, 128, CacheLineSize, BackingMemoryStorage_t, CompileInAddressDebuggingSupport>;
using L1Cache = Cache2Instance_t<TenWayRandPLRUCacheWay, 256, CacheLineSize, BackingMemoryStorage_t, CompileInAddressDebuggingSupport>;

L1Cache theCache;

void waitForCycleUnlock() noexcept {
    while (DigitalPin<i960Pinout::DoCycle>::isDeasserted());
}
[[nodiscard]] bool informCPU() noexcept {
    // don't pulse READY, instead just pull it low, the interrupt latency on the 4809 is horrible
    // so we just pull Ready high as soon as we get the next phase in.
    //DigitalPin<i960Pinout::Ready>::pulse();
    DigitalPin<i960Pinout::Ready>::assertPin();
    // make sure that we just wait for the gating signal before continuing
    while (DigitalPin<i960Pinout::InTransaction>::isAsserted() && DigitalPin<i960Pinout::BurstNext>::isDeasserted());
    bool outcome = DigitalPin<i960Pinout::InTransaction>::isDeasserted();
    DigitalPin<i960Pinout::Ready>::deassertPin();
    return outcome;
}
constexpr auto IncrementAddress = true;
constexpr auto LeaveAddressAlone = false;
// while the i960 does not allow going beyond 8 words, we can use the number of words cached in all cases to be safe
constexpr byte MaximumNumberOfWordsTransferrableInASingleTransaction = decltype(theCache)::NumWordsCached;
void displayRequestedAddress() noexcept {
    auto address = ProcessorInterface::getAddress();
    Serial.print(F("ADDRESS: 0x"));
    Serial.println(address, HEX);
}

template<bool inDebugMode>
void fallbackBodyRead() noexcept {
    if constexpr (inDebugMode) {
        displayRequestedAddress();
    }
    ProcessorInterface::setupDataLinesForRead();
    do {
        // wait for
        waitForCycleUnlock();
        ProcessorInterface::setDataBits(0);
        if (informCPU()) {
            break;
        }
        ProcessorInterface::burstNext<LeaveAddressAlone>();
    } while (true);
}
template<bool inDebugMode>
void fallbackBodyWrite() noexcept {
    if constexpr (inDebugMode) {
        displayRequestedAddress();
    }
    ProcessorInterface::setupDataLinesForWrite();
    do {
        // wait for
        waitForCycleUnlock();
        // get the data bits but do nothing with it just to delay things
        (void)ProcessorInterface::getDataBits();
        if (informCPU()) {
            break;
        }
        ProcessorInterface::burstNext<LeaveAddressAlone>();
    } while (true);
}
template<bool inDebugMode>
void fallbackBody() noexcept {
    // fallback, be consistent to make sure we don't run faster than the i960
    if (ProcessorInterface::isReadOperation()) {
        fallbackBodyRead<inDebugMode>();
    } else {
        fallbackBodyWrite<inDebugMode>();
    }
}
template<bool inDebugMode>
void handleMemoryInterfaceRead() noexcept {
    if constexpr (inDebugMode) {
        displayRequestedAddress();
    }
    auto start = ProcessorInterface::getCacheOffsetEntry<decltype(theCache)::CacheEntryMask>();
    auto end = start + 8;
    auto& theEntry = theCache.getLine();
    // when dealing with read operations, we can actually easily unroll the do while by starting at the cache offset entry and walking
    // forward until we either hit the end of the cache line or blast is asserted first (both are valid states)
    for (auto i = start; i < end; ++i) {
        // start working on getting the given value way ahead of cycle unlock happening
        waitForCycleUnlock();
        if constexpr (inDebugMode && CompileInExtendedDebugInformation) {
            auto outcome = theEntry.get(i);
            Serial.print(F("\tOffset: 0x")) ;
            Serial.print(i << 1, HEX);
            Serial.print(F(" (")) ;
            Serial.print(i);
            Serial.print(F("),  Read the value: 0x"));
            Serial.println(outcome, HEX);
            ProcessorInterface::setDataBits(outcome);
        } else {
            ProcessorInterface::setDataBits(theEntry.get(i));
        }
        // Only pay for what we need even if it is slower
        if (informCPU()) {
            break;
        }
        // so if I don't increment the address, I think we run too fast xD based on some experimentation
        ProcessorInterface::burstNext<LeaveAddressAlone>();
    }
}
template<bool inDebugMode>
void handleMemoryInterfaceWrite() noexcept {
    if constexpr (inDebugMode) {
        displayRequestedAddress();
    }
    auto start = ProcessorInterface::getCacheOffsetEntry<decltype(theCache)::CacheEntryMask>();
    auto end = start + 8;
    auto& theEntry = theCache.getLine();
    // when dealing with writes to the cache line we are safe in just looping through from the start to at most 8 because that is as
    // far as we can go with how the Sx works!

    // Also the manual states that the processor cannot burst across 16-byte boundaries so :D.
    for (auto i = start; i < end; ++i) {
        waitForCycleUnlock();
        if constexpr (inDebugMode && CompileInExtendedDebugInformation) {
            auto bits = ProcessorInterface::getDataBits();
            Serial.print(F("\tOffset: 0x")) ;
            Serial.print(i << 1, HEX);
            Serial.print(F(" (")) ;
            Serial.print(i);
            Serial.print(F("),  Writing the value: 0x"));
            Serial.println(bits.getWholeValue(), HEX);
            theEntry.set(i, ProcessorInterface::getStyle(), bits);
        } else {

            theEntry.set(i, ProcessorInterface::getStyle(), ProcessorInterface::getDataBits());
        }
        if (informCPU()) {
            break;
        }
        // the manual doesn't state that the burst transaction will always have BE0 and BE1 pulled low and this is very true, you must
        // check the pins because it will do unaligned burst transactions but even that will never span multiple 16-byte entries
        // so if I don't increment the address, I think we run too fast xD based on some experimentation
        ProcessorInterface::burstNext<LeaveAddressAlone>();
    }
}
template<bool inDebugMode>
void handleMemoryInterface() noexcept {
    // okay we are dealing with the psram chips
    // now take the time to compute the cache offset entries
    if (ProcessorInterface::isReadOperation()) {
        ProcessorInterface::setupDataLinesForRead();
        handleMemoryInterfaceRead<inDebugMode>();
    } else {
        ProcessorInterface::setupDataLinesForWrite();
        handleMemoryInterfaceWrite<inDebugMode>();
    }
}
template<bool inDebugMode, typename T>
void handleExternalDeviceRequestRead() noexcept {
    if constexpr (inDebugMode) {
        displayRequestedAddress();
    }
    for(;;) {
        waitForCycleUnlock();
        if constexpr (inDebugMode && TargetBoard::compileInExtendedDebugInformation()) {
            auto result = T::read(ProcessorInterface::getPageIndex(),
                                  ProcessorInterface::getPageOffset(),
                                  ProcessorInterface::getStyle());
            Serial.print(F("\tPage Index: 0x")) ;
            Serial.println(ProcessorInterface::getPageIndex(), HEX);
            Serial.print(F("\tPage Offset: 0x")) ;
            Serial.println(ProcessorInterface::getPageOffset(), HEX);
            Serial.print(F("\tRead Value: 0x"));
            Serial.println(result, HEX);
            ProcessorInterface::setDataBits(result);
        } else {
            ProcessorInterface::setDataBits(
                    T::read(ProcessorInterface::getPageIndex(),
                            ProcessorInterface::getPageOffset(),
                            ProcessorInterface::getStyle()));

        }
        if (informCPU()) {
            break;
        }
        ProcessorInterface::burstNext<IncrementAddress>();
    }
}
template<bool inDebugMode, typename T>
void handleExternalDeviceRequestWrite() noexcept {
    if constexpr (inDebugMode) {
        displayRequestedAddress();
    }
    for (;;) {
        waitForCycleUnlock();
        if constexpr (inDebugMode && CompileInExtendedDebugInformation) {
            auto dataBits = ProcessorInterface::getDataBits();
            Serial.print(F("\tPage Index: 0x")) ;
            Serial.println(ProcessorInterface::getPageIndex(), HEX);
            Serial.print(F("\tPage Offset: 0x")) ;
            Serial.println(ProcessorInterface::getPageOffset(), HEX);
            Serial.print(F("\tData To Write: 0x"));
            Serial.println(dataBits.getWholeValue(), HEX);
            T::write(ProcessorInterface::getPageIndex(),
                     ProcessorInterface::getPageOffset(),
                     ProcessorInterface::getStyle(),
                     dataBits);
        } else {
            T::write(ProcessorInterface::getPageIndex(),
                     ProcessorInterface::getPageOffset(),
                     ProcessorInterface::getStyle(),
                     ProcessorInterface::getDataBits());
        }
        if (informCPU()) {
            break;
        }
        // be careful of querying i960 state at this point because the chipset runs at twice the frequency of the i960
        // so you may still be reading the previous i960 cycle state!
        ProcessorInterface::burstNext<IncrementAddress>();
    }
}
template<bool inDebugMode, typename T>
void handleExternalDeviceRequest() noexcept {
    // with burst transactions in the core chipset, we do not have access to a cache line to write into.
    // instead we need to do the old style infinite iteration design
    if (ProcessorInterface::isReadOperation()) {
        ProcessorInterface::setupDataLinesForRead();
        handleExternalDeviceRequestRead<inDebugMode, T>();
    } else {
        ProcessorInterface::setupDataLinesForWrite();
        handleExternalDeviceRequestWrite<inDebugMode, T>();
    }
}
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
    if constexpr (TargetBoard::compileInAddressDebuggingSupport()) {
        if (TheConsoleInterface::addressDebuggingEnabled())  {
            invocationBody<true>();
        } else {
            invocationBody<false>();
        }
    } else {
        invocationBody<false>();
    }
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
        static constexpr auto CacheSize = theCache.getCacheSize();
        //static constexpr auto CacheSize = ::CacheSize;
        auto *storage = theCache.viewAsStorage();
        if constexpr (ValidateTransferDuringInstall) {
            static constexpr auto RealCacheSize = CacheSize / 2;
            byte* storage0 = storage;
            byte* storage1 = storage + (RealCacheSize);
            for (Address addr = 0; addr < size; addr += RealCacheSize) {
                // do a linear read from the start to the end of storage
                // wait around to make sure we don't run afoul of the sdcard itself
                while (theFile.isBusy());
                auto numRead = theFile.read(storage0, RealCacheSize);
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
        theCache.clear();
    }
}

using DispatchTable = BodyFunction[256];
using DualDispatchTable = SplitBodyFunction[256];
DispatchTable lookupTable;
DualDispatchTable lookupTableSplit;
DualDispatchTable lookupTableSplit_Debug;
DispatchTable lookupTable_Debug;
void setupDispatchTable() noexcept {
    Serial.println(F("Setting up the initial lookup table"));
    auto defaultFallback = std::make_tuple(fallbackBodyRead<false>, fallbackBodyWrite<false>);
    for (int i = 0; i < 256; ++i) {
        lookupTable[i] = fallbackBody<false>;
        lookupTableSplit[i] = defaultFallback;
    }
    // since this uses SD card as memory, just increase the size of it to 1 gigabyte
    // 64 * 16 => 64 sixteen megabyte sections
    for (int i = 0; i < 64; ++i) {
        lookupTable[i] = handleMemoryInterface<false>;
        lookupTableSplit[i] = std::make_tuple(handleMemoryInterfaceRead<false>, handleMemoryInterfaceWrite<false>);
    }
    lookupTable[TheRTCInterface::SectionID] = handleExternalDeviceRequest<false, TheRTCInterface>;
    lookupTableSplit[TheRTCInterface::SectionID] = std::make_tuple(handleExternalDeviceRequestRead<false, TheRTCInterface>, handleExternalDeviceRequestWrite<false, TheRTCInterface>);

    lookupTable[TheDisplayInterface::SectionID] = handleExternalDeviceRequest<false, TheDisplayInterface>;
    lookupTableSplit[TheDisplayInterface::SectionID] = std::make_tuple(handleExternalDeviceRequestRead<false, TheDisplayInterface>, handleExternalDeviceRequestWrite<false, TheDisplayInterface>);

    lookupTable[TheSDInterface::SectionID] = handleExternalDeviceRequest<false, TheSDInterface>;
    lookupTableSplit[TheSDInterface::SectionID] = std::make_tuple(handleExternalDeviceRequestRead<false, TheSDInterface>, handleExternalDeviceRequestWrite<false, TheSDInterface>);

    lookupTable[TheConsoleInterface::SectionID] = handleExternalDeviceRequest<false, TheConsoleInterface>;
    lookupTableSplit[TheConsoleInterface::SectionID] = std::make_tuple(handleExternalDeviceRequestRead<false, TheConsoleInterface>, handleExternalDeviceRequestWrite<false, TheConsoleInterface>);

    lookupTable[ConfigurationSpace::SectionID] = handleExternalDeviceRequest<false, ConfigurationSpace>;
    lookupTableSplit[ConfigurationSpace::SectionID] = std::make_tuple(handleExternalDeviceRequestRead<false, ConfigurationSpace>, handleExternalDeviceRequestWrite<false, ConfigurationSpace>);
    if constexpr (TargetBoard::compileInAddressDebuggingSupport()) {
        auto defaultFallback_Debug = std::make_tuple(fallbackBodyRead<true>, fallbackBodyWrite<true>);
        for (int i = 0; i < 256; ++i) {
            lookupTable_Debug[i] = fallbackBody<true>;
            lookupTableSplit_Debug[i] = defaultFallback_Debug;
        }
        // since this uses SD card as memory, just increase the size of it to 1 gigabyte
        // 64 * 16 => 64 sixteen megabyte sections
        for (int i = 0; i < 64; ++i) {
            lookupTable_Debug[i] = handleMemoryInterface<true>;
            lookupTableSplit_Debug[i] = std::make_tuple(handleMemoryInterfaceRead<true>, handleMemoryInterfaceWrite<true>);
        }
        lookupTable_Debug[TheRTCInterface::SectionID] = handleExternalDeviceRequest<true, TheRTCInterface>;
        lookupTableSplit_Debug[TheRTCInterface::SectionID] = std::make_tuple(handleExternalDeviceRequestRead<true, TheRTCInterface>, handleExternalDeviceRequestWrite<true, TheRTCInterface>);

        lookupTable_Debug[TheDisplayInterface::SectionID] = handleExternalDeviceRequest<true, TheDisplayInterface>;
        lookupTableSplit_Debug[TheDisplayInterface::SectionID] = std::make_tuple(handleExternalDeviceRequestRead<true, TheDisplayInterface>, handleExternalDeviceRequestWrite<true, TheDisplayInterface>);

        lookupTable_Debug[TheSDInterface::SectionID] = handleExternalDeviceRequest<true, TheSDInterface>;
        lookupTableSplit_Debug[TheSDInterface::SectionID] = std::make_tuple(handleExternalDeviceRequestRead<true, TheSDInterface>, handleExternalDeviceRequestWrite<true, TheSDInterface>);

        lookupTable_Debug[TheConsoleInterface::SectionID] = handleExternalDeviceRequest<true, TheConsoleInterface>;
        lookupTableSplit_Debug[TheConsoleInterface::SectionID] = std::make_tuple(handleExternalDeviceRequestRead<true, TheConsoleInterface>, handleExternalDeviceRequestWrite<true, TheConsoleInterface>);

        lookupTable_Debug[ConfigurationSpace::SectionID] = handleExternalDeviceRequest<true, ConfigurationSpace>;
        lookupTableSplit_Debug[ConfigurationSpace::SectionID] = std::make_tuple(handleExternalDeviceRequestRead<true, ConfigurationSpace>, handleExternalDeviceRequestWrite<true, ConfigurationSpace>);
    }
}

void waitForBootSignal() noexcept {
    while (DigitalPin<i960Pinout::SuccessfulBoot>::read() == LOW);
    attachInterrupt(i960Pinout::SuccessfulBoot,
                    []() { signalHaltState("CHECKSUM FAILURE"); },
                    FALLING);
}
// the setup routine runs once when you press reset:
void setup() {
    // by resetting the 4809 we are resetting the entire system
    DigitalPin<i960Pinout::Reset4809>::configure();
    DigitalPin<i960Pinout::Reset4809>::assertPin();
    // Reset960 is waitboot960 in type 3.00 but in 3.01 it is actually used a chipset booted pin instead
    DigitalPin<i960Pinout::Reset960>::configure();
#ifndef CHIPSET300
    DigitalPin<i960Pinout::Reset960>::deassertPin();
#else
    DigitalPin<i960Pinout::Reset960>::assertPin();
#endif

    // always do this first to make sure that we put the i960 into reset regardless of target
    // make sure that the 4809 has enough time and also make sure that the i960 has enough time to undegrade itself!
    delay(1);
    DigitalPin<i960Pinout::Reset4809>::deassertPin();
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    // seed random on startup to be on the safe side from analog pin A0, A1, A2, and A3
    randomSeed(analogRead(A0) + analogRead(A1) + analogRead(A2) + analogRead(A3));
    // before we do anything else, configure as many pins as possible and then
    // pull the i960 into a reset state, it will remain this for the entire
    // duration of the setup function
    // get SPI setup ahead of time
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
            i960Pinout::INT_EN1,
            i960Pinout::INT_EN2,
            i960Pinout::INT_EN3,
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
            i960Pinout::MUXADR8,
            i960Pinout::MUXADR9,
            i960Pinout::MUXADR10,
            i960Pinout::MUXADR11,
            i960Pinout::MUXADR12,
            i960Pinout::MUXADR13,
            i960Pinout::MUXADR14,
            i960Pinout::MUXADR15,
            i960Pinout::MUXSel0
            >();
    // all of these pins need to be pulled high
    DigitalPin<i960Pinout::SD_EN>::deassertPin();
    DigitalPin<i960Pinout::Ready>::deassertPin();
    DigitalPin<i960Pinout::GPIOSelect>::deassertPin();
    DigitalPin<i960Pinout::MUXSel0>::deassertPin();
    // setup the pins that could be attached to an io expander separately
    theCache.begin();
    // purge the cache pages
    ConfigurationSpace::begin();
    Serial.println(F("i960Sx chipset bringup"));
    ProcessorInterface::begin();
    BackingMemoryStorage_t::begin();
    setupDispatchTable();
    installBootImage();
    delay(100);
    Serial.println(F("i960Sx chipset brought up fully!"));
#ifndef CHIPSET300
    DigitalPin<i960Pinout::Reset960>::assertPin();
#else
    DigitalPin<i960Pinout::Reset960>::deassertPin();
#endif

    ProcessorInterface::setupDataLinesForRead();
    waitForBootSignal();
}
// ----------------------------------------------------------------
// state machine
// ----------------------------------------------------------------
// The bootup process has a separate set of states
// TStart - Where we start
// TSystemTest - Processor performs self test
//
// TStart -> TSystemTest via FAIL being asserted
// TSystemTest -> Ti via FAIL being deasserted
//
// State machine will stay here for the duration
// State diagram based off of i960SA/SB Reference manual
// Basic Bus States
// Ti - Idle State
// Ta - Address State
// Td - Data State
// Tw - Wait State

// READY - ~READY asserted
// NOT READY - ~READY not asserted
// BURST - ~BLAST not asserted
// NO BURST - ~BLAST asserted
// NEW REQUEST - ~AS asserted
// NO REQUEST - ~AS not asserted when in

// Ti -> Ti via no request
// Ti -> Ta via new request
// on enter of Ta, set address state to false
// on enter of Td, burst is sampled
// Ta -> Td
// Td -> Ti after signaling ready and no burst (blast low)
// Td -> Td after signaling ready and burst (blast high)
// Ti -> TChecksumFailure if FAIL is asserted
// NOTE: Tw may turn out to be synthetic

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
#ifdef __arm__
[[noreturn]]
void
signalHaltState(const std::string& haltMsg) noexcept {
    signalHaltState(haltMsg.c_str());
}
#endif
BodyFunction getNonDebugBody(byte index) noexcept {
    return lookupTable[index];
}
BodyFunction getDebugBody(byte index) noexcept {
    if constexpr (CompileInAddressDebuggingSupport) {
        return lookupTable_Debug[index];
    } else {
        return fallbackBody<true>;
    }
}

SplitBodyFunction getSplitNonDebugBody(byte index) noexcept {
    return lookupTableSplit[index];
}
SplitBodyFunction getSplitDebugBody(byte index) noexcept {
    static constexpr SplitBodyFunction fallback(fallbackBodyRead<true>, fallbackBodyWrite<true>);
    if constexpr (CompileInAddressDebuggingSupport) {
        return lookupTableSplit_Debug[index];
    } else {
        return fallback;
    }
}
SdFat SD;
