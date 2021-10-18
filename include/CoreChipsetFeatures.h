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
// Created by jwscoggins on 6/21/21.
//

#ifndef I960SXCHIPSET_CORECHIPSETFEATURES_H
#define I960SXCHIPSET_CORECHIPSETFEATURES_H
#include "ProcessorSerializer.h"
#include "SDCardInterface.h"
#include "DisplayInterface.h"
#include "EEPROMInterface.h"
#include "Serial0Interface.h"
template<bool overrideExternalDeviceLocations,
        Address Serial0Base,
        Address SDCardBase,
        Address DisplayBase,
        Address EEPROMBase,
        byte maximumNumberOfOpenFiles = 16>
class CoreChipsetFeatures /* : public IOSpaceThing */ {
public:
    static constexpr auto MaximumNumberOfOpenFiles = maximumNumberOfOpenFiles;
    static constexpr Address IOBaseAddress = 0xFE00'0000;
    static constexpr SplitWord32 IOBaseSplit { IOBaseAddress };
    static constexpr byte SectionID = IOBaseSplit.getMostSignificantByte();
    // each one of these 256 byte pages have a prescribed start and end
    static constexpr Address IOConfigurationSpaceStart = IOBaseAddress;
    static constexpr Address IOConfigurationSpaceEnd = IOConfigurationSpaceStart + (16 * 0x100);
    // then start our initial designs after this point
    static constexpr Address RegisterPage0BaseAddress = overrideExternalDeviceLocations ? Serial0Base : IOConfigurationSpaceEnd;
    static constexpr Address RegisterPage0EndAddress = RegisterPage0BaseAddress + 0x100;
    static constexpr SplitWord32 Serial0BaseAddress {RegisterPage0BaseAddress};
    static constexpr byte Serial0Page = Serial0BaseAddress.getTargetPage();
    using SDInterface = SDCardInterface<MaximumNumberOfOpenFiles,
                                        overrideExternalDeviceLocations ? SDCardBase : RegisterPage0EndAddress>;
    //static constexpr auto SDCardInterfaceBaseAddress = SDInterface :: StartAddress;
    //static constexpr auto SDCardInterfaceEndAddress = SDInterface :: EndAddress;
    using DisplayInterface = ::DisplayInterface<overrideExternalDeviceLocations ? DisplayBase : SDInterface::EndAddress>;
    using EEPROMInterface = ::EEPROMInterface<overrideExternalDeviceLocations ? EEPROMBase : DisplayInterface::EndAddress>;
    // we have a bunch of pages in here that are useful :)
    enum class IOConfigurationSpace0Registers : uint8_t {
#define TwoByteEntry(Prefix) Prefix ## 0, Prefix ## 1
#define FourByteEntry(Prefix) \
        TwoByteEntry(Prefix ## 0), \
        TwoByteEntry(Prefix ## 1)

        FourByteEntry(Serial0BaseAddress),
        FourByteEntry(SDCardInterfaceBaseAddress),
        FourByteEntry(SDCardFileBlock0BaseAddress),
        FourByteEntry(DisplayShieldBaseAddress),
        FourByteEntry(ST7735DisplayBaseAddress),
        FourByteEntry(EEPROMBaseAddress),
#undef FourByteEntry
#undef TwoByteEntry
        End,
        Serial0BaseAddressLower = Serial0BaseAddress00,
        Serial0BaseAddressUpper = Serial0BaseAddress10,
        SDCardInterfaceBaseAddressLower = SDCardInterfaceBaseAddress00,
        SDCardInterfaceBaseAddressUpper = SDCardInterfaceBaseAddress10,
        SDCardFileBlock0BaseAddressLower = SDCardFileBlock0BaseAddress00,
        SDCardFileBlock0BaseAddressUpper = SDCardFileBlock0BaseAddress10,
        DisplayShieldBaseAddressLower = DisplayShieldBaseAddress00,
        DisplayShieldBaseAddressUpper = DisplayShieldBaseAddress10,
        ST7735DisplayBaseAddressLower = ST7735DisplayBaseAddress00,
        ST7735DisplayBaseAddressUpper = ST7735DisplayBaseAddress10,
        EEPROMBaseAddressLower = EEPROMBaseAddress00,
        EEPROMBaseAddressUpper = EEPROMBaseAddress10,
    };
    enum class IOConfigurationSpace1Registers : uint8_t {
#define TwoByteEntry(Prefix) Prefix ## 0, Prefix ## 1
#define FourByteEntry(Prefix) \
        TwoByteEntry(Prefix ## 0), \
        TwoByteEntry(Prefix ## 1)
        FourByteEntry(EEPROMSize),
#undef FourByteEntry
#undef TwoByteEntry
        End,
        EEPROMSizeLower = EEPROMSize00,
        EEPROMSizeUpper = EEPROMSize10,
    };


public:
    CoreChipsetFeatures() = delete;
    ~CoreChipsetFeatures() = delete;
    CoreChipsetFeatures(const CoreChipsetFeatures&) = delete;
    CoreChipsetFeatures(CoreChipsetFeatures&&) = delete;
    CoreChipsetFeatures& operator=(const CoreChipsetFeatures&) = delete;
    CoreChipsetFeatures& operator=(CoreChipsetFeatures&&) = delete;
    static void begin() noexcept {
        DisplayInterface::begin();
        SDInterface::begin();
        EEPROMInterface::begin();
    }
private:
    static uint16_t readIOConfigurationSpace0(uint8_t offset, LoadStoreStyle) noexcept {
        switch (static_cast<IOConfigurationSpace0Registers>(offset)) {
#define X(title, var) \
             case IOConfigurationSpace0Registers:: title ## Lower : return static_cast<uint16_t>(var); \
             case IOConfigurationSpace0Registers:: title ## Upper : return static_cast<uint16_t>(var >> 16)
            X(Serial0BaseAddress, RegisterPage0BaseAddress);
            X(SDCardInterfaceBaseAddress, SDInterface::ControlBaseAddress);
            X(SDCardFileBlock0BaseAddress, SDInterface::FilesBaseAddress);
            X(DisplayShieldBaseAddress, DisplayInterface::SeesawSectionStart);
            X(ST7735DisplayBaseAddress, DisplayInterface::DisplaySectionStart);
#undef X

            default: return 0; // zero is never an io page!
        }
    }

    static uint16_t readIOConfigurationSpace1(uint8_t offset, LoadStoreStyle) noexcept {
        switch (static_cast<IOConfigurationSpace1Registers>(offset)) {
#define X(title, var) \
             case IOConfigurationSpace1Registers:: title ## Lower : return static_cast<uint16_t>(var); \
             case IOConfigurationSpace1Registers:: title ## Upper : return static_cast<uint16_t>(var >> 16)
            X(EEPROMSize, EEPROMInterface::Size);
#undef X

            default: return 0; // zero is never an io page!
        }
    }
public:
    [[nodiscard]] static uint16_t read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept {
        // force override the default implementation
        if (targetPage == 0) {
            return readIOConfigurationSpace0(offset, lss);
        } else if (targetPage == 1) {
            return readIOConfigurationSpace1(offset, lss);
        } else if (targetPage == Serial0Page) {
            return handleFirstPageRegisterReads(offset, lss);
        } else {
            if constexpr (!overrideExternalDeviceLocations) {
                if (SDInterface::respondsTo(targetPage)) {
                    return SDInterface::read(targetPage, offset, lss);
                } else if (DisplayInterface::respondsTo(targetPage)) {
                    return DisplayInterface::read(targetPage, offset, lss);
                } else if (EEPROMInterface::respondsTo(targetPage)) {
                    return EEPROMInterface::read(targetPage, offset, lss);
                } else {
                    return 0;
                }
            } else {
                return 0;
            }
        }
    }
    static void write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
        if (targetPage == Serial0Page) {
        } else {
            if constexpr (!overrideExternalDeviceLocations) {
                if (SDInterface::respondsTo(targetPage)) {
                    SDInterface::write(targetPage, offset, lss, value);
                } else if (DisplayInterface::respondsTo(targetPage)) {
                    DisplayInterface::write(targetPage, offset, lss, value);
                } else if (EEPROMInterface::respondsTo(targetPage)) {
                    EEPROMInterface::write(targetPage, offset, lss, value);
                } else {
                    // do nothing
                }
            }
        }
    }
};
#endif //I960SXCHIPSET_CORECHIPSETFEATURES_H
