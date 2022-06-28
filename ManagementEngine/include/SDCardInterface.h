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
// Created by jwscoggins on 10/10/21.
//

#ifndef SXCHIPSET_SDCARDINTERFACE_H
#define SXCHIPSET_SDCARDINTERFACE_H
#include "Pinout.h"
#include "OpenFileHandle.h"
#include <SdFat.h>
extern SdFat SD;
template<Address maxFiles, Address startAddress>
class SDCardInterface {
public:
    static constexpr auto MaximumNumberOfOpenFiles = maxFiles;
    static constexpr auto StartAddress = startAddress;
    static constexpr auto ControlBaseAddress = startAddress;
    static constexpr auto ControlEndAddress = ControlBaseAddress + 0x100;
    static constexpr auto FilesBaseAddress = ControlEndAddress;
    static constexpr auto FilesEndAddress = FilesBaseAddress + (MaximumNumberOfOpenFiles * 0x100);
    static constexpr auto EndAddress = FilesEndAddress;
    static constexpr SplitWord32 StartAddressDecomposition {StartAddress};
    static constexpr SplitWord32 EndAddressDecomposition {EndAddress};
    static constexpr SplitWord32 CTLAddress {StartAddress};
    static constexpr SplitWord32 DecomposedFilesStart{FilesBaseAddress};
    static constexpr SplitWord32 DecomposedFilesEnd {FilesEndAddress};
    static constexpr auto StartPage = StartAddressDecomposition.getTargetPage();
    static constexpr auto EndPage = EndAddressDecomposition.getTargetPage();
    static constexpr auto CTLPage = CTLAddress.getTargetPage();
    static constexpr auto FileStartPage = DecomposedFilesStart.getTargetPage();
    static constexpr auto FileEndPage = DecomposedFilesEnd.getTargetPage();
    static constexpr auto SectionID = StartAddressDecomposition.getMostSignificantByte();
    enum class SDCardFileSystemRegisters : uint8_t {
#define TwoByteEntry(Prefix) Prefix ## 0, Prefix ## 1
#define FourByteEntry(Prefix) \
        TwoByteEntry(Prefix ## 0), \
        TwoByteEntry(Prefix ## 1)
#define EightByteEntry(Prefix) \
        FourByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define TwelveByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define SixteenByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        EightByteEntry(Prefix ## 1)
        SixteenByteEntry(Path0),
        SixteenByteEntry(Path1),
        SixteenByteEntry(Path2),
        SixteenByteEntry(Path3),
        SixteenByteEntry(Path4),
        TwoByteEntry(OpenPort),
        TwoByteEntry(MakeDirectoryPort),
        TwoByteEntry(ExistsPort),
        TwoByteEntry(RemovePort),
        FourByteEntry(SDClusterCount),
        FourByteEntry(SDVolumeSectorCount),
        TwoByteEntry(SDBytesPerSector),
        TwoByteEntry(NumberOfOpenFiles),
        TwoByteEntry(MaximumNumberOfOpenFiles),
        TwoByteEntry(ErrorCode),
        TwoByteEntry(MakeMissingParentDirectories),
        TwoByteEntry(FilePermissions), // raw interface
        TwoByteEntry(OpenReadWrite), // O_READ | O_WRITE
        TwoByteEntry(OpenReadOnly), // O_READ
        TwoByteEntry(OpenWriteOnly), // O_WRITE
        TwoByteEntry(CreateFileIfMissing), // O_CREAT
        TwoByteEntry(ClearFileContentsOnOpen), // O_TRUNC
        TwoByteEntry(MountCTL), // controls mount/unmount functionality when writing and reading it yields the status

#undef SixteenByteEntry
#undef TwelveByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
        End,
        PathStart = Path00000,
        PathEnd = Path41111,
        OpenPort = OpenPort0,
        MakeDirectoryPort = MakeDirectoryPort0,
        ExistsPort = ExistsPort0,
        RemovePort = RemovePort0,
        SDClusterCountLower = SDClusterCount00,
        SDClusterCountUpper = SDClusterCount10,
        SDClusterCount = SDClusterCountLower,
        SDVolumeSectorCountLower = SDVolumeSectorCount00,
        SDVolumeSectorCountUpper = SDVolumeSectorCount10,
        SDVolumeSectorCount = SDVolumeSectorCountLower,
        SDBytesPerSector = SDBytesPerSector0,
        NumberOfOpenFiles = NumberOfOpenFiles0,
        MaximumNumberOfOpenFiles = MaximumNumberOfOpenFiles0,
        ErrorCode = ErrorCode0,
        MakeMissingParentDirectories = MakeMissingParentDirectories0,
        OpenReadWrite = OpenReadWrite0,
        OpenReadOnly = OpenReadOnly0,
        OpenWriteOnly = OpenWriteOnly0,
        CreateFileIfMissing = CreateFileIfMissing0,
        ClearFileContentsOnOpen = ClearFileContentsOnOpen0,
        FilePermissions = FilePermissions0,
        MountCTL = MountCTL0,
        // we ignore the upper half of the register but reserve it to make sure
    };
    SDCardInterface() = delete;
    ~SDCardInterface() = delete;
private:
    ///@todo make it possible to unmount the sdcard while the i960 is running
    static void unmountSDCard() noexcept {
        if (cardMounted_) {
            // first close all open files
            for (auto &file: files_) {
                if (file.isOpen()) {
                    // flush everything in progress
                    file.flush();
                    file.close();
                }
            }
            // according to my research this should be enough
            cardMounted_ = false;
        }
    }
    /**
     * @brief Try to mount/remount the primary SDCard
     * @return
     */
    static auto tryMountSDCard() noexcept {
        if (!cardMounted_) {
            cardMounted_ = SD.begin(static_cast<int>(i960Pinout::SD_EN));
        }
        return cardMounted_;
    }
    static uint16_t findFreeFile() noexcept {
        for (uint16_t i = 0; i < MaximumNumberOfOpenFiles; ++i) {
            if (!files_[i].isOpen()) {
                return i;
            }
        }
        return 0xFFFF;
    }
    static uint16_t tryOpenFile() noexcept {
        if (numberOfOpenFiles_ < MaximumNumberOfOpenFiles) {
            // when we open a new file we have to make sure that we are less than the number of open files
            // But we also need to keep track of proper indexes as well. This is a two layer process
            auto newId = findFreeFile();
            auto& targetFile = files_[newId];
            if (targetFile.open(sdCardPath_, filePermissions_)) {
                ++numberOfOpenFiles_;
                return newId;
            } else {
                /// @todo set appropriate error condition for bad file open
            }
        } else {
            /// @todo set appropriate error condition for too many open files
        }
        return -1;
    }
    static bool tryMakeDirectory(bool makeMissingParents = false) noexcept { return SD.mkdir(sdCardPath_, makeMissingParents); }
    static bool exists() noexcept { return SD.exists(sdCardPath_); }
    static bool remove() noexcept { return SD.remove(sdCardPath_); }
    static uint16_t ctlRead16(uint8_t offset) noexcept {
        if (offset < 80) {
            SplitWord16 result(0);
            result.bytes[0] = sdCardPath_[offset];
            result.bytes[1] = sdCardPath_[offset + 1];
            return result.getWholeValue();
        } else {
            using T = SDCardFileSystemRegisters;
            switch (static_cast<T>(offset)) {
                case T::OpenPort:
                    return tryOpenFile();
                case T::MakeDirectoryPort:
                    return tryMakeDirectory(makeMissingParentDirectories_);
                case T::ExistsPort:
                    return exists();
                case T::RemovePort:
                    return remove();
                case T::SDBytesPerSector:
                    return bytesPerSector_;
                case T::MaximumNumberOfOpenFiles:
                    return MaximumNumberOfOpenFiles;
                case T::NumberOfOpenFiles:
                    return numberOfOpenFiles_;
                case T::MakeMissingParentDirectories:
                    return makeMissingParentDirectories_;
                case T::FilePermissions:
                    return filePermissions_;
                case T::MountCTL:
                    return cardMounted_ ? 0xFFFF : 0;
                default:
                    return 0;
            }
        }
    }
    static uint32_t ctlRead32(uint8_t offset) noexcept {
        if (offset < 80) {
            SplitWord32 result(0);
            for (uint8_t i = offset, j = 0; i < 80; ++i, ++j) {
                result.bytes[j] = sdCardPath_[i];
            }
            return result.getWholeValue();
        } else {
            using T = SDCardFileSystemRegisters;
            switch (static_cast<T>(offset)) {
                case T::SDClusterCount:
                    return clusterCount_.getWholeValue();
                case T::SDVolumeSectorCount:
                    return volumeSectorCount_.getWholeValue();
                default:
                    return 0;
            }
        }
    }
    static void ctlWrite16(uint8_t offset, uint16_t value) noexcept {
        // write16 will always be aligned
        if (offset < 80) {
            sdCardPath_[offset] = static_cast<char>(value);
            sdCardPath_[offset + 1] = static_cast<char>(value >> 8);
        } else {
            using T = SDCardFileSystemRegisters;
            switch (static_cast<T>(offset)) {
                case T::MakeMissingParentDirectories:
                    makeMissingParentDirectories_ = value != 0;
                    break;
                case T::FilePermissions:
                    filePermissions_ = value;
                    break;
                case T::OpenReadWrite:
                    if (value != 0) {
                        filePermissions_ |= O_RDWR;
                    }
                    break;
                case T::OpenReadOnly:
                    if (value != 0) {
                        filePermissions_ |= O_RDONLY;
                    }
                    break;
                case T::OpenWriteOnly:
                    if (value != 0) {
                        filePermissions_ |= O_WRITE;
                    }
                    break;
                case T::CreateFileIfMissing:
                    if (value != 0) {
                        filePermissions_ |= O_CREAT;
                    }
                    break;
                case T::ClearFileContentsOnOpen:
                    if (value != 0) {
                        filePermissions_ |= O_TRUNC;
                    }
                    break;
                case T::MountCTL:
                    // 0 means unmount,
                    // 1 means mount
                    // other values are ignored
                    if (value == 0) {
                        // unmount
                        unmountSDCard();
                    } else if (value == 1) {
                        // mount
                        (void)tryMountSDCard();
                    }
                    break;
                default:
                    break;
            }
        }
    }
    static void ctlWrite8(uint8_t offset, uint8_t value) noexcept {
        if (offset < 80) {
            sdCardPath_[offset] = static_cast<char>(value);
        } else {
            // if you're doing byte by byte writes, I don't now what that means. So I'm just going to state that you have to have
            // BE0 and BE1 low to perform proper execution
        }
    }
    static void ctlWrite32(uint8_t offset, uint32_t value) noexcept {
        if (offset <= 76) {
            sdCardPath_[offset]  = static_cast<char>(value);
            sdCardPath_[offset+1]  = static_cast<char>(value >> 8);
            sdCardPath_[offset+2]  = static_cast<char>(value >> 16);
            sdCardPath_[offset+3]  = static_cast<char>(value >> 24);
        } else {
           //  don't allow 32-bit writes beyond the character section
        }
    }
    static uint16_t fileRead16(uint8_t index, uint8_t offset) noexcept { return files_[index].read16(offset); }
    static uint32_t fileRead32(uint8_t index, uint8_t offset) noexcept { return files_[index].read32(offset); }
    static void fileWrite8(uint8_t index, uint8_t offset, uint8_t value) noexcept { files_[index].write8(offset, value); }
    static void fileWrite16(uint8_t index, uint8_t offset, uint16_t value) noexcept { files_[index].write16(offset, value); }
    static void fileWrite32(uint8_t index, uint8_t offset, uint32_t value) noexcept { files_[index].write32(offset, value); }
public:
    static constexpr bool respondsTo(byte targetPage) noexcept {
        return targetPage >= StartPage && targetPage < EndPage;
    }
    static void begin() noexcept {
        if (!initialized_) {
            while (!tryMountSDCard()) {
                Serial.println(F("SD CARD INIT FAILED...WILL RETRY SOON"));
                delay(1000);
            }
            Serial.println(F("SD CARD UP!"));
            clusterCount_ = SplitWord32(SD.clusterCount());
            // if we use SdFat class then we have to do some work ourselves
            // we only know how many sectors per cluster and the number of clusters so we multiply them together
            // to get the volume sector count
            volumeSectorCount_ = SplitWord32{SD.sectorsPerCluster() * SD.clusterCount() };
            // to get the bytes per sector, we get the number of bytes per cluster and divide it by the number of sectors in a cluster
            // this will yield the number of bytes per sector.
            bytesPerSector_ = SD.bytesPerCluster() / SD.sectorsPerCluster();
            initialized_ = true;
        }
    }
    static uint32_t read32(uint32_t address) noexcept {
        if (auto targetPage = static_cast<uint8_t>(address >> 8), offset = static_cast<uint8_t>(address); targetPage == CTLPage) {
            return ctlRead32(offset);
        } else if (targetPage >= FileStartPage && targetPage < FileEndPage) {
            return fileRead32(targetPage - FileStartPage, offset);
        } else {
            return 0;
        }
    }
    static uint16_t read16(uint32_t address) noexcept {
        if (auto targetPage = static_cast<uint8_t>(address >> 8), offset = static_cast<uint8_t>(address); targetPage == CTLPage) {
            return ctlRead16(offset);
        } else if (targetPage >= FileStartPage && targetPage < FileEndPage) {
            return fileRead16(targetPage - FileStartPage, offset);
        } else {
            return 0;
        }
    }
public:
    static void write8(uint32_t address, uint8_t value) noexcept {
        // do nothing
        auto targetOffset = static_cast<byte>(address);
        if (auto targetPage = static_cast<byte>(address >> 8); targetPage == CTLPage) {
            ctlWrite8(targetOffset, value);
        } else if (targetPage >= FileStartPage && targetPage < FileEndPage) {
            fileWrite8(targetPage - FileStartPage, targetOffset, value);
        } else {

        }
    }
    static void write16(uint32_t address, uint16_t value) noexcept {
        auto targetOffset = static_cast<byte>(address);
        if (auto targetPage = static_cast<byte>(address >> 8); targetPage == CTLPage) {
            ctlWrite16(targetOffset, value);
        } else if (targetPage >= FileStartPage && targetPage < FileEndPage) {
            fileWrite16(targetPage - FileStartPage, targetOffset, value);
        } else {

        }

        // do nothing
    }
    static void write32(uint32_t address, uint32_t value) noexcept {
        auto targetOffset = static_cast<byte>(address);
        if (auto targetPage = static_cast<byte>(address >> 8); targetPage == CTLPage) {
            ctlWrite32(targetOffset, value);
        } else if (targetPage >= FileStartPage && targetPage < FileEndPage) {
            fileWrite32(targetPage - FileStartPage, targetOffset, value);
        } else {

        }
    }
private:
    static inline bool initialized_ = false;
    static inline SplitWord32 clusterCount_ {0};
    static inline SplitWord32 volumeSectorCount_ {0};
    static inline uint16_t bytesPerSector_ = 0;
    static inline uint16_t numberOfOpenFiles_ = 0;
    static inline char sdCardPath_[81] = { 0 };
    static inline OpenFileHandle files_[MaximumNumberOfOpenFiles];
    static inline bool makeMissingParentDirectories_ = false;
    static inline uint16_t filePermissions_ = 0;
    static inline bool cardMounted_ = false;
};
#endif //SXCHIPSET_SDCARDINTERFACE_H
