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
#include "MemorySpace.h"
#include <SdFat.h>
template<Address maxFiles>
class SDCardInterface : public SizedMemorySpace {
public:
    static constexpr auto MaximumNumberOfOpenFiles = maxFiles;
    static constexpr auto CTLPage = 0;
    static constexpr auto FileStartPage = CTLPage + 1;
    static constexpr auto FileEndPage = FileStartPage + MaximumNumberOfOpenFiles;
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
        SDVolumeSectorCountLower = SDVolumeSectorCount00,
        SDVolumeSectorCountUpper = SDVolumeSectorCount10,
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
public:
    using Parent = SizedMemorySpace;
    using Self = SDCardInterface<maxFiles>;
    explicit SDCardInterface(SdFat& backingStore) : Parent(MaximumNumberOfOpenFiles + 1 ), backingStore_(backingStore) { }
    explicit SDCardInterface() : Self(SD) { }
    ~SDCardInterface() override = default;
private:
    ///@todo make it possible to unmount the sdcard while the i960 is running
    void unmountSDCard() noexcept {
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
    auto tryMountSDCard() noexcept {
        if (!cardMounted_) {
            cardMounted_ = backingStore_.begin(static_cast<int>(i960Pinout::SD_EN));
        }
        return cardMounted_;
    }
    uint16_t findFreeFile() noexcept {
        for (uint16_t i = 0; i < MaximumNumberOfOpenFiles; ++i) {
            if (!files_[i].isOpen()) {
                return i;
            }
        }
        return 0xFFFF;
    }
    uint16_t tryOpenFile() noexcept {
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
    bool tryMakeDirectory(bool makeMissingParents = false) noexcept { return backingStore_.mkdir(sdCardPath_, makeMissingParents); }
    bool exists() noexcept { return backingStore_.exists(sdCardPath_); }
    bool remove() noexcept { return backingStore_.remove(sdCardPath_); }
    uint16_t ctlRead(uint8_t offset, LoadStoreStyle lss) noexcept {
        if (offset < 80) {
            if (auto result = SplitWord16(reinterpret_cast<uint16_t*>(sdCardPath_)[offset >> 1]); lss == LoadStoreStyle::Upper8) {
                return result.bytes[1];
            } else if (lss == LoadStoreStyle::Lower8) {
                return result.bytes[0];
            } else {
                return result.getWholeValue();
            }
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
                case T::SDClusterCountLower:
                    return clusterCount_.halves[0];
                case T::SDClusterCountUpper:
                    return clusterCount_.halves[1];
                case T::SDVolumeSectorCountLower:
                    return volumeSectorCount_.halves[0];
                case T::SDVolumeSectorCountUpper:
                    return volumeSectorCount_.halves[1];
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
    struct TreatAs16Bit { using StorageType = uint16_t; };
    struct TreatAs32Bit { using StorageType = uint32_t; };
    struct TreatAs8Bit { using StorageType = uint8_t; };
    auto fileRead(uint8_t index, uint8_t offset, TreatAs8Bit) noexcept { return files_[index].read8(offset); }
    auto fileRead(uint8_t index, uint8_t offset, TreatAs16Bit) noexcept { return files_[index].read16(offset); }
    auto fileRead(uint8_t index, uint8_t offset, TreatAs32Bit) noexcept { return files_[index].read32(offset); }
    void fileWrite(uint8_t index, uint8_t offset, uint8_t value, TreatAs8Bit) noexcept { files_[index].write8(offset, value); }
    void fileWrite(uint8_t index, uint8_t offset, uint16_t value, TreatAs16Bit) noexcept { files_[index].write16(offset, value); }
    void fileWrite(uint8_t index, uint8_t offset, uint32_t value, TreatAs32Bit) noexcept { files_[index].write32(offset, value); }
    void ctlWrite(uint8_t offset, uint8_t value, TreatAs8Bit) noexcept {
        if (offset < 80) {
            sdCardPath_[offset] = static_cast<char>(value);
        } else {
            // do nothing
        }
    }
    void ctlWrite(uint8_t offset, uint16_t value, TreatAs16Bit) noexcept {
        if (offset < 80) {
            sdCardPath_[offset] = static_cast<char>(value);
            sdCardPath_[offset+1] = static_cast<char>(value >> 8);
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
    void ctlWrite(uint8_t offset, uint32_t value, TreatAs32Bit) noexcept {
        if (offset <= 76) {
            sdCardPath_[offset] = static_cast<char>(value);
            sdCardPath_[offset+1] = static_cast<char>(value >> 8);
            sdCardPath_[offset+2] = static_cast<char>(value >> 16);
            sdCardPath_[offset+3] = static_cast<char>(value >> 24);
        } else {
            // do nothing
        }
    }
    uint8_t ctlRead(uint8_t offset, TreatAs8Bit) noexcept {
        if (offset < 80) {
            return sdCardPath_[offset];
        } else {
            /// @todo does it make sense to return parts of values?
            return 0;
        }
    }
    uint16_t ctlRead(uint8_t offset, TreatAs16Bit) noexcept {
        if (offset < 80) {
            return SplitWord16(reinterpret_cast<uint16_t*>(sdCardPath_)[offset >> 1]).getWholeValue();
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
    uint32_t ctlRead(uint8_t offset, TreatAs32Bit) noexcept {
        if (offset < 80) {
            return SplitWord32(reinterpret_cast<uint32_t*>(sdCardPath_)[offset >> 2]).getWholeValue();
        } else {
            using T = SDCardFileSystemRegisters;
            switch (static_cast<T>(offset)) {
                case T::SDClusterCountLower:
                    return clusterCount_.getWholeValue();
                case T::SDVolumeSectorCountLower:
                    return volumeSectorCount_.getWholeValue();
                default:
                    return 0;
            }
        }
    }
    template<typename T>
    void
    writeGeneric(uint32_t address, typename T::StorageType value) noexcept {
        if (auto targetPage = static_cast<uint8_t>(address >> 8), targetOffset = static_cast<uint8_t>(address); targetPage) {
            ctlWrite(targetOffset, value, T{});
        } else if (targetPage >= FileStartPage && targetPage < FileEndPage) {
            fileWrite(targetPage - FileStartPage, targetOffset, value, T{});
        } else {

        }
    }
    template<typename T>
    typename T::StorageType
    readGeneric(uint32_t address) noexcept {
        if (auto targetPage = static_cast<uint8_t>(address >> 8), targetOffset = static_cast<uint8_t>(address); targetPage) {
            return ctlRead(targetOffset, T{});
        } else if (targetPage >= FileStartPage && targetPage < FileEndPage) {
            return fileRead(targetPage - FileStartPage, targetOffset, T{});
        } else {
            return 0;

        }
    }
public:
    void begin() noexcept {
        if (!initialized_) {
            while (!tryMountSDCard()) {
                Serial.println(F("SD CARD INIT FAILED...WILL RETRY SOON"));
                delay(1000);
            }
            Serial.println(F("SD CARD UP!"));
            clusterCount_ = SplitWord32(backingStore_.clusterCount());
            // if we use SdFat class then we have to do some work ourselves
            // we only know how many sectors per cluster and the number of clusters so we multiply them together
            // to get the volume sector count
            volumeSectorCount_ = SplitWord32{backingStore_.sectorsPerCluster() * backingStore_.clusterCount() };
            // to get the bytes per sector, we get the number of bytes per cluster and divide it by the number of sectors in a cluster
            // this will yield the number of bytes per sector.
            bytesPerSector_ = backingStore_.bytesPerCluster() / backingStore_.sectorsPerCluster();
            initialized_ = true;
        }
    }
#if 0
    static uint16_t read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept {
        if (targetPage == CTLPage) {
            return ctlRead(offset, lss);
        } else if (targetPage >= FileStartPage && targetPage < FileEndPage) {
            return fileRead(targetPage - FileStartPage, offset, lss);
        } else {
            return 0;
        }
    }
    static void write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
        if (targetPage == CTLPage) {
            ctlWrite(offset, lss, value);
        } else if (targetPage >= FileStartPage && targetPage < FileEndPage) {
            fileWrite(targetPage - FileStartPage, offset, lss, value);
        } else {
            // do nothing
        }
    }
#endif
    void write8(uint32_t address, uint8_t value) noexcept override {
        writeGeneric<TreatAs8Bit>(address, value);
    }
    void write16(uint32_t address, uint16_t value) noexcept override {
        writeGeneric<TreatAs16Bit>(address, value);
    }
    void write32(uint32_t address, uint32_t value) noexcept override {
        writeGeneric<TreatAs32Bit>(address, value);
    }
    uint8_t read8(uint32_t address) const noexcept override {
        return readGeneric<TreatAs8Bit>(address);
    }
    uint16_t read16(uint32_t address) const noexcept override {
        return readGeneric<TreatAs16Bit>(address);
    }
    uint32_t read32(uint32_t address) const noexcept override {
        return readGeneric<TreatAs32Bit>(address);
    }

private:
    SdFat& backingStore_;
    SplitWord32 clusterCount_ {0};
    SplitWord32 volumeSectorCount_ {0};
    uint16_t bytesPerSector_ = 0;
    uint16_t numberOfOpenFiles_ = 0;
    char sdCardPath_[81] = { 0 };
    OpenFileHandle files_[MaximumNumberOfOpenFiles];
    bool makeMissingParentDirectories_ = false;
    uint16_t filePermissions_ = 0;
    bool cardMounted_ = false;
    bool initialized_ = false;
};
#endif //SXCHIPSET_SDCARDINTERFACE_H
