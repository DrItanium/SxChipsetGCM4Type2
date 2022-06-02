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

// contains routines for handling microcontroller platform detection

#ifndef I960SXCHIPSET_MCUPLATFORM_H
#define I960SXCHIPSET_MCUPLATFORM_H
#include <Arduino.h>
#include "type_traits.h"
#include "DependentFalse.h"

// comment this out to disable sram cache support
#ifdef ARDUINO_AVR_ATmega1284
#define PACKED_ATTRIBUTE
#else
#define PACKED_ATTRIBUTE __attribute__((packed))
#endif
#ifdef __AVR__
using int24_t = __int24;
using uint24_t = __uint24;
#else
using int24_t = int32_t;
using uint24_t = uint32_t;
#endif
#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

template<typename E>
constexpr bool isValidPin(E pin) noexcept {
    return static_cast<int>(pin) < static_cast<int>(E::Count) &&
           static_cast<int>(pin) >= 0;
}
template<auto pin>
constexpr bool isValidPin_v = isValidPin(pin);
constexpr unsigned long long int operator "" _KB(unsigned long long int value) noexcept { return value * 1024; }
constexpr unsigned long long int operator "" _MB(unsigned long long int value) noexcept { return value * 1024 * 1024; }
constexpr unsigned long long int operator "" _KHz(unsigned long long int value) noexcept { return value * 1000; }
constexpr unsigned long long int operator "" _MHz(unsigned long long int value) noexcept { return value * 1000 * 1000; }
static_assert(2_KHz == 2'000);
static_assert(2_MHz == 2'000'000);
static_assert(20_MHz == 20'000'000);

static constexpr byte BitMaskTable_Byte[8] {
        0b0000'0001,
        0b0000'0010,
        0b0000'0100,
        0b0000'1000,
        0b0001'0000,
        0b0010'0000,
        0b0100'0000,
        0b1000'0000,
};

constexpr uint64_t pow2(uint64_t value) noexcept {
    if (value == 0) {
        return 1;
    } else {
        return pow2(value - 1) * 2;
    }
}

static constexpr byte numberOfBitsForCount(uint64_t count) noexcept {
    switch (count) {
#define X(offset) case pow2(offset): return offset
        X(1);
        X(2);
        X(3);
        X(4);
        X(5);
        X(6);
        X(7);
        X(8);
        X(9);
        X(10);
        X(11);
        X(12);
        X(13);
        X(14);
        X(15);
        X(16);
        X(17);
        X(18);
        X(19);
        X(20);
        X(21);
        X(22);
        X(23);
        X(24);
        X(25);
        X(26);
        X(27);
        X(28);
        X(29);
        X(30);
        X(31);
        X(32);
        X(33);
        X(34);
        X(35);
        X(36);
        X(37);
        X(38);
        X(39);
        X(40);
        X(41);
        X(42);
        X(43);
        X(44);
        X(45);
        X(46);
        X(47);
        X(48);
        X(49);
        X(50);
        X(51);
        X(52);
        X(53);
        X(54);
        X(55);
        X(56);
        X(57);
        X(58);
        X(59);
        X(60);
        X(61);
        X(62);
        X(63);
#undef X
        default: return 0;
    }
}
static constexpr byte getNumberOfBitsForNumberOfEntries(uint64_t count) noexcept { return numberOfBitsForCount(count); }

static_assert(getNumberOfBitsForNumberOfEntries(512/4) == 7);
static_assert(getNumberOfBitsForNumberOfEntries(256/4) == 6);

template<byte numBits>
using ClosestBitValue_t = conditional_t<numBits <= 8, byte,
                                        conditional_t<numBits <= 16, uint16_t,
                                        conditional_t<numBits <= 24, uint24_t,
                                        conditional_t<numBits <= 32, uint32_t, uint64_t>>>>;

static_assert(is_same_v<ClosestBitValue_t<1>, ClosestBitValue_t<4>>);
static_assert(is_same_v<ClosestBitValue_t<4>, byte>);
static_assert(is_same_v<ClosestBitValue_t<10>, uint16_t>);
static_assert(!is_same_v<ClosestBitValue_t<10>, ClosestBitValue_t<4>>);


enum class TargetMCU {
    GrandCentralM4_Type3,
    Unknown,
};
enum class ConfigurationOptions : byte {
    CompileInAddressDebuggingSupport = 0,
    AddressDebuggingSupportEnabledOnStartup,
    CompileInCacheSystemDebuggingSupport,
    CompileInExtendedDebugInformation,
    ValidateTransferDuringInstall,
    UseIOExpanderAddressLineInterrupts,
    UsePortReads,
    SeparateReadAndWriteFunctionPointers,
    EnableDisplayDriver,
    EnableRTCInterface,
    CaptureAddressWithParallelLines,
};
enum class ConfigurationFlags : uint64_t {
    None = 0,
#define X(name) name = static_cast<uint64_t>(static_cast<uint64_t>(1) << (static_cast<uint64_t>(ConfigurationOptions:: name)))
    X(CompileInAddressDebuggingSupport),
    X(AddressDebuggingSupportEnabledOnStartup),
    X(CompileInCacheSystemDebuggingSupport),
    X(CompileInExtendedDebugInformation),
    X(ValidateTransferDuringInstall),
    X(UseIOExpanderAddressLineInterrupts),
    X(UsePortReads),
    X(SeparateReadAndWriteFunctionPointers),
    X(EnableDisplayDriver),
    X(EnableRTCInterface),
    X(CaptureAddressWithParallelLines),
#undef X
};
template<ConfigurationFlags ... flags>
constexpr ConfigurationFlags makeConfigFlags() noexcept {
    return static_cast<ConfigurationFlags>((static_cast<uint64_t>(flags) | ...));
}
class MCUConfiguration final {
public:
    constexpr MCUConfiguration(uint32_t sramSize,
                               uint32_t ioExpanderSpeedCap,
                               uint32_t numBitsPerCacheLine,
                               uint32_t cacheSizeInBytes,
                               ConfigurationFlags flags
    ) noexcept : sramAmount_(sramSize),
                 ioExpanderPeripheralSpeed_(ioExpanderSpeedCap > 10_MHz ? 10_MHz : ioExpanderSpeedCap),
                 numBitsPerCacheLine_(numBitsPerCacheLine),
                 cacheSizeInBytes_(cacheSizeInBytes),
                 flags_(flags)
                 {}

    [[nodiscard]] constexpr uint32_t getSramAmount() const noexcept { return sramAmount_; }
    [[nodiscard]] constexpr auto runIOExpanderSPIInterfaceAt() const noexcept  { return ioExpanderPeripheralSpeed_; }
    [[nodiscard]] constexpr auto getNumBitsPerCacheLine() const noexcept { return numBitsPerCacheLine_; }
    [[nodiscard]] constexpr auto getCacheSizeInBytes() const noexcept { return cacheSizeInBytes_; }
    template<ConfigurationFlags flag> [[nodiscard]] constexpr auto flagSet() const noexcept { return (static_cast<uint64_t>(flags_) & static_cast<uint64_t>(flag)) != 0; }
    template<ConfigurationFlags flag> [[nodiscard]] constexpr auto flagClear() const noexcept { return !flagSet<flag>(); }
    [[nodiscard]] constexpr auto compileInAddressDebuggingSupport() const noexcept { return flagSet<ConfigurationFlags::CompileInAddressDebuggingSupport>(); }
    [[nodiscard]] constexpr auto addressDebuggingEnabledOnStartup() const noexcept { return flagSet<ConfigurationFlags::AddressDebuggingSupportEnabledOnStartup>(); }
    [[nodiscard]] constexpr auto compileInCacheSystemDebuggingSupport() const noexcept { return flagSet<ConfigurationFlags::CompileInCacheSystemDebuggingSupport>(); }
    [[nodiscard]] constexpr auto compileInExtendedDebugInformation() const noexcept { return flagSet<ConfigurationFlags::CompileInExtendedDebugInformation>(); }
    [[nodiscard]] constexpr auto validateTransferDuringInstall() const noexcept { return flagSet<ConfigurationFlags::ValidateTransferDuringInstall>(); }
    [[nodiscard]] constexpr auto useIOExpanderAddressLineInterrupts() const noexcept { return flagSet<ConfigurationFlags::UseIOExpanderAddressLineInterrupts>(); }
    [[nodiscard]] constexpr auto usePortReads() const noexcept { return flagSet<ConfigurationFlags::UsePortReads>(); }
    [[nodiscard]] constexpr auto separateReadAndWriteFunctionPointers() const noexcept { return flagSet<ConfigurationFlags::SeparateReadAndWriteFunctionPointers>(); }
    [[nodiscard]] constexpr auto enableDisplayDriver() const noexcept { return flagSet<ConfigurationFlags::EnableDisplayDriver>(); }
    [[nodiscard]] constexpr auto enableRTCInterface() const noexcept { return flagSet<ConfigurationFlags::EnableRTCInterface>(); }
    [[nodiscard]] constexpr auto captureAddressWithSPI() const noexcept { return flagClear<ConfigurationFlags::CaptureAddressWithParallelLines>(); }
    [[nodiscard]] constexpr auto captureAddressWithParallel() const noexcept { return flagSet<ConfigurationFlags::CaptureAddressWithParallelLines>(); }
private:
    uint32_t sramAmount_;
    uint32_t ioExpanderPeripheralSpeed_;
    uint32_t numBitsPerCacheLine_;
    uint32_t cacheSizeInBytes_;
    ConfigurationFlags flags_;
};
template<TargetMCU mcu>
constexpr MCUConfiguration BoardDescription = {
        0,
        10_MHz,
        6, // 64 bytes
        8192,
        makeConfigFlags <ConfigurationFlags::UsePortReads,
                ConfigurationFlags::UseIOExpanderAddressLineInterrupts,
                ConfigurationFlags::ValidateTransferDuringInstall>()
};

template<>
constexpr MCUConfiguration BoardDescription<TargetMCU::GrandCentralM4_Type3> = {
        256_KB,
        8_MHz, // although 10_MHz is the max, the clock rate of 120MHz makes the clock rate actually 12MHz, I know 8mhz works
        6,
        128_KB,
        makeConfigFlags<ConfigurationFlags::UsePortReads,
                //ConfigurationFlags::CompileInAddressDebuggingSupport,
                //ConfigurationFlags::AddressDebuggingSupportEnabledOnStartup,
                ConfigurationFlags::UseIOExpanderAddressLineInterrupts,
                ConfigurationFlags::SeparateReadAndWriteFunctionPointers,
                ConfigurationFlags::ValidateTransferDuringInstall,
                ConfigurationFlags::CaptureAddressWithParallelLines>()
};

class TargetBoard {
public:
    [[nodiscard]] static constexpr auto getCPUFrequency() noexcept { return F_CPU; }
    [[nodiscard]] static constexpr TargetMCU getMCUTarget() noexcept {
#if defined(CHIPSET_TYPE3)
        return TargetMCU::GrandCentralM4_Type3;
#else
        return TargetMCU::Unknown;
#endif
    }
    template<TargetMCU mcu>
    [[nodiscard]] static constexpr auto targetMCUIs() noexcept { return getMCUTarget() == mcu; }
    template<TargetMCU ... rest>
    [[nodiscard]] static constexpr auto targetMCUIsOneOfThese() noexcept {
        return (targetMCUIs<rest>() || ...);
    }
    [[nodiscard]] static constexpr auto onType3() noexcept { return targetMCUIs<TargetMCU::GrandCentralM4_Type3>(); }
    [[nodiscard]] static constexpr auto onSAMD51() noexcept { return targetMCUIsOneOfThese<TargetMCU::GrandCentralM4_Type3>(); }
    [[nodiscard]] static constexpr auto onGrandCentralM4() noexcept { return onType3(); }
    [[nodiscard]] static constexpr auto onUnknownTarget() noexcept { return targetMCUIs<TargetMCU::Unknown>(); }
    [[nodiscard]] static constexpr auto getSRAMAmountInBytes() noexcept { return BoardDescription<getMCUTarget()>.getSramAmount(); }
    [[nodiscard]] static constexpr auto runIOExpanderSPIInterfaceAt() noexcept { return BoardDescription<getMCUTarget()>.runIOExpanderSPIInterfaceAt(); }
    [[nodiscard]] static constexpr auto getCacheSize() noexcept { return BoardDescription<getMCUTarget()>.getCacheSizeInBytes(); }
    [[nodiscard]] static constexpr auto getCacheLineSizeInBits() noexcept { return BoardDescription<getMCUTarget()>.getNumBitsPerCacheLine(); }
    [[nodiscard]] static constexpr auto compileInAddressDebuggingSupport() noexcept { return BoardDescription<getMCUTarget()>.compileInAddressDebuggingSupport(); }
    [[nodiscard]] static constexpr auto addressDebuggingEnabledOnStartup() noexcept { return BoardDescription<getMCUTarget()>.addressDebuggingEnabledOnStartup(); }
    [[nodiscard]] static constexpr auto compileInCacheSystemDebuggingSupport() noexcept { return BoardDescription<getMCUTarget()>.compileInCacheSystemDebuggingSupport(); }
    [[nodiscard]] static constexpr auto compileInExtendedDebugInformation() noexcept { return BoardDescription<getMCUTarget()>.compileInExtendedDebugInformation(); }
    [[nodiscard]] static constexpr auto validateTransferDuringInstall() noexcept { return BoardDescription<getMCUTarget()>.validateTransferDuringInstall(); }
    [[nodiscard]] static constexpr auto useIOExpanderAddressLineInterrupts() noexcept { return BoardDescription<getMCUTarget()>.useIOExpanderAddressLineInterrupts(); }
    [[nodiscard]] static constexpr auto usePortReads() noexcept { return BoardDescription<getMCUTarget()>.usePortReads(); }
    [[nodiscard]] static constexpr auto separateReadWriteFunctionPointers() noexcept { return BoardDescription<getMCUTarget()>.separateReadAndWriteFunctionPointers(); }
    [[nodiscard]] static constexpr auto enableDisplayDriver() noexcept { return BoardDescription<getMCUTarget()>.enableDisplayDriver(); }
    [[nodiscard]] static constexpr auto enableRTCInterface() noexcept { return BoardDescription<getMCUTarget()>.enableRTCInterface(); }
    [[nodiscard]] static constexpr auto addressViaSPI() noexcept { return BoardDescription<getMCUTarget()>.captureAddressWithSPI(); }
    [[nodiscard]] static constexpr auto addressViaParallel() noexcept { return BoardDescription<getMCUTarget()>.captureAddressWithParallel(); }
public:
    TargetBoard() = delete;
    ~TargetBoard() = delete;
    TargetBoard(const TargetBoard&) = delete;
    TargetBoard(TargetBoard&&) = delete;
    TargetBoard& operator=(const TargetBoard&) = delete;
    TargetBoard& operator=(TargetBoard&&) = delete;
};

static_assert(!TargetBoard::onUnknownTarget(), "ERROR: Target Board has not been defined, please define to continue");
static_assert(TargetBoard::getSRAMAmountInBytes() >= 16_KB, "ERROR: Less than 16kb of sram is not allowed!");

/**
 * @brief The backing design of the registers within the chipset that are 32-bits in width
 */
union SplitWord16 {
    explicit constexpr SplitWord16(uint16_t value = 0) noexcept : wholeValue_(value) { }
    constexpr SplitWord16(uint8_t lower, uint8_t upper) noexcept : bytes{lower, upper} { }
    [[nodiscard]] constexpr auto getWholeValue() const noexcept { return wholeValue_; }
    [[nodiscard]] constexpr auto getLowerHalf() const noexcept { return bytes[0]; }
    [[nodiscard]] constexpr auto getUpperHalf() const noexcept { return bytes[1]; }
    uint16_t wholeValue_ = 0;
    uint8_t bytes[sizeof(uint16_t) / sizeof(uint8_t)];
};
union SplitWord32 {
    // adding this dropped program size by over 500 bytes!
    explicit constexpr SplitWord32(uint32_t value = 0) noexcept : wholeValue_(value) { }
    constexpr SplitWord32(uint16_t lower, uint16_t upper) noexcept : halves{lower, upper} {}
    constexpr SplitWord32(uint8_t lowest, uint8_t lower, uint8_t higher, uint8_t highest) noexcept : bytes{lowest, lower, higher, highest} {}
    [[nodiscard]] constexpr auto getWholeValue() const noexcept { return wholeValue_; }
    [[nodiscard]] constexpr auto getSignedRepresentation() const noexcept { return signedRepresentation_; }
    [[nodiscard]] constexpr auto getTargetPage() const noexcept { return static_cast<byte>(wholeValue_ >> 8); }
    [[nodiscard]] constexpr auto getMostSignificantByte() const noexcept { return static_cast<byte>(wholeValue_ >> 24); }
    [[nodiscard]] constexpr auto getLowerHalf() const noexcept { return halves[0]; }
    [[nodiscard]] constexpr auto getUpperHalf() const noexcept { return halves[1]; }
    void setLowerHalf(SplitWord16 value) noexcept { words_[0] = value; }
    void setUpperHalf(SplitWord16 value) noexcept { words_[1] = value; }
    uint32_t wholeValue_ = 0;
    int32_t signedRepresentation_;
    byte bytes[sizeof(uint32_t)];
    uint16_t halves[sizeof(uint32_t) / sizeof(uint16_t)];
    SplitWord16 words_[sizeof(uint32_t) / sizeof(SplitWord16)];
    float floatingPointRepresentation_;
};
#endif //I960SXCHIPSET_MCUPLATFORM_H
