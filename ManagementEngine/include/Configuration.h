/*
SxChipsetGCM4Type2
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
//
// Created by jwscoggins on 7/2/22.
//

#ifndef MANAGEMENTENGINE_CONFIGURATION_H
#define MANAGEMENTENGINE_CONFIGURATION_H
#include <Arduino.h>

/// @todo implement a better configuration setup
class TargetConfiguration {
public:
    enum Flags : uint16_t {
/**
 * @brief Is this board using an external 20Mhz clock source? If so configure the ME to use that instead
 */
        HasExternalClockSource = (1 << 0),
/**
 * @brief Should the ME emit it core clock source? If true then PA7 becomes a CLKO pin
 */
        EnableClockOutput = (1 << 1),
/**
 * @brief Should the ME use PA4,5,6 as a serial communication channel for configuration purposes?
 */
        EnableCommunicationChannel = (1 << 2) ,
/**
 * @brief For some designs, it is important to introduce a one cycle wait state into the code at specific points for timing purposes.
 * Enable this if you're running into problems with random checksum fails during execution.
 */
        EnableOneCycleWaitStates = (1 << 3),
        /**
         * @brief If set, then the CCLs of the ME are used for edge detection of interrupt sources
         */
        BuiltinInterruptController = (1 << 4),
        EnableDebugConsole = (1 << 5),
        /**
         * @brief Due to a screwup in the expanded processor board, in transaction and boot successful are swapped.
         */
        InTransactionAndBootSuccessfulAreSwapped = (1 << 6),

        /**
         * @brief For some of the single board computers the management engine is responsible for watching the reset circuit
         */
        HandleResetManually = (1 << 7),
        /**
         * @brief Instead of actually acting as a management engine, just pulse ready constantly
         */
        DoReadyPulseLoopMode = (1 << 8),

        /**
         * @brief Wait for serial console connection before continuing execution
         */
        WaitForSerialConnect = ( 1 << 9),
        /**
         * @brief Inspect the ready input signal to see what its value is
         */
        TestReadyPinMode = (1 << 10),
    };

public:
    constexpr explicit TargetConfiguration(Flags flags, byte version, byte cyclesBeforePause = 64) noexcept :
            configuration_(static_cast<uint16_t>(flags)),
            version_(version),
            maxNumberOfCyclesBeforePause_(cyclesBeforePause) { }

    template<Flags flag>
    [[nodiscard]] constexpr bool hasFlagSet() const noexcept {
        return (configuration_ & static_cast<uint16_t>(flag)) != 0;
    }
    template<Flags flag>
    [[nodiscard]] constexpr bool hasFlagClear() const noexcept {
        return (configuration_ & static_cast<uint16_t>(flag)) == 0;
    }
    constexpr auto useExternalClockSource() const noexcept { return hasFlagSet<Flags::HasExternalClockSource>(); }
    constexpr auto useInternalOscillator() const noexcept { return hasFlagClear<Flags::HasExternalClockSource>(); }
    constexpr auto hasPICBuiltin() const noexcept { return hasFlagSet<Flags::BuiltinInterruptController>(); }
    constexpr auto emitClockSignalOnPA7() const noexcept { return hasFlagSet<Flags::EnableClockOutput>(); }
    constexpr auto enableOneCycleWaitStates() const noexcept { return hasFlagSet<Flags::EnableOneCycleWaitStates>(); }
    constexpr auto enableCommunicationChannel() const noexcept { return hasFlagSet<Flags::EnableCommunicationChannel>(); }
    constexpr auto getMaxNumberOfCyclesBeforePause() const noexcept { return maxNumberOfCyclesBeforePause_; }
    constexpr auto getVersion() const noexcept { return version_; }
    constexpr auto debugConsoleActive() const noexcept { return hasFlagSet<Flags::EnableDebugConsole>(); }
    constexpr auto inTransactionAndBootSuccessfulSwapped() const noexcept { return hasFlagSet<Flags::InTransactionAndBootSuccessfulAreSwapped>(); }
    constexpr auto waitForSerialConnect() const noexcept { return hasFlagSet<Flags::WaitForSerialConnect>(); }
    constexpr auto enableReadyPulseMode() const noexcept { return hasFlagSet<Flags::DoReadyPulseLoopMode>(); }
    constexpr auto enableTestReadyPinMode() const noexcept { return hasFlagSet<Flags::TestReadyPinMode>(); }
private:
    uint16_t configuration_;
    byte version_;
    byte maxNumberOfCyclesBeforePause_;
};
constexpr TargetConfiguration version2GCM {
        TargetConfiguration::Flags::HandleResetManually |
        TargetConfiguration::Flags::EnableOneCycleWaitStates,
        2,
        64
};
constexpr TargetConfiguration currentConfiguration = version2GCM;
#endif //MANAGEMENTENGINE_CONFIGURATION_H
