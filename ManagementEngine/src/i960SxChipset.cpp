/*
i960SxManagementEngine
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

/// i960Sx management engine, based on atmega4809 with fuses set for:
/// - C++17
/// Board Platform: MegaCoreX
#include <Arduino.h>
#include <Event.h>
#include <Logic.h>

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
enum class i960Pinout : int {
    // PORT A
    ME_PA0 = PIN_PA0,
    ME_PA1 = PIN_PA1,
    ME_PA2 = PIN_PA2,
    CLK = PIN_PA3,
    ME_PA4 = PIN_PA4,
    ME_PA5 = PIN_PA5,
    ME_PA6 = PIN_PA6,
    CLK2 = PIN_PA7,
    // PORT C
    PIC_BOOTED_ = PIN_PC0,
    CHIPSET_BOOTED_ = PIN_PC1,
    ME_PC2 = PIN_PC2,
    RESET960_ = PIN_PC3,
    UPDI_TXD = PIN_PC4,
    UPDI_RXD = PIN_PC5,
    // PORT D
    BE1_ = PIN_PD0,
    BE0_ = PIN_PD1,
    BLAST_ = PIN_PD2,
    FAIL_SIG = PIN_PD3,
    FAIL = PIN_PD4,
    LOCK_ = PIN_PD5,
    CLK_READY_ = PIN_PD6,
    ME_PD7 = PIN_PD7,
    //
    BurstLast_ = PIN_PE0,
    InTransaction_ = PIN_PE1,
    DoCycle_ = PIN_PE2,
    BootSuccessful_ = PIN_PE3,
    //
    ME_Ready = PIN_PF0,
    ME_PF1 = PIN_PF1,
    ME_PF2 = PIN_PF2,
    READY960 = PIN_PF3, // connected to external inverter, CCL connector
    READY_CHIPSET_ = PIN_PF4, // comes from chipset
    DEN960 = PIN_PF5,
};

enum class PinStyle {
    Input,
    Output,
    InputPullup,
    Any,
};

template<PinStyle style>
constexpr decltype(OUTPUT) PinDirection_v = INPUT; // default to input
template<> constexpr auto PinDirection_v<PinStyle::Input> = INPUT;
template<> constexpr auto PinDirection_v<PinStyle::InputPullup> = INPUT_PULLUP;
template<> constexpr auto PinDirection_v<PinStyle::Output> = OUTPUT;

template<i960Pinout pin, PinStyle style, decltype(HIGH) asserted, decltype(HIGH) deasserted>
struct DigitalPin {
    static_assert(asserted != deasserted, "asserted cannot be equal to deasserted");
    DigitalPin() = delete;
    ~DigitalPin() = delete;
    DigitalPin(const DigitalPin&) = delete;
    DigitalPin(DigitalPin&&) = delete;
    DigitalPin& operator=(const DigitalPin&) = delete;
    DigitalPin& operator=(DigitalPin&&) = delete;
    static constexpr bool isBidirectionalPin() noexcept { return style == PinStyle::Any; }
    static constexpr bool isInputPin() noexcept { return style == PinStyle::Input || isBidirectionalPin(); }
    static constexpr bool isOutputPin() noexcept { return style == PinStyle::Output || isBidirectionalPin(); }
    static constexpr bool isInputPullupPin() noexcept { return style == PinStyle::InputPullup || isBidirectionalPin(); }
    static constexpr auto getDirection() noexcept { return PinDirection_v<style>; }
    static constexpr auto getPin() noexcept { return pin; }
    static void configure(decltype(OUTPUT) pinDirection = getDirection()) noexcept {
        ::pinMode(static_cast<int>(getPin()), pinDirection);
    }
    static auto read() noexcept {
        return digitalReadFast(static_cast<int>(pin));
    }
    static bool inputAsserted() noexcept { return read() == asserted; }
    static bool inputDeasserted() noexcept { return read() == deasserted; }
    static bool inputLow() noexcept { return read() == LOW; }
    static bool inputHigh() noexcept { return read() == HIGH; }
    static void write(decltype(HIGH) value) noexcept {
        digitalWriteFast(static_cast<int>(pin), value);
    }
    static void assertPin() noexcept { write(asserted); }
    static void deassertPin() noexcept { write(deasserted); }
    static void pulse() noexcept {
        assertPin();
        __builtin_avr_nops(2);
        deassertPin();
    }
};

/**
 * @brief Abstract interface to the Serial object used as the debug console
 */
decltype(Serial1)& DebugConsole = Serial1;
/**
 * @brief Abstract interface to the Serial object used as the communication and configuration channel
 */
decltype(Serial)& CommunicationChannel = Serial;
template<i960Pinout pin, decltype(HIGH) asserted, decltype(HIGH) deasserted>
using OutputPin = DigitalPin<pin, PinStyle::Output, asserted, deasserted>;

template<i960Pinout pin, decltype(HIGH) asserted, decltype(HIGH) deasserted>
using InputPin = DigitalPin<pin, PinStyle::Input, asserted, deasserted>;
template<i960Pinout pin, decltype(HIGH) asserted, decltype(HIGH) deasserted>
using InputPullupPin = DigitalPin<pin, PinStyle::InputPullup, asserted, deasserted>;

template<i960Pinout pin, decltype(HIGH) asserted, decltype(HIGH) deasserted>
using BidirectionalPin = DigitalPin<pin, PinStyle::Any, asserted, deasserted>;

[[gnu::always_inline]]
inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
    digitalWriteFast(static_cast<int>(ip), value);
}

[[gnu::always_inline]]
inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
    pinMode(static_cast<int>(ip), value);
}
[[gnu::always_inline]]
inline auto digitalRead(i960Pinout ip) {
    return digitalRead(static_cast<int>(ip));
}
template<bool condition, typename A, typename B>
struct ConditionalSelector {
public:
    using SelectedType = B;
public:
    ConditionalSelector() = delete;
    ~ConditionalSelector() = delete;
    ConditionalSelector(const ConditionalSelector&) = delete;
    ConditionalSelector(ConditionalSelector&&) = delete;
    ConditionalSelector& operator=(const ConditionalSelector&) = delete;
    ConditionalSelector& operator=(ConditionalSelector&&) = delete;

};


using Den960 = InputPin<i960Pinout::DEN960, LOW, HIGH>;
using BlastPin = InputPin<i960Pinout::BLAST_, LOW, HIGH>;
using FailPin = InputPin<i960Pinout::FAIL, HIGH, LOW>;
using FailOutPin = OutputPin<i960Pinout::FAIL_SIG, HIGH, LOW>;
using LockPin = InputPin<i960Pinout::LOCK_, LOW, HIGH>;
using BootSuccessfulPin = OutputPin<i960Pinout::BootSuccessful_, LOW, HIGH>;
using ChipsetBootedPin = InputPin<i960Pinout::CHIPSET_BOOTED_, LOW, HIGH>;
using PICBootedPin = InputPin<i960Pinout::PIC_BOOTED_, LOW, HIGH>;
using ClockReady = OutputPin<i960Pinout::CLK_READY_, LOW, HIGH>;
using DoCyclePin = OutputPin<i960Pinout::DoCycle_, LOW, HIGH>;
using ReadySyncPin = OutputPin<i960Pinout::ME_Ready, LOW, HIGH>;
using ReadyInputPin = OutputPin<i960Pinout::READY_CHIPSET_, LOW, HIGH>;
using InTransactionPin = OutputPin<i960Pinout::InTransaction_, LOW, HIGH>;
using BurstNext = OutputPin<i960Pinout::BurstLast_, HIGH, LOW>;
using ResetPin = OutputPin<i960Pinout::RESET960_, LOW, HIGH>;

template<typename ... pins>
[[gnu::always_inline]]
inline void configurePins() noexcept {
    (pins::configure(), ...);
}
template<typename ... pins>
[[gnu::always_inline]]
inline void deassertPins() noexcept {
    (pins::deassertPin(), ...);
}
void
setupPins() noexcept {
    configurePins<FailPin,
            BlastPin ,
            Den960 ,
            LockPin,
            FailOutPin,
            ChipsetBootedPin,
            PICBootedPin,
            DoCyclePin,
            ReadySyncPin,
            ReadyInputPin,
            InTransactionPin,
            BurstNext,
            ResetPin
    >();
    // the lock pin is special as it is an open collector pin, we want to stay off of it as much as possible
    LockPin::configure(INPUT);
    ResetPin ::assertPin();
    DoCyclePin ::deassertPin();
    ReadyInputPin :: deassertPin();
    ReadySyncPin :: deassertPin();
    InTransactionPin :: deassertPin();
    BurstNext :: deassertPin();
    FailOutPin :: deassertPin();
    // make all outputs deasserted
}
template<i960Pinout pin, decltype(HIGH) value>
[[gnu::always_inline]]
inline void digitalWrite() noexcept {
    digitalWrite(pin, value);
}
template<i960Pinout pin>
[[gnu::always_inline]]
inline void digitalWrite(decltype(HIGH) value) noexcept {
    digitalWriteFast(static_cast<int>(pin), value);
}
template<i960Pinout pin>
[[gnu::always_inline]] inline void digitalWrite(bool level) noexcept {
    digitalWriteFast(static_cast<int>(pin), level ? HIGH : LOW);
}

template<i960Pinout pin, decltype(HIGH) asserted = LOW>
[[gnu::always_inline]]
inline void pulse() noexcept {
    static constexpr auto deasserted = asserted == LOW ? HIGH : LOW;
    // use the switch to value to compute what to revert to
    digitalWrite<pin, asserted>();
    __builtin_avr_nops(4);
    digitalWrite<pin, deasserted>();
}

template<i960Pinout pin>
[[gnu::always_inline]]
inline auto digitalRead() noexcept {
    return digitalReadFast(static_cast<int>(pin));
}




template<typename T>
class PinAsserter {
public:
    static_assert(T::isOutputPin());
    PinAsserter() { T::assertPin(); }
    ~PinAsserter() { T::deassertPin(); }
};

[[noreturn]]
void
terminateExecution() noexcept {
    BootSuccessfulPin :: deassertPin();
    while(true) {
        delay(1000);
    }
}
[[noreturn]]
void
handleChecksumFail() noexcept {
    terminateExecution();
}

[[noreturn]]
void
tooManyCyclesForTransaction() noexcept {
   terminateExecution();
}

void
configureClockSource() noexcept {
    ClockReady::configure();
    ClockReady::deassertPin();
    CCP = 0xD8;
    CLKCTRL.MCLKCTRLA = 0b1000'0000;
    CCP = 0xD8;
    CCP = 0xD8;
    CLKCTRL.OSC20MCTRLA |= 0b0000'0010;
    CCP = 0xD8;
}
void
setup10MHz_CCL() noexcept {
    // use CCL0 and CCL1 to generate a 10MHz clock source
    // connect the output pin (pa7) to the event system which is the 20MHz clock
    Event0.set_generator(gen0::pin_pa7);
    Event0.set_user(user::ccl1_event_a);
    Event0.set_user(user::ccl0_event_a);
    Event1.set_generator(gen::ccl0_out);
    Event1.set_user(user::ccl3_event_a);
    Logic0.enable = true;
    Logic0.input0 = in::feedback; // route the result of the flipflop back to CCL0 to generate the divider effect
    Logic0.input1 = in::disable;
    Logic0.input2 = in::disable;
    Logic0.output = out::enable; // output the 10MHz clock from here
    Logic0.truth = 0b01010101;
    Logic0.sequencer = sequencer::jk_flip_flop;

    Logic1.enable = true;
    Logic1.input0 = in::event_a;
    Logic1.input1 = in::disable;
    Logic1.input2 = in::disable;
    Logic1.output = out::disable;
    Logic1.sequencer = sequencer::disable;
    Logic1.truth = 0b01010101;
    Logic0.init();
    Logic1.init();
    Event0.start();
    Event1.start();
}
void
setupFailSignalDetector_CCL() noexcept {
    Logic2.enable = true;
    // BE0, BE1, ~BLAST -> 0b011
    Logic2.input0 = in::pin;
    Logic2.input1 = in::pin;
    Logic2.input2 = in::pin;
    Logic2.output = out::enable;
    Logic2.sequencer = sequencer::disable;
    Logic2.truth = 0b0000'1000;
    Logic2.init();
}
void
setupReadySignal_CCL() noexcept {
    Logic3.enable = true;
    Logic3.input0 = in::pin;
    Logic3.input1 = in::disable;
    Logic3.input2 = in::event_a;
    Logic3.output = out::enable;
    Logic3.clocksource = clocksource::in2;
    Logic3.edgedetect = edgedetect::enable;
    Logic3.filter = filter::sync;
    Logic3.truth = 0b0001'0001;
    Logic3.init();
}
void
setupCCL() noexcept {
    setup10MHz_CCL();
    setupFailSignalDetector_CCL();
    setupReadySignal_CCL();
    Logic::start();
}
// the setup routine runs once when you press reset:
void setup() {
    configureClockSource();
    delay(2000);
    setupPins();
    setupCCL();
    // at this point all of the clock sources are ready
    ClockReady::assertPin();
    // now wait for chipset to boot and pic to be booted
    while (ChipsetBootedPin::inputDeasserted() || PICBootedPin :: inputDeasserted());
    ResetPin :: deassertPin();
    // no need to wait for the chipset to release control
    while (FailPin::inputLow()) {
        if (Den960::inputAsserted()) {
            break;
        }
    }
    while (FailPin::inputHigh()) {
        if (Den960::inputAsserted()) {
            break;
        }
    }
    BootSuccessfulPin::assertPin();
    // at this point, if we ever go from low to high again then we have a checksum failure
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::FAIL)), handleChecksumFail, RISING);
}
template<bool enable = currentConfiguration.enableOneCycleWaitStates()>
[[gnu::always_inline]]
inline void waitOneBusCycle() noexcept {
    if constexpr (enable) {
        __builtin_avr_nops(2);
    }
}
[[gnu::always_inline]]
inline void informCPUAndWait() noexcept {
    ReadySyncPin :: pulse();
    while (ReadyInputPin::inputAsserted());
    waitOneBusCycle();
}

[[gnu::always_inline]]
inline void waitForCycleEnd() noexcept {
    while (ReadyInputPin::inputDeasserted());
    DoCyclePin ::deassertPin();
    waitOneBusCycle();
}
volatile byte numCycles = 0;
[[noreturn]]
void loop() {
    for (;;) {
        // introduce some delay to make sure the bus has time to recover properly
        waitOneBusCycle();
        // okay so we need to wait for DEN to go low
        while (Den960::inputDeasserted());

        if (numCycles >= currentConfiguration.getMaxNumberOfCyclesBeforePause()) {
            // provide a pause/cooldown period after a new data request to make sure that the bus has time to "cool".
            // Failure to do so can cause very strange checksum failures / chipset faults to happen with the GCM4
            // this is not an issue since the i960 will wait until ready is signaled
            while (numCycles > 0) {
                // use the loop itself to provide some amount of time to cool off
                // numCycles is volatile to prevent the compiler from optimizing any of this away.
               --numCycles;
            }
        }
        // now do the logic
        {
            // tell the chipset we are starting a transaction
            InTransactionPin :: assertPin();
            // okay now we need to emulate a wait loop to allow the chipset time to do its thing
           for (/*byte numBurstCycles = 0;;++numBurstCycles*/ ;;) {
#if 0
               if (numBurstCycles >= 8) {
                   // too many cycles for a given transaction have occurred
                   tooManyCyclesForTransaction();
               }
#endif
               // instead of pulsing do cycle, we just assert it while we wait
               // this has the added benefit of providing proper synchronization between two different clock domains
               // for example, the GCM4 runs at 120MHz while this chip runs at 20MHz. Making the chipset wait provides implicit
               // synchronization
               DoCyclePin :: assertPin();
               // we have entered a new transaction so increment the counter
               // we want to count the number of transaction cycles
               ++numCycles;
               // now wait for the chipset to tell us that it has satisifed the current part of the transaction
               if (BlastPin::inputAsserted()) {
                   // if it turns out that blast is asserted then we break out of this loop and handle it specially
                   break;
               }
               // we are dealing with a burst transaction at this point
               waitForCycleEnd();
               // let the chipset know that the operation will continue
               {
                   BurstNext::assertPin();
                   informCPUAndWait();
                   BurstNext::deassertPin();
               }
           }
           // the end of the current transaction needs to be straighline code
           waitForCycleEnd();
           // okay tell the chipset transaction complete
            InTransactionPin :: deassertPin();
        }
        // we have to tie off the transaction itself first
        // let the i960 know and then wait for the chipset to pull MCU READY high
        informCPUAndWait();
        // to make sure the bus has time to recover we can introduce an i960 bus cycle worth of delay
        waitOneBusCycle();
    }
}

