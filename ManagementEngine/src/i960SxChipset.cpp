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
constexpr TargetConfiguration version1 {
        TargetConfiguration::Flags::HasExternalClockSource
        | TargetConfiguration::Flags::EnableCommunicationChannel
        | TargetConfiguration::Flags::BuiltinInterruptController
        | TargetConfiguration::Flags::InTransactionAndBootSuccessfulAreSwapped
        //| TargetConfiguration::Flags::EnableDebugConsole
        ,
        1 /* version */,
        64 /* delay */ };
constexpr TargetConfiguration version1WithDebug {
        TargetConfiguration::Flags::HasExternalClockSource
        | TargetConfiguration::Flags::EnableCommunicationChannel
        | TargetConfiguration::Flags::BuiltinInterruptController
        | TargetConfiguration::Flags::InTransactionAndBootSuccessfulAreSwapped
        | TargetConfiguration::Flags::EnableDebugConsole
        | TargetConfiguration::Flags::WaitForSerialConnect
        ,
        1 /* version */,
        64 /* delay */ };
constexpr TargetConfiguration v1TestReadyPin {
        TargetConfiguration::Flags::HasExternalClockSource
        | TargetConfiguration::Flags::EnableCommunicationChannel
        | TargetConfiguration::Flags::BuiltinInterruptController
        | TargetConfiguration::Flags::InTransactionAndBootSuccessfulAreSwapped
        | TargetConfiguration::Flags::EnableDebugConsole
        | TargetConfiguration::Flags::WaitForSerialConnect
        | TargetConfiguration::Flags::TestReadyPinMode
        ,
        1 /* version */,
        64 /* delay */ };
constexpr TargetConfiguration version2GCM {
    TargetConfiguration::Flags::HandleResetManually,
    2,
    64
};
constexpr TargetConfiguration currentConfiguration = version1;
enum class i960Pinout : int {
    SRC0_TRIGGER_INT1 = PIN_PF0,
    SRC1_TRIGGER_INT1 = PIN_PF1,
    BUS_LOCKED = PIN_PF2,
    INT1 = PIN_PF3,
    LOCK = PIN_PF4,
    LOCK_REQUESTED = PIN_PF5,
    BOOT_SUCCESSFUL = PIN_PE0,
    DO_CYCLE = PIN_PE1,
    BURST_LAST_ME = PIN_PE2,
    IN_TRANSACTION = PIN_PE3,
    SRC0_TRIGGER_INT3 = PIN_PC0,
    SRC1_TRIGGER_INT3 = PIN_PC1,
    READY_IN = PIN_PC2,
    INT3 = PIN_PC3,
    CLK2 = PIN_PA0,
    SRC0_TRIGGER_INT0 = PIN_PA1,
    SRC1_TRIGGER_INT0 = PIN_PA7,
    INT0 = PIN_PA3,
    CLK = PIN_PA2,
    SRC0_TRIGGER_INT2 = PIN_PD0,
    SRC1_TRIGGER_INT2 = PIN_PD1,
    ME_BOOTED = PIN_PD2,
    INT2 = PIN_PD3,
    BLAST = PIN_PD4,
    DEN = PIN_PD5,
    FAIL = PIN_PD6,
    READY960 = PIN_PD7,
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


using DenPin = InputPin<i960Pinout::DEN, LOW, HIGH>;
using BlastPin = InputPin<i960Pinout::BLAST, LOW, HIGH>;
using FailPin = InputPin<i960Pinout::FAIL, HIGH, LOW>;
using Src0Trigger3Pin = InputPin<i960Pinout::SRC0_TRIGGER_INT3, LOW, HIGH>;
using Src1Trigger3Pin = InputPin<i960Pinout::SRC1_TRIGGER_INT3, LOW, HIGH>;
using Src0Trigger2Pin = InputPin<i960Pinout::SRC0_TRIGGER_INT2, LOW, HIGH>;
using Src1Trigger2Pin = InputPin<i960Pinout::SRC1_TRIGGER_INT2, LOW, HIGH>;
using Src0Trigger1Pin = InputPin<i960Pinout::SRC0_TRIGGER_INT1, LOW, HIGH>;
using Src1Trigger1Pin = InputPin<i960Pinout::SRC1_TRIGGER_INT1, LOW, HIGH>;
using Src0Trigger0Pin = InputPin<i960Pinout::SRC0_TRIGGER_INT0, LOW, HIGH>;
using Src1Trigger0Pin = InputPin<i960Pinout::SRC1_TRIGGER_INT0, LOW, HIGH>;
using LockRequestedPin = InputPin<i960Pinout::LOCK_REQUESTED, LOW, HIGH>;
using ReadyInputPin = InputPin<i960Pinout::READY_IN, LOW, HIGH>;
constexpr i960Pinout BootSuccessfulPinIndex = currentConfiguration.inTransactionAndBootSuccessfulSwapped() ? i960Pinout::IN_TRANSACTION : i960Pinout::BOOT_SUCCESSFUL;
constexpr i960Pinout InTransactionPinIndex = currentConfiguration.inTransactionAndBootSuccessfulSwapped() ? i960Pinout::BOOT_SUCCESSFUL : i960Pinout::IN_TRANSACTION;
using BootSuccessfulPin = OutputPin<BootSuccessfulPinIndex, HIGH, LOW>; // protocol assumed is active high
using DoCyclePin = OutputPin<i960Pinout::DO_CYCLE, LOW, HIGH>;
using BurstNext = OutputPin<i960Pinout::BURST_LAST_ME, HIGH, LOW>;
using InTransactionPin = OutputPin<InTransactionPinIndex, LOW, HIGH>;
using Int0Pin = OutputPin<i960Pinout::INT0, HIGH, LOW>;
using Int1Pin = OutputPin<i960Pinout::INT1, HIGH, LOW>;
using Int2Pin = OutputPin<i960Pinout::INT2, HIGH, LOW>;
using Int3Pin = OutputPin<i960Pinout::INT3, HIGH, LOW>;
using BootedPin = OutputPin<i960Pinout::ME_BOOTED, HIGH, LOW>;
using ReadySyncPin = OutputPin<i960Pinout::READY960, LOW, HIGH>;
using BusLockedPin = OutputPin<i960Pinout::BUS_LOCKED, LOW, HIGH>;

using LockPin = BidirectionalPin<i960Pinout::LOCK, LOW, HIGH>;

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
    configurePins<FailPin, BlastPin , DenPin ,
            LockRequestedPin , BusLockedPin ,
            Int0Pin , Int1Pin , Int2Pin, Int3Pin,
            InTransactionPin , DoCyclePin, BootSuccessfulPin , BurstNext ,
            Src0Trigger0Pin , Src1Trigger0Pin ,
            Src0Trigger1Pin , Src1Trigger1Pin ,
            Src0Trigger2Pin , Src1Trigger2Pin ,
            Src0Trigger3Pin , Src1Trigger3Pin ,
            ReadySyncPin , ReadyInputPin >();
    // the lock pin is special as it is an open collector pin, we want to stay off of it as much as possible
    LockPin::configure(OUTPUT);
    LockPin::write(HIGH);
    // make all outputs deasserted
    deassertPins<Int0Pin, Int1Pin, Int2Pin, Int3Pin, ReadySyncPin,
            BootSuccessfulPin, InTransactionPin, DoCyclePin,
            BurstNext,
            BusLockedPin>();
    /// @todo configure event system here
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
    if constexpr (currentConfiguration.debugConsoleActive()) {
        DebugConsole.println("Checksum failure!");
    }
    terminateExecution();
}

[[noreturn]]
void
tooManyCyclesForTransaction() noexcept {
   if constexpr (currentConfiguration.debugConsoleActive())  {
       DebugConsole.println("Too many cycles for the given transaction");
   }
   terminateExecution();
}

void
configureClockSource() noexcept {
    byte clkBits = 0;
    if constexpr (currentConfiguration.useExternalClockSource()) {
        clkBits |= 0b0000'0011;
    }
    if constexpr (currentConfiguration.emitClockSignalOnPA7()) {
        clkBits |= 0b1000'0000;
    }
    if constexpr (currentConfiguration.useExternalClockSource() || currentConfiguration.emitClockSignalOnPA7()) {
        CCP = 0xD8;
        CLKCTRL.MCLKCTRLA = clkBits;
        CCP = 0xD8;
    }
    if constexpr (currentConfiguration.emitClockSignalOnPA7()) {
        CCP = 0xD8;
        CLKCTRL.MCLKCTRLA |= 0b0000'0010;
        CCP = 0xD8;
    }
}
void configurePIC() noexcept {
    if constexpr (currentConfiguration.debugConsoleActive()) {
        DebugConsole.println("Setting up PIC!");
    }
    if constexpr (currentConfiguration.getVersion() == 1) {
        // okay so configure the event and logic system for version 1
        // connect 10MHz clock to all CCLs
        if constexpr (currentConfiguration.debugConsoleActive()) {
            DebugConsole.println("Setting up Event0!");
        }
        Event0.set_generator(gen0::pin_pa2);
        Event0.set_generator(user::ccl0_event_b);
        Event0.set_generator(user::ccl1_event_b);
        Event0.set_generator(user::ccl2_event_b);
        Event0.set_generator(user::ccl3_event_b);
        if constexpr (currentConfiguration.debugConsoleActive()) {
            DebugConsole.println("Setting up Event1!");
        }
        // use PA7 as a CCL input
        Event1.set_generator(gen1::pin_pa7);
        Event1.set_user(user::ccl0_event_a);
        if constexpr (currentConfiguration.debugConsoleActive()) {
            DebugConsole.println("Setting up CCLs!");
        }
        // setup Logic0 for int0 trigger but keep the truth table for the end
        Logic0.edgedetect = edgedetect::enable;
        Logic0.filter = filter::sync;
        Logic0.input0 = in::event_a;
        Logic0.input1 = in::input_pullup;
        Logic0.input2 = in::event_b;
        Logic0.clocksource = clocksource::in2;
        Logic0.output = out::enable;
        // do the same thing for the other CCLs
        Logic1.edgedetect = edgedetect::enable;
        Logic1.filter = filter::sync;
        Logic1.input0 = in::input_pullup;
        Logic1.input1 = in::input_pullup;
        Logic1.input2 = in::event_b;
        Logic1.clocksource = clocksource::in2;
        Logic1.output = out::enable;
        Logic2.edgedetect = edgedetect::enable;
        Logic2.filter = filter::sync;
        Logic2.input0 = in::input_pullup;
        Logic2.input1 = in::input_pullup;
        Logic2.input2 = in::event_b;
        Logic2.output = out::enable;
        Logic2.clocksource = clocksource::in2;
        Logic3.edgedetect = edgedetect::enable;
        Logic3.filter = filter::sync;
        Logic3.input0 = in::input_pullup;
        Logic3.input1 = in::input_pullup;
        Logic3.input2 = in::event_b;
        Logic3.clocksource = clocksource::in2;
        Logic3.output = out::enable;
        // now setup the truth tables
        // So we should trigger an enable on all falling edges so
        // 0b00010001 would be the standard setup so
        // in2, in1, in0
        // 0b000 -> 1 // we got a trigger of some kind
        // 0b001 -> 1 // we got a trigger of some kind
        // 0b010 -> 1 // we got a trigger of some kind
        // 0b011 -> 0 // we got no trigger at all
        // 0b100 -> 1 // we got a trigger of some kind
        // 0b101 -> 1 // we got a trigger of some kind
        // 0b110 -> 1 // we got a trigger of some kind
        // 0b111 -> 0 // we got no trigger at all
        Logic0.truth = 0b01110111;
        Logic1.truth = 0b01110111;
        Logic2.truth = 0b01110111;
        Logic3.truth = 0b01110111;
        // okay this starts up at the end
        Logic0.init();
        Logic1.init();
        Logic2.init();
        Logic3.init();
        Event0.start();
        Event1.start();
    }
    if constexpr (currentConfiguration.debugConsoleActive()) {
        DebugConsole.println("PIC Configured!");
    }
}
// the setup routine runs once when you press reset:
void setup() {
    configureClockSource();
    // the booted pin is the reset pin conceptually
    BootedPin ::configure();
    BootedPin ::assertPin();
    delay(2000);
    if constexpr (currentConfiguration.debugConsoleActive()) {
        DebugConsole.swap(1);
        DebugConsole.begin(9600);
        if constexpr (currentConfiguration.waitForSerialConnect()) {
            while (!DebugConsole) {
                delay(1);
            }
        }
        DebugConsole.println("i960 Management Engine");
    }
    setupPins();
    if constexpr (currentConfiguration.enableCommunicationChannel()) {
        CommunicationChannel.swap(1);
        // okay so activate interrupt sources if it makes sense
        /// @todo implement support for communication channel interrupts
    }
    if constexpr (currentConfiguration.hasPICBuiltin()) {
        configurePIC();
    }
    if constexpr (currentConfiguration.enableReadyPulseMode()) {
        while (true) {
            ReadySyncPin::pulse();
        }
    }
    if constexpr (currentConfiguration.enableTestReadyPinMode()) {
        while (true) {
            DebugConsole.print("Ready Input Pin Value: ");
            DebugConsole.println(ReadyInputPin::read());
        }
    }
    // no need to wait for the chipset to release control
    BootedPin::deassertPin();
    while (FailPin::inputLow()) {
        if (DenPin::inputAsserted()) {
            if constexpr (currentConfiguration.debugConsoleActive()) {
                DebugConsole.println("DEN Already Asserted!");
            }
            break;
        }
    }
    while (FailPin::inputHigh()) {
        if (DenPin::inputAsserted()) {
            if constexpr (currentConfiguration.debugConsoleActive()) {
                DebugConsole.println("DEN Already Asserted!");
            }
            break;
        }
    }
    if constexpr (currentConfiguration.debugConsoleActive()) {
        DebugConsole.println("Boot successful signalling!");
    }
    BootSuccessfulPin::assertPin();
    // at this point, if we ever go from low to high again then we have a checksum failure
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::FAIL)), handleChecksumFail, RISING);
    if constexpr (currentConfiguration.debugConsoleActive()) {
        DebugConsole.println("Interrupt attached!");
    }
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
    if constexpr (currentConfiguration.debugConsoleActive()) {
        DebugConsole.println("Entering Loop!");
    }
    for (;;) {
        if constexpr (currentConfiguration.debugConsoleActive()) {
            DebugConsole.println("Loop Top!");
        }
        // introduce some delay to make sure the bus has time to recover properly
        waitOneBusCycle();
        // okay so we need to wait for DEN to go low
        while (DenPin::inputDeasserted());
        if constexpr (currentConfiguration.debugConsoleActive()) {
            DebugConsole.println("In Data Cycle!");
        }

        if (numCycles >= currentConfiguration.getMaxNumberOfCyclesBeforePause()) {
            if constexpr (currentConfiguration.debugConsoleActive()) {
                DebugConsole.println("Pausing!");
            }
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
            if constexpr (currentConfiguration.debugConsoleActive()) {
                DebugConsole.println("Asserting InTransaction!");
            }
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
               if constexpr (currentConfiguration.debugConsoleActive()) {
                   DebugConsole.println("Asserting DoCycle!");
               }
               DoCyclePin :: assertPin();
               // we have entered a new transaction so increment the counter
               // we want to count the number of transaction cycles
               ++numCycles;
               // now wait for the chipset to tell us that it has satisifed the current part of the transaction
               if (BlastPin::inputAsserted()) {
                   if constexpr (currentConfiguration.debugConsoleActive()) {
                       DebugConsole.println("Burst Last!");
                   }
                   // if it turns out that blast is asserted then we break out of this loop and handle it specially
                   break;
               }
               // we are dealing with a burst transaction at this point
               waitForCycleEnd();
               // let the chipset know that the operation will continue
               {
                   if constexpr (currentConfiguration.debugConsoleActive()) {
                       DebugConsole.println("Assert Burst Next and Wait!");
                   }
                   BurstNext::assertPin();
                   informCPUAndWait();
                   BurstNext::deassertPin();
                   if constexpr (currentConfiguration.debugConsoleActive()) {
                       DebugConsole.println("Waiting for next cycle!");
                   }
               }
           }
           // the end of the current transaction needs to be straighline code
           waitForCycleEnd();
           // okay tell the chipset transaction complete
            if constexpr (currentConfiguration.debugConsoleActive()) {
                DebugConsole.println("Transaction Complete!");
            }
            InTransactionPin :: deassertPin();
        }
        if constexpr (currentConfiguration.debugConsoleActive()) {
            DebugConsole.println("Tell CPU about last part of transaction!");
        }
        // we have to tie off the transaction itself first
        // let the i960 know and then wait for the chipset to pull MCU READY high
        informCPUAndWait();
        // to make sure the bus has time to recover we can introduce an i960 bus cycle worth of delay
        waitOneBusCycle();
    }
}

