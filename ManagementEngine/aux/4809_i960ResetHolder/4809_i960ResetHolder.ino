// Target Chip: Atmega 4809 (DIP40)
// Target Platform: MegaCoreX
// Clockspeed: 20MHz internal oscillator
// PF6 is reset
// No bootloader
// UPDI is connected to Serial1's alternate pinout
// this sketch just holds the i960 in reset, sets up the aux pins and then releases the reset pin 2 seconds after finishing everything else.
// the serial console is totally optional and could be easily removed if needed.
//
// The objective of this program is to eliminate pins
constexpr auto EnableExternalClockSource = false;
constexpr auto EnableClockOutputPort = true;

constexpr auto HLDA960 = PIN_PA0;
constexpr auto HOLD960 = PIN_PA1;
constexpr auto INT0_960 = PIN_PA2;
constexpr auto INT1_960 = PIN_PA3;
constexpr auto INT2_960 = PIN_PA4;
constexpr auto INT3_960 = PIN_PA5;
constexpr auto Reset960 = PIN_PA6;
constexpr auto CLK960 = PIN_PA7;

constexpr auto FAIL960 = PIN_PD6;
constexpr auto MCU_READY = PIN_PD7;

constexpr auto DO_CYCLE = PIN_PE0;
constexpr auto IN_TRANSACTION = PIN_PE1;
constexpr auto SYSTEMBOOT = PIN_PE2;
constexpr auto BURST_NEXT = PIN_PE3;

constexpr auto WAITBOOT960 = PIN_PF0;
constexpr auto READY960 = PIN_PF1;
constexpr auto AS = PIN_PF2;
constexpr auto LOCK960 = PIN_PF3;
constexpr auto BLAST = PIN_PF4;
constexpr auto DEN = PIN_PF5;


// unused pins
constexpr auto Unused0 = PIN_PC0;
constexpr auto Unused1 = PIN_PC1;
constexpr auto Unused2 = PIN_PC2;
constexpr auto Unused3 = PIN_PC3;
constexpr auto Unused4 = PIN_PD0;
constexpr auto Unused5 = PIN_PD1;
constexpr auto Unused6 = PIN_PD2;
constexpr auto Unused7 = PIN_PD3;
constexpr auto Unused8 = PIN_PD4;
constexpr auto Unused9 = PIN_PD5;


template<decltype(WAITBOOT960) pin>
struct DigitalPin {
  DigitalPin() = delete;
  ~DigitalPin() = delete;
  DigitalPin(const DigitalPin&) = delete;
  DigitalPin(DigitalPin&&) = delete;
  DigitalPin& operator=(const DigitalPin&) = delete;
  DigitalPin& operator=(DigitalPin&&) = delete;
  static constexpr bool isInputPin() noexcept {
    return false;
  }
  static constexpr bool isOutputPin() noexcept {
    return false;
  }
  static constexpr auto isInputPullupPin() noexcept {
    return false;
  }
  static constexpr bool getDirection() noexcept {
    return false;
  }
  static constexpr auto getPin() noexcept {
    return pin;
  }
};

#define DefOutputPin(pin, asserted, deasserted) \
  template<> \
  struct DigitalPin< pin > { \
    static_assert(asserted != deasserted, "Asserted and deasserted must not be equal!"); \
    DigitalPin() = delete; \
    ~DigitalPin() = delete; \
    DigitalPin(const DigitalPin&) = delete; \
    DigitalPin(DigitalPin&&) = delete; \
    DigitalPin& operator=(const DigitalPin&) = delete; \
    DigitalPin& operator=(DigitalPin&&) = delete; \
    static constexpr auto isInputPin() noexcept { return false; } \
    static constexpr auto isOutputPin() noexcept { return true; } \
    static constexpr auto isInputPullupPin() noexcept { return false; } \
    static constexpr auto getPin() noexcept { return pin; } \
    static constexpr auto getDirection() noexcept { return OUTPUT; } \
    static constexpr auto getAssertionState() noexcept { return asserted; } \
    static constexpr auto getDeassertionState() noexcept { return deasserted; }      \
    [[gnu::always_inline]] inline static void assertPin() noexcept { digitalWriteFast(pin, getAssertionState()); } \
    [[gnu::always_inline]] inline static void deassertPin() noexcept { digitalWriteFast(pin,getDeassertionState()); } \
    [[gnu::always_inline]] inline static void write(decltype(LOW) value) noexcept { digitalWriteFast(pin,value); } \
    [[gnu::always_inline]] inline static void pulse() noexcept {  assertPin(); __builtin_avr_nops(2); deassertPin(); } \
    [[gnu::always_inline]] static void configure() noexcept { pinMode(pin, OUTPUT); } \
  }

#define DefInputPin(pin, asserted, deasserted) \
  template<> \
  struct DigitalPin< pin > { \
    static_assert(asserted != deasserted, "Asserted and deasserted must not be equal!"); \
    DigitalPin() = delete; \
    ~DigitalPin() = delete; \
    DigitalPin(const DigitalPin&) = delete; \
    DigitalPin(DigitalPin&&) = delete; \
    DigitalPin& operator=(const DigitalPin&) = delete; \
    DigitalPin& operator=(DigitalPin&&) = delete; \
    static constexpr auto isInputPin() noexcept { return true; } \
    static constexpr auto isOutputPin() noexcept { return false; } \
    static constexpr auto isInputPullupPin() noexcept { return false; } \
    static constexpr auto getPin() noexcept { return pin; } \
    static constexpr auto getDirection() noexcept { return INPUT; } \
    static constexpr auto getAssertionState() noexcept { return asserted; } \
    static constexpr auto getDeassertionState() noexcept { return deasserted; } \
    [[gnu::always_inline]] inline static auto read() noexcept { return digitalReadFast(pin); } \
    [[gnu::always_inline]] inline static bool isAsserted() noexcept { return read() == getAssertionState(); } \
    [[gnu::always_inline]] inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); } \
    [[gnu::always_inline]] static void configure() noexcept { pinMode(pin, INPUT); }     \
    [[gnu::always_inline]] inline static auto isLow() noexcept { return read() == LOW; } \
    [[gnu::always_inline]] inline static auto isHigh() noexcept { return read() == HIGH; } \
  }

#define DefInputPullupPin(pin, asserted, deasserted) \
  template<> \
  struct DigitalPin< pin > { \
    static_assert(asserted != deasserted, "Asserted and deasserted must not be equal!"); \
    DigitalPin() = delete; \
    ~DigitalPin() = delete; \
    DigitalPin(const DigitalPin&) = delete; \
    DigitalPin(DigitalPin&&) = delete; \
    DigitalPin& operator=(const DigitalPin&) = delete; \
    DigitalPin& operator=(DigitalPin&&) = delete; \
    static constexpr auto isInputPin() noexcept { return true; } \
    static constexpr auto isOutputPin() noexcept { return false; }  \
    static constexpr auto isInputPullupPin() noexcept { return true; } \
    static constexpr auto getPin() noexcept { return pin; } \
    static constexpr auto getDirection() noexcept { return INPUT_PULLUP; } \
    static constexpr auto getAssertionState() noexcept { return asserted; } \
    static constexpr auto getDeassertionState() noexcept { return deasserted; } \
    [[gnu::always_inline]] inline static auto read() noexcept { return digitalReadFast(pin); } \
    [[gnu::always_inline]] inline static bool isAsserted() noexcept { return read() == getAssertionState(); } \
    [[gnu::always_inline]] inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); } \
    [[gnu::always_inline]] static void configure() noexcept { pinMode(pin, INPUT_PULLUP); } \
    [[gnu::always_inline]] inline static auto isLow() noexcept { return read() == LOW; } \
    [[gnu::always_inline]] inline static auto isHigh() noexcept { return read() == HIGH; } \
  }

DefInputPin(DEN, LOW, HIGH);
DefInputPin(AS, LOW, HIGH);
DefInputPin(BLAST, LOW, HIGH);

DefInputPin(FAIL960, HIGH, LOW);
DefInputPin(HLDA960, HIGH, LOW);

DefOutputPin(Reset960, LOW, HIGH);
DefOutputPin(READY960, LOW, HIGH);
DefOutputPin(LOCK960, LOW, HIGH);
DefOutputPin(HOLD960, HIGH, LOW);

DefOutputPin(INT0_960, LOW, HIGH);
DefOutputPin(INT1_960, HIGH, LOW);
DefOutputPin(INT2_960, HIGH, LOW);
DefOutputPin(INT3_960, LOW, HIGH);
DefOutputPin(SYSTEMBOOT, HIGH, LOW);
DefOutputPin(IN_TRANSACTION, LOW, HIGH);
DefOutputPin(DO_CYCLE, LOW, HIGH);
DefOutputPin(BURST_NEXT, HIGH, LOW);

DefInputPullupPin(WAITBOOT960, LOW, HIGH);
DefInputPullupPin(MCU_READY, LOW, HIGH);
#undef DefInputPin
#undef DefOutputPin
#undef DefInputPullupPin

template<decltype(WAITBOOT960) pin>
struct PinAsserter final {
  static_assert(DigitalPin<pin>::isOutputPin(), "Pin asserting only works with output pins!");
  PinAsserter() noexcept {
    DigitalPin<pin>::assertPin();
  }
  ~PinAsserter() noexcept {
    DigitalPin<pin>::deassertPin();
  }
};

[[noreturn]] void
handleChecksumFail() noexcept {
  // keep an eye on the FAIL960 pin, if we run into an issue then tell the chipset this
  DigitalPin<SYSTEMBOOT>::deassertPin();
  while (true) {
    delay(1000);
  }
}

void configureClockSource() noexcept {
  byte clkBits = 0;
  if constexpr (EnableExternalClockSource) {
    clkBits |= 0b0000'0011;
             }
               if constexpr (EnableClockOutputPort) {
               clkBits |= 0b1000'0000;
  }
  // setup all of the pins that can affect the i960 itself
  if constexpr (EnableExternalClockSource || EnableClockOutputPort) {
    CCP = 0xD8;
    CLKCTRL.MCLKCTRLA = clkBits; // enable the external 20mhz clock source just to be on the safe side
    CCP = 0xD8;
  }
  if constexpr (EnableClockOutputPort) {
    CCP = 0xD8;
    CLKCTRL.OSC20MCTRLA |= 0b00000010;
    CCP = 0xD8;
  }
}
template<int ... pins>
void configurePins() noexcept {
  (DigitalPin<pins>::configure() , ...);
}
template<int ... pins>
void deassertPins() noexcept {
  (DigitalPin<pins>::deassertPin(), ...);
}
volatile bool readyTriggered = false;
void handleREADY() noexcept {
  readyTriggered = true;
}
void setup() {
  configureClockSource();
  // always pull this low to start
  DigitalPin<Reset960>::configure();
  {
    PinAsserter<Reset960> holdI960InReset;
    DigitalPin<WAITBOOT960>::configure();
    delay(2000);
    //Serial1.swap(1);
    //Serial1.begin(9600);
    //Serial1.println("Bringing everything up!");
    configurePins<DEN,
                  HLDA960,
                  FAIL960,
                  AS,
                  BLAST,
                  MCU_READY,
                  READY960,
                  LOCK960,
                  HOLD960,
                  INT0_960,
                  INT1_960,
                  INT2_960,
                  INT3_960,
                  SYSTEMBOOT,
                  IN_TRANSACTION,
                  DO_CYCLE,
                  BURST_NEXT>();

    deassertPins<BURST_NEXT,
                 DO_CYCLE,
                 IN_TRANSACTION,
                 LOCK960,
                 HOLD960,
                 INT0_960,
                 INT1_960,
                 INT2_960,
                 INT3_960,
                 SYSTEMBOOT,
                 READY960>();
    while (DigitalPin<WAITBOOT960>::isAsserted());
  }

  while (DigitalPin<FAIL960>::isLow()) {
    if (DigitalPin<DEN>::isAsserted()) {
      break;
    }
  }
  while (DigitalPin<FAIL960>::isHigh()) {
    if (DigitalPin<DEN>::isAsserted()) {
      break;
    }
  }
  DigitalPin<SYSTEMBOOT>::assertPin();
  //Serial1.println("SYSTEM BOOTED!");
  // after this point, if FAIL960 ever goes from LOW to HIGH again, then we have checksum failed!
  attachInterrupt(digitalPinToInterrupt(FAIL960), handleChecksumFail, RISING);
}
constexpr bool EnableOneCycleWaitStates = false;
template<bool enable = EnableOneCycleWaitStates>
[[gnu::always_inline]]
inline void waitOneBusCycle() noexcept {
  if constexpr (enable) {
    asm volatile ("nop");
    asm volatile ("nop");
  }
}
[[gnu::always_inline]]
inline void informCPUAndWait() noexcept {
  DigitalPin<READY960>::pulse();
  while (DigitalPin<MCU_READY>::isAsserted());
  waitOneBusCycle();
}

[[gnu::always_inline]]
inline void waitForCycleEnd() noexcept {
  while (DigitalPin<MCU_READY>::isDeasserted());
  // at this point the chipset will wait around until we end this cycle
  // so we can use this time to clear state of pins
  // we want to make sure that we deassert BURST_NEXT ahead of time
  DigitalPin<BURST_NEXT>::deassertPin();
  DigitalPin<DO_CYCLE>::deassertPin();
  // at this point we are outside a bus cycle
  waitOneBusCycle();
}
constexpr byte MaxNumberOfCyclesBeforePause = 64;
volatile byte numCycles = 0;
[[noreturn]]
void loop() {
  for (;;) {
    // introduce some delay to make sure the bus has time to recover properly
    waitOneBusCycle();
    // okay so we need to wait for AS and DEN to go low
    while (DigitalPin<DEN>::isDeasserted());
    if (numCycles >= MaxNumberOfCyclesBeforePause) {
      // Provide a pause/cooldown period after a new data request to make sure
      // that the bus has time to "cool". If you don't have this then the 
      // i960 can chipset fault when it shouldn't... very strange
      while (numCycles > 0) {
        // use the loop itself to provide some amount of time to cool off
        // numTransactions is volatile to prevent the compiler from optimizing any of 
        // this away.
        --numCycles;
      }
    }
    // now do the logic
    {
      PinAsserter<IN_TRANSACTION> transactionEnter; // tell the chipset that we are starting a transaction
      // okay now we need to emulate the wait loop
      for (;;) {
        // instead of pulsing do cycle, we just assert do cycle while we wait
        if (DigitalPin<BLAST>::isAsserted()) {
          // BLAST being low means that we set BURST NEXT LOW
          DigitalPin<BURST_NEXT>::deassertPin();
        } else {
          DigitalPin<BURST_NEXT>::assertPin();
        }

        // okay now start the cycle
        DigitalPin<DO_CYCLE>::assertPin();
        // we have entered a new transaction (burst or non) so increment the counter
        // we want to count the number of cycles
        ++numCycles;
        // now wait for the chipset to tell us it has satisified the current part of the transaction
        if (DigitalPin<BLAST>::isAsserted()) {
          // break out of the current loop if we are in the last transaction
          break;
        }
        // we are dealing with a burst transaction at this point
        waitForCycleEnd();
        {
          // let the chipset know that the operation will continue
          PinAsserter<BURST_NEXT> letChipsetKnowBurstNext;
          // let the i960 know and then wait for the chipset to pull MCU READY high
          informCPUAndWait();
        }
      }
      // the end of the current transaction needs to be straightline code
      waitForCycleEnd();
    }
    // let the i960 know and then wait for chipset to pull MCU READY high
    informCPUAndWait();
    // to make sure the bus has time to recover we need to introduce an i960 bus cycle worth of delay
    waitOneBusCycle();
  }
}
