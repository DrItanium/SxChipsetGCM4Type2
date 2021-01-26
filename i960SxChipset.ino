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

/// i960Sx chipset mcu, based on atmega1284p with fuses set for:
/// - 20Mhz crystal
/// - D1 acts as CLKO
/// Language options:
/// - C++17
/// Board Platform: MightyCore
#include <SPI.h>
#include <Wire.h>
#include <libbonuspin.h>
#include <Adafruit_SI5351.h>
#include <PCF8523.h>
#include <Arduino_JSON.h>
#include <SD.h>
#include <OPL3Duo.h>
#include <Adafruit_BLE.h> 
Adafruit_SI5351 clockgen;
volatile bool clockgenActive = false;
PCF8523 rtc;
volatile bool rtcActive = false;
OPL3Duo opl3;
volatile bool opl3Active = false;
template<typename T>
class TreatAs final {
	public:
		using ReturnType = T;
};
using Address = uint32_t;
using TreatAsByte = TreatAs<uint8_t>;
using TreatAsShort = TreatAs<uint16_t>;
using TreatAsWord = TreatAs<uint32_t>;

enum class i960Pinout : decltype(A0) {
// PORT B
	Led = 0, 	  // output
  CLOCK_OUT, // output, unusable
	Reset960,     // output
	AVR_INT2, 	  // AVR Interrupt  INT2
	Ready,		  // output
	GPIOSelect,   // output
	MOSI,		  // reserved
	MISO,		  // reserved
	SCK, 		  // reserved
// PORT D
	RX0, 		  // reserved
	TX0, 		  // reserved
	AVR_INT0,	  // AVR Interrupt INT0
	AVR_INT1,	  // AVR Interrupt INT1
	PWM0,		  // unused
	PWM1, 		  // unused
	PWM2, 		  // unused
	PWM3, 		  // unused
// PORT C
	SCL,		  // reserved
	SDA, 		  // reserved
	W_R, 		  // input
	Hold,		  // output
	HLDA,         // input
	ALE,          // input
	ResetGPIO,    // output
	AS_, 		  // input
// PORT A
	Lock_,		  // bidirectional but default to output
	Int0_,	      // output 
	DT_R, 		  // input
	DEN_, 		  // input
	NC0, 		  // unused
	NC1,         // unused
	NC2, 	   // unused
	Count,		   // special
};
enum class IOExpanderAddress : byte {
	DataLines = 0b000,
	Lower16Lines,
	Upper16Lines,
	MemoryCommitExtras,
	OtherDevice0,
	OtherDevice1,
	OtherDevice2,
	OtherDevice3,
};

static_assert(static_cast<decltype(HIGH)>(i960Pinout::Count) <= 32);

template<IOExpanderAddress addr>
using IOExpander = bonuspin::MCP23S17<static_cast<int>(addr),
	  static_cast<int>(i960Pinout::GPIOSelect),
	  static_cast<int>(i960Pinout::ResetGPIO)>;

inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
	digitalWrite(static_cast<int>(ip), value);
}

inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
	pinMode(static_cast<int>(ip), value);
}

template<typename ... Pins>
void setupPins(decltype(OUTPUT) direction, Pins ... pins) {
	(pinMode(pins, direction), ...);
}

template<typename ... Pins>
void digitalWriteBlock(decltype(HIGH) value, Pins ... pins) {
	(digitalWrite(pins, value), ...);
}

template<i960Pinout pinId, decltype(HIGH) onConstruction, decltype(LOW) onDestruction>
class PinToggler {
	public:
		PinToggler() { digitalWrite(pinId, onConstruction); }
		~PinToggler() { digitalWrite(pinId, onDestruction); }
};


template<i960Pinout pinId>
using HoldPinLow = PinToggler<pinId, LOW, HIGH>;

template<i960Pinout pinId>
using HoldPinHigh = PinToggler<pinId, HIGH, LOW>;

// 8 IOExpanders to a single enable line for SPI purposes 
// 3 of them are reserved
IOExpander<IOExpanderAddress::DataLines> dataLines;
IOExpander<IOExpanderAddress::Lower16Lines> lower16;
IOExpander<IOExpanderAddress::Upper16Lines> upper16;
IOExpander<IOExpanderAddress::MemoryCommitExtras> extraMemoryCommit;

// upper five are actually unused but could be through these devices
// this would give us 80 pins to work with for other purposes
IOExpander<IOExpanderAddress::OtherDevice0> dev0;
IOExpander<IOExpanderAddress::OtherDevice1> dev1;
IOExpander<IOExpanderAddress::OtherDevice2> dev2;
IOExpander<IOExpanderAddress::OtherDevice3> dev3;

uint32_t
getAddress() noexcept {
	auto lower16Addr = static_cast<uint32_t>(lower16.readGPIOs());
	auto upper16Addr = static_cast<uint32_t>(upper16.readGPIOs()) << 16;
	return lower16Addr | upper16Addr;
}
uint16_t
getDataBits() noexcept {
	dataLines.writeGPIOsDirection(0);
	return static_cast<uint16_t>(dataLines.readGPIOs());
}

void
setDataBits(uint16_t value) noexcept {
	dataLines.writeGPIOsDirection(0xFFFF);
	dataLines.writeGPIOs(value);
}

uint8_t getByteEnableBits() noexcept {
	return (extraMemoryCommit.readGPIOs() & 0b11);
}

uint8_t getBurstAddress() noexcept {
	return (extraMemoryCommit.readGPIOs() & 0b11100) >> 2;
}

bool isBurstLast() noexcept {
	return (extraMemoryCommit.readGPIOs() & 0b100000);
}
uint8_t 
load(Address address, TreatAsByte) noexcept {
	return 0;
}
uint16_t
load(Address address, TreatAsShort) noexcept {
	return 0;
}
void setupCPUInterface() {
	setupPins(OUTPUT,
			i960Pinout::Ready,
			i960Pinout::GPIOSelect,
			i960Pinout::Lock_,
			i960Pinout::Int0_,
			i960Pinout::Hold);
	digitalWriteBlock(HIGH,
			i960Pinout::Ready,
			i960Pinout::GPIOSelect,
			i960Pinout::Lock_,
			i960Pinout::Int0_);
	digitalWrite(i960Pinout::Hold, LOW);
	setupPins(INPUT,
			i960Pinout::ALE,
			i960Pinout::AS_,
			i960Pinout::DT_R,
			i960Pinout::DEN_,
			i960Pinout::W_R,
			i960Pinout::HLDA);
}
void setupIOExpanders() {
	dataLines.begin();
	lower16.begin();
	upper16.begin();
	extraMemoryCommit.begin();
	dataLines.reset();
	lower16.reset();
	upper16.reset();
	extraMemoryCommit.reset();
	lower16.writeGPIOsDirection(0);
	upper16.writeGPIOsDirection(0);
	extraMemoryCommit.writeGPIOsDirection(0);
}
void setupClockGenerator() {
	if (clockgen.begin() != ERROR_NONE) {
		Serial.println("Ooops, no Si5351 detected ... skipping!");
		clockgenActive = false;
		return;
	}
	Serial.println("Si5351 was detected successfully!");
	clockgenActive = true;
}
	
// the setup routine runs once when you press reset:
void setup() {
	Serial.begin(115200);
	Serial.println("80960Sx Chipset Starting up...");
	setupPins(OUTPUT, 
			i960Pinout::ResetGPIO, 
			i960Pinout::Reset960,
			i960Pinout::Led);
	HoldPinLow<i960Pinout::Reset960> holdi960InReset;
	HoldPinLow<i960Pinout::ResetGPIO> gpioReset;
	setupCPUInterface();
	SPI.begin();
	setupIOExpanders();
	setupClockGenerator();
	/// wait two seconds to ensure that reset is successful
	delay(2000);
}

// the loop routine runs over and over again forever:
void loop() {
	digitalWrite(i960Pinout::Led, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(1000);               // wait for a second
	digitalWrite(i960Pinout::Led, LOW);    // turn the LED off by making the voltage LOW
	delay(1000);               // wait for a second
}
