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
#include <libbonuspin.h>
#include <Fsm.h>
#include <Timer.h>
#include <SD.h>
#include <Wire.h>

#include <ArduinoJson.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "Pinout.h"
#include "Device.h"
#include "RAM.h"
#include "SPIBus.h"
#include "IOExpanders.h"
#include "PSRAM64H.h"



/**
 * Normally generated by the linker as the value used for validation purposes
 * on system bootup. Since we are on a microcontroller, this can be done
 * dynamically. This is taken from start.ld in the SA/SB Reference manual's
 * Appendix D.
 */
constexpr auto computeCS1(uint32_t satPtr, uint32_t pcrbPtr, uint32_t startIP) noexcept {
	return - (satPtr + pcrbPtr + startIP);
}
Timer t;
Adafruit_ILI9341 tft(static_cast<int>(i960Pinout::DISPLAY_EN),
                     static_cast<int>(i960Pinout::DC));
// boot rom and sd card loading stuff
Sd2Card theBootSDCard;
SdVolume theBootVolume;
SdFile rootDirectory;
SdFile theBootROM;
// the upper 64 elements of the bus are exposed for direct processor usage
union WordEntry {
    byte bytes[2];
    uint16_t word;
};
constexpr auto OnBoardSRAMCacheSizeInBytes = 4096;
constexpr auto OnBoardSRAMCacheSize = OnBoardSRAMCacheSizeInBytes / sizeof (WordEntry);
/**
 * @brief Allocate a portion of on board sram as accessible to the i960 without having to walk out onto the separate busses
 */
volatile WordEntry OnBoardSRAMCache[OnBoardSRAMCacheSize];

static constexpr Address RamStartingAddress = 0x8000'0000;
static constexpr auto FlashStartingAddress = 0x0000'0000;

// with the display I want to expose a 16 color per pixel interface. Each specific function needs to be exposed
// set a single pixel in the display, storage area
// we map these pixel values to specific storage cells on the microcontroller
// A four address is used to act as a door bell!
// storage area for the display + a doorbell address as well
// the command is setup to send off to the display
template<typename DisplayType>
class DisplayCommand {
public:
    /// @todo extend Device and take in a base address
    DisplayCommand(DisplayType& display) : display_(display) { }
    [[nodiscard]] constexpr auto getCommand() const noexcept { return command_; }
    [[nodiscard]] constexpr auto getX() const noexcept { return x_; }
    [[nodiscard]] constexpr auto getY() const noexcept { return y_; }
    [[nodiscard]] constexpr auto getW() const noexcept { return w_; }
    [[nodiscard]] constexpr auto getH() const noexcept { return h_; }
    [[nodiscard]] constexpr auto getRadius() const noexcept { return radius_; }
    [[nodiscard]] constexpr uint16_t getColor() const noexcept { return color_; }
    [[nodiscard]] constexpr auto getX0() const noexcept { return x0_; }
    [[nodiscard]] constexpr auto getY0() const noexcept { return y0_; }
    [[nodiscard]] constexpr auto getX1() const noexcept { return x1_; }
    [[nodiscard]] constexpr auto getY1() const noexcept { return y1_; }
    [[nodiscard]] constexpr auto getX2() const noexcept { return x2_; }
    [[nodiscard]] constexpr auto getY2() const noexcept { return y2_; }
    void setCommand(uint16_t command) noexcept { command_ = command; }
    void setX(int16_t x) noexcept { x_ = x; }
    void setY(int16_t y) noexcept { y_ = y; }
    void setW(int16_t w) noexcept { w_ = w; }
    void setH(int16_t h) noexcept { h_ = h; }
    void setRadius(int16_t radius) noexcept { radius_ = radius; }
    void setColor(uint16_t color) noexcept { color_ = color; }
    void setX0(int16_t value) noexcept { x0_ = value; }
    void setY0(int16_t value) noexcept { y0_ = value; }
    void setX1(int16_t value) noexcept { x1_ = value; }
    void setY1(int16_t value) noexcept { y1_ = value; }
    void setX2(int16_t value) noexcept { x2_ = value; }
    void setY2(int16_t value) noexcept { y2_ = value; }
    const DisplayType& getAssociatedDisplay() const noexcept { return display_; }
    DisplayType& getAssociatedDisplay() noexcept { return display_; }
    /**
     * @brief Invoke on doorbell write
     * @param value the value written to the doorbell
     */
    void invoke(uint16_t value) {
        // perhaps we'll do nothing with the value but hold onto it for now
    }
private:
    DisplayType& display_;
    uint16_t command_ = 0;
    int16_t x_ = 0;
    int16_t y_ = 0;
    int16_t w_ = 0;
    int16_t h_ = 0;
    int16_t radius_ = 0;
    uint16_t color_ = 0;
    int16_t x0_ = 0;
    int16_t y0_ = 0;
    int16_t x1_ = 0;
    int16_t y1_ = 0;
    int16_t x2_ = 0;
    int16_t y2_ = 0;
};

DisplayCommand<decltype(tft)> displayCommandSet(tft);


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
// Tr - Recovery State
// Tw - Wait State
// TChecksumFailure - Checksum Failure State

// READY - ~READY asserted
// NOT READY - ~READY not asserted
// BURST - ~BLAST not asserted
// NO BURST - ~BLAST asserted
// NEW REQUEST - ~AS asserted
// NO REQUEST - ~AS not asserted when in 

// Ti -> Ti via no request
// Tr -> Ti via no request
// Tr -> Ta via request pending
// Ti -> Ta via new request
// on enter of Ta, set address state to false
// on enter of Td, burst is sampled
// Ta -> Td
// Td -> Tr after signaling ready and no burst (blast low)
// Td -> Td after signaling ready and burst (blast high)
// Ti -> TChecksumFailure if FAIL is asserted
// Tr -> TChecksumFailure if FAIL is asserted

// NOTE: Tw may turn out to be synthetic
volatile bool asTriggered = false;
volatile bool denTriggered = false;
volatile uint32_t baseAddress = 0;
volatile bool performingRead = false;
constexpr auto NoRequest = 0;
constexpr auto NewRequest = 1;
constexpr auto ReadyAndBurst = 2;
constexpr auto NotReady = 3;
constexpr auto ReadyAndNoBurst = 4;
constexpr auto RequestPending = 5;
constexpr auto ToDataState = 6;
constexpr auto PerformSelfTest = 7;
constexpr auto SelfTestComplete = 8;
constexpr auto ChecksumFailure = 9;
void startupState() noexcept;
void systemTestState() noexcept;
void idleState() noexcept;
void doAddressState() noexcept;
void processDataRequest() noexcept;
void doRecoveryState() noexcept;
void enteringDataState() noexcept;
State tStart(nullptr, startupState, nullptr);
State tSystemTest(nullptr, systemTestState, nullptr);
Fsm fsm(&tStart);
State tIdle(nullptr,
		idleState, 
		nullptr);
State tAddr([]() { asTriggered = false; }, 
		doAddressState, 
		nullptr);
State tData(enteringDataState, 
		processDataRequest, 
		nullptr);
State tRecovery(nullptr,
		doRecoveryState,
		nullptr);
State tChecksumFailure(nullptr, nullptr, nullptr);


void startupState() noexcept {
	if (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
		fsm.trigger(PerformSelfTest);
	}
}
void systemTestState() noexcept {
    if (DigitalPin<i960Pinout::FAIL>::isDeasserted()) {
		fsm.trigger(SelfTestComplete);
	}
}
void onASAsserted() {
    asTriggered = true;
}
void onDENAsserted() {
    denTriggered = true;
}

void idleState() noexcept {
	if (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
		fsm.trigger(ChecksumFailure);
	} else {
		if (asTriggered) {
			fsm.trigger(NewRequest);
		}
	}
}
void doAddressState() noexcept {
	if (denTriggered) {
		fsm.trigger(ToDataState);
	}
}



void
enteringDataState() noexcept {
	// when we do the transition, record the information we need
	denTriggered = false;
	baseAddress = getAddress();
	performingRead = isReadOperation();
}
LoadStoreStyle getStyle() noexcept { return static_cast<LoadStoreStyle>(getByteEnableBits()); }

void
performWrite(Address address, uint16_t value) noexcept {
    Serial.print(F("Write 0x"));
    Serial.print(value, HEX);
    Serial.print(F(" to 0x"));
    Serial.println(address, HEX);
    if (address < RamStartingAddress) {
        // we are in program land for the time being so do nothing!
    } else {
        // upper half needs to be walked down further
        if (address >= 0xFF00'0000) {
            // cpu internal space, we should never get to this location
            Serial.println(F("Request to write into CPU internal space"));
        } else if ((address >= 0xFE00'0000) && (address < 0xFF00'0000)) {
            // this is the internal IO space
            Serial.println(F("Request to write into IO space"));
        }
    }
    /// @todo implement
}
uint16_t
performRead(Address address) noexcept {
    Serial.print(F("Read from 0x"));
    Serial.println(address, HEX);
    /// @todo implement
    if (address < RamStartingAddress) {
        // write to flash
    } else {
        // upper half needs to be walked down further
        if (address >= 0xFF00'0000) {
            // cpu internal space, we should never get to this location
            Serial.println(F("Request to read from CPU internal space?"));
        } else if ((address >= 0xFE00'0000) && (address < 0xFF00'0000)) {
            // this is the internal IO space
            Serial.println(F("Request to read from IO space"));
        }
    }
    return 0;
}
void processDataRequest() noexcept {
    auto burstAddress = getBurstAddress(baseAddress);
	if (performingRead) {
		setDataBits(performRead(burstAddress));
	} else {
	    performWrite(burstAddress, getDataBits());
	}
	// setup the proper address and emit this over serial
	auto blastPin = getBlastPin();
	DigitalPin<i960Pinout::Ready>::pulse();
	if (blastPin == LOW) {
		// we not in burst mode
		fsm.trigger(ReadyAndNoBurst);
	} 

}

void doRecoveryState() noexcept {
	if (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
		fsm.trigger(ChecksumFailure);
	} else {
		if (asTriggered) {
			fsm.trigger(RequestPending);
		} else {
			fsm.trigger(NoRequest);
		}
	}
}


// ----------------------------------------------------------------
// setup routines
// ----------------------------------------------------------------

void setupBusStateMachine() noexcept {
	fsm.add_transition(&tStart, &tSystemTest, PerformSelfTest, nullptr);
	fsm.add_transition(&tSystemTest, &tIdle, SelfTestComplete, nullptr);
	fsm.add_transition(&tIdle, &tAddr, NewRequest, nullptr);
	fsm.add_transition(&tIdle, &tChecksumFailure, ChecksumFailure, nullptr);
	fsm.add_transition(&tAddr, &tData, ToDataState, nullptr);
	fsm.add_transition(&tData, &tRecovery, ReadyAndNoBurst, nullptr);
	fsm.add_transition(&tRecovery, &tAddr, RequestPending, nullptr);
	fsm.add_transition(&tRecovery, &tIdle, NoRequest, nullptr);
	fsm.add_transition(&tRecovery, &tChecksumFailure, ChecksumFailure, nullptr);
	fsm.add_transition(&tData, &tChecksumFailure, ChecksumFailure, nullptr);
}
//State tw(nullptr, nullptr, nullptr); // at this point, this will be synthetic
//as we have no concept of waiting inside of the mcu
void setupCPUInterface() {
    Serial.println(F("Setting up interrupts!"));
	setupPins(OUTPUT,
			i960Pinout::Ready,
			i960Pinout::GPIOSelect,
			i960Pinout::Int0_);
	digitalWriteBlock(HIGH,
			i960Pinout::Ready,
			i960Pinout::GPIOSelect,
			i960Pinout::Int0_);
	setHOLDPin(LOW);
	setLOCKPin(HIGH);
	setupPins(INPUT,
			i960Pinout::BLAST_,
			i960Pinout::AS_,
			i960Pinout::W_R_,
			i960Pinout::DEN_,
			i960Pinout::FAIL);
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::AS_)), onASAsserted, FALLING);
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::DEN_)), onDENAsserted, FALLING);
    Serial.println(F("Done setting up interrupts!"));
}
void setupIOExpanders() {
    Serial.println(F("Setting up IOExpanders!"));
	// at bootup, the IOExpanders all respond to 0b000 because IOCON.HAEN is
	// disabled. We can send out a single IOCON.HAEN enable message and all
	// should receive it. 
	// so do a begin operation on all chips (0b000)
	dataLines.begin(); 
	// set IOCON.HAEN on all chips
	dataLines.enableHardwareAddressPins();
	// now we have to refresh our on mcu flags for each io expander
	lower16.refreshIOCon();
	upper16.refreshIOCon();
	extraMemoryCommit.refreshIOCon();
	// now all devices tied to this ~CS pin have separate addresses
	// make each of these inputs
	lower16.writeGPIOsDirection(0xFFFF);
	upper16.writeGPIOsDirection(0xFFFF);
	dataLines.writeGPIOsDirection(0xFFFF);
	// set lower eight to inputs and upper eight to outputs
	extraMemoryCommit.writeGPIOsDirection(0x00FF);
	// then indirectly mark the outputs
	pinMode(static_cast<int>(ExtraGPIOExpanderPinout::LOCK_), OUTPUT, extraMemoryCommit);
	pinMode(static_cast<int>(ExtraGPIOExpanderPinout::HOLD), OUTPUT, extraMemoryCommit);
	setupSPIBus();
    Serial.println(F("Done setting up io expanders!"));
}
void printMacAddress(byte mac[]) {
    for (int i = 5; i >= 0; --i) {
        if (mac[i] < 16) {
            Serial.print(F("0"));
        }
        Serial.print(mac[i], HEX);
        if (i > 0) {
            Serial.print(F(":"));
        }
    }
    Serial.println();
}

void setupSRAMCache() {
    // 8k on board cache
    for (uint32_t i = 0; i < OnBoardSRAMCacheSize; ++i) {
        OnBoardSRAMCache[i].word = 0;
    }
}
constexpr auto computeAddressStart(Address start, Address size, Address count) noexcept {
    return start + (size * count);
}
// we have access to 12 Winbond Flash Modules, which hold onto common program code, This gives us access to 96 megabytes of Flash.
// At this point it is a massive pain in the ass to program all these devices but who cares

void setupTFT() {
    tft.begin();
    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(0,0);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(3);
    tft.println(F("i960Sx!"));
}
[[noreturn]]
void signalHaltState() {
    Serial.println(F("HALTING"));
    digitalWrite(i960Pinout::Led, HIGH);
    while(true) {
        delay(1000);
    }
}
void setupSDCard() {
    if (!theBootSDCard.init(SPI_FULL_SPEED, static_cast<int>(i960Pinout::SD_EN))) {
        Serial.println(F("SD Card initialization failed"));
        Serial.println(F("Make sure of the following:"));
        Serial.println(F("1) Is an SD Card is inserted?"));
        Serial.println(F("2) Is the wiring is correct?"));
        Serial.println(F("3) Does the ~CS pin match the shield or module?"));
        signalHaltState();
    }
    Serial.println(F("SD Card initialization successful"));
    Serial.println();
    Serial.println(F("Card Info"));
    Serial.println(F("Card Type:\t"));
    switch (theBootSDCard.type()) {
        case SD_CARD_TYPE_SD1: Serial.println(F("SD1")); break;
        case SD_CARD_TYPE_SD2: Serial.println(F("SD2")); break;
        case SD_CARD_TYPE_SDHC: Serial.println(F("SDHC")); break;
        default: Serial.println(F("Unknown")); break;
    }
    if (!theBootVolume.init(theBootSDCard)) {
        Serial.println(F("Could not find a valid FAT16/FAT32 parition"));
        Serial.println(F("Make sure you've formatted the card!"));
        signalHaltState();
    }
    Serial.print(F("Clusters:\t"));
    Serial.println(theBootVolume.clusterCount());
    Serial.print(F("Blocks x Cluster:\t"));
    Serial.println(theBootVolume.blocksPerCluster());
    Serial.print(F("Total Blocks:\t"));
    Serial.println(theBootVolume.blocksPerCluster() * theBootVolume.clusterCount());
    Serial.println();

    Serial.print(F("Volume type is: FAT"));
    Serial.println(theBootVolume.fatType(), DEC);
    auto volumeSize = theBootVolume.blocksPerCluster(); // clusters are collections of blocks
    volumeSize *= theBootVolume.clusterCount(); // sd cards have many clusters
    volumeSize /= 2; // SD Card blocks are always 512 bytes
    Serial.print(F("Volume size (Kb):\t"));
    Serial.println(volumeSize);
    Serial.print(F("Volume size (Mb):\t"));
    Serial.println(volumeSize / 1024);
    Serial.print(F("Volume size (Gb):\t"));
    Serial.println(static_cast<float>(volumeSize / 1024 / 1024));

    Serial.println();
    Serial.println(F("Files found on the card (name, date and size in bytes): "));
    rootDirectory.openRoot(theBootVolume);
    rootDirectory.ls(LS_R | LS_DATE | LS_SIZE);
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.print(F("Checking for file /boot.rom...."));
    if (!theBootROM.open(rootDirectory, "boot.rom", O_READ)) {
        Serial.println(F("NOT FOUND!"));
        signalHaltState();
    }
    Serial.println("FOUND!");
    Serial.print(F("Size of boot.rom: 0x"));
    Serial.print(theBootROM.fileSize(), HEX);
    Serial.println(F(" bytes"));
}
template<typename T>
bool deviceSanityCheck(T& thing) {
    bool outcome = true;
    for (uint16_t i = 0; i < 0x100000; ++i ) {
        thing.write(i, i, LoadStoreStyle::Lower8);
        delay(1);
    }
    delay(5000);
    for (uint16_t i = 0; i < 0x100000; ++i ) {
        if (auto result = thing.read(0, LoadStoreStyle::Lower8); result != i) {
#if 0
            Serial.print(F("Got: "));
            Serial.print(result, HEX);
            Serial.print(F(" Expected: "));
            Serial.println(result, HEX);
            delay(10);
#endif
            delay(1);
            outcome = false;
        }
    }
    return outcome;
}

void setupPeripherals() {
    setupTFT();
    setupSDCard();
    setupSRAMCache();
}
// the setup routine runs once when you press reset:
void setup() {
    Serial.begin(115200);
    Serial.println(F("i960Sx chipset bringup"));
    setupPins(OUTPUT,
              i960Pinout::Reset960,
              i960Pinout::SPI_BUS_EN,
              i960Pinout::DISPLAY_EN,
              i960Pinout::SD_EN );
	digitalWrite(i960Pinout::SPI_BUS_EN, HIGH);
    digitalWrite(i960Pinout::SD_EN, HIGH);
    digitalWrite(i960Pinout::DISPLAY_EN, HIGH);
    pinMode(static_cast<int>(i960Pinout::Led), OUTPUT);
    t.oscillate(static_cast<int>(i960Pinout::Led), 1000, HIGH);
    Wire.begin();
    SPI.begin();
	PinAsserter<i960Pinout::Reset960> holdi960InReset;
	setupIOExpanders();
	setupCPUInterface();
	setupBusStateMachine();
	setupPeripherals();
	delay(1000);
	// we want to jump into the code as soon as possible after this point
	Serial.println(F("i960Sx chipset brought up fully!"));
}
void loop() {
	fsm.run_machine();
    t.update();
}
/// @todo Eliminate after MightyCore update
#if __cplusplus >= 201402L

void operator delete(void * ptr, size_t)
{
    ::operator delete(ptr);
}

void operator delete[](void * ptr, size_t)
{
    ::operator delete(ptr);
}

#endif // end language is C++14 or greater