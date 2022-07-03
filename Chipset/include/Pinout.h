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

#ifndef ARDUINO_PINOUT_H
#define ARDUINO_PINOUT_H
#include <Arduino.h>
#include "MCUPlatform.h"
#include <tuple>
using Address = uint32_t;
/**
 * @brief Sx Load/Store styles that the processor will request
 */
enum class LoadStoreStyle : uint8_t {
    Full16 = 0,
    Upper8,
    Lower8,
    None
};
constexpr uint8_t makeLoadStoreMask(LoadStoreStyle lo, LoadStoreStyle hi) noexcept {
    return (static_cast<uint8_t>(lo) | (static_cast<uint8_t>(hi) << 2)) & 0b1111;
}
enum class LoadStoreStyle32 : uint8_t {
#define X(Hi, Lo) Hi ## _ ## Lo = makeLoadStoreMask(LoadStoreStyle:: Lo , LoadStoreStyle:: Hi)
#define Y(Hi) X(Hi, Full16), X(Hi, Upper8), X(Hi, Lower8), X(Hi, None)
    Y(Full16),
    Y(Upper8),
    Y(Lower8),
    Y(None),
#undef Y
#undef X
    Full32 = Full16_Full16,
    Upper16_Lower8 = Full16_Upper8,
    Upper16_Lowest8 = Full16_Lower8,
    Upper16 = Full16_None,
    Highest8 = Upper8_None,
    Highest8_Lower16 = Upper8_Full16,
    Highest8_Lower8 = Upper8_Upper8,
    Highest8_Lowest8 = Upper8_Lower8,
    Higher8 = Lower8_None,
    Higher8_Lower16 = Lower8_Full16,
    Higher8_Lower8 = Lower8_Upper8,
    Higher8_Lowest8 = Lower8_Lower8,
    Lower16 = None_Full16,
    Lowest8 = None_Lower8,
    Lower8 = None_Upper8,
    None = None_None,
};
constexpr LoadStoreStyle32 convert16To32(LoadStoreStyle lower, LoadStoreStyle upper) noexcept {
    auto lowerHalf = static_cast<uint8_t>(lower);
    auto upperHalf = static_cast<uint8_t>(upper) << 2;
    return static_cast<LoadStoreStyle32>((lowerHalf | upperHalf) & 0b1111);
}
constexpr std::tuple<LoadStoreStyle, LoadStoreStyle> convert32To16(LoadStoreStyle32 value) noexcept {
    return std::make_tuple(static_cast<LoadStoreStyle>(static_cast<uint8_t>(value) & 0b11),
                           static_cast<LoadStoreStyle>((static_cast<uint8_t>(value) >> 2) & 0b11));
}

enum class i960Pinout : int {
#include "Type201Pinout.def"
};
[[gnu::always_inline]]
inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
    ::digitalWrite(static_cast<int>(ip), value);
}

[[gnu::always_inline]]
inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
    pinMode(static_cast<uint32_t>(ip), value);
}
[[gnu::always_inline]]
inline auto digitalRead(i960Pinout ip) {
    return digitalRead(static_cast<int>(ip));
}
template<i960Pinout pin>
constexpr auto isValidPin960_v = static_cast<int>(pin) < static_cast<int>(i960Pinout::Count);
#include "GrandCentralM4_Pinout.h"
template<i960Pinout pin>
[[gnu::always_inline]] inline void digitalWrite(bool level) noexcept {
    digitalWrite<pin>(level ? HIGH : LOW);
}

template<i960Pinout pin, decltype(HIGH) switchTo = LOW>
[[gnu::always_inline]]
inline void pulse() noexcept {
    // use the switch to value to compute what to revert to
    digitalWrite<pin, switchTo>();
    digitalWrite<pin, ((switchTo == LOW) ? HIGH : LOW)>();
}
template<i960Pinout pin>
struct DigitalPinConfiguration {
    DigitalPinConfiguration() = delete;
    ~DigitalPinConfiguration() = delete;
    DigitalPinConfiguration(const DigitalPinConfiguration&) = delete;
    DigitalPinConfiguration(DigitalPinConfiguration&&) = delete;
    DigitalPinConfiguration& operator=(const DigitalPinConfiguration&) = delete;
    DigitalPinConfiguration& operator=(DigitalPinConfiguration&&) = delete;
    static constexpr bool isSpecialized() noexcept { return false; }
    static constexpr bool isInputPin() noexcept { return false; }
    static constexpr bool isOutputPin() noexcept { return false; }
    static constexpr auto isInputPullupPin() noexcept { return false; }
    static constexpr auto isInputPulldownPin() noexcept { return false; }
    static constexpr int getDirection() noexcept { return INPUT; }
    static constexpr auto getPin() noexcept { return pin; }
    static constexpr auto valid() noexcept { return isValidPin960_v<pin>; }
    static constexpr auto getAssertionState() noexcept { return LOW; }
    static constexpr auto getDeassertionState() noexcept { return HIGH; }
};

#define DefInputPin2(pin, assert, deassert) \
template<> \
struct DigitalPinConfiguration<pin> { \
    DigitalPinConfiguration() = delete; \
    ~DigitalPinConfiguration() = delete; \
    DigitalPinConfiguration(const DigitalPinConfiguration&) = delete; \
    DigitalPinConfiguration(DigitalPinConfiguration&&) = delete; \
    DigitalPinConfiguration& operator=(const DigitalPinConfiguration&) = delete; \
    DigitalPinConfiguration& operator=(DigitalPinConfiguration&&) = delete; \
    static constexpr bool isSpecialized() noexcept { return true; } \
    static constexpr bool isInputPin() noexcept { return true; } \
    static constexpr bool isOutputPin() noexcept { return false; } \
    static constexpr auto isInputPullupPin() noexcept { return false; } \
    static constexpr auto isInputPulldownPin() noexcept { return false; } \
    static constexpr auto getDirection() noexcept { return INPUT; } \
    static constexpr auto getPin() noexcept { return pin; } \
    static constexpr auto valid() noexcept { return isValidPin960_v<pin>; } \
    static constexpr auto getAssertionState() noexcept { return assert; } \
    static constexpr auto getDeassertionState() noexcept { return deassert; } \
}

#define DefOutputPin2(pin, assert, deassert) \
template<> \
struct DigitalPinConfiguration<pin> { \
    DigitalPinConfiguration() = delete; \
    ~DigitalPinConfiguration() = delete; \
    DigitalPinConfiguration(const DigitalPinConfiguration&) = delete; \
    DigitalPinConfiguration(DigitalPinConfiguration&&) = delete; \
    DigitalPinConfiguration& operator=(const DigitalPinConfiguration&) = delete; \
    DigitalPinConfiguration& operator=(DigitalPinConfiguration&&) = delete; \
    static constexpr bool isSpecialized() noexcept { return true; } \
    static constexpr bool isInputPin() noexcept { return false; } \
    static constexpr bool isOutputPin() noexcept { return true; } \
    static constexpr auto isInputPullupPin() noexcept { return false; } \
    static constexpr auto isInputPulldownPin() noexcept { return false; } \
    static constexpr auto getDirection() noexcept { return OUTPUT; } \
    static constexpr auto getPin() noexcept { return pin; } \
    static constexpr auto valid() noexcept { return isValidPin960_v<pin>; } \
    static constexpr auto getAssertionState() noexcept { return assert; } \
    static constexpr auto getDeassertionState() noexcept { return deassert; } \
}

#define DefInputPullupPin2(pin, assert, deassert) \
template<> \
struct DigitalPinConfiguration<pin> { \
    DigitalPinConfiguration() = delete; \
    ~DigitalPinConfiguration() = delete; \
    DigitalPinConfiguration(const DigitalPinConfiguration&) = delete; \
    DigitalPinConfiguration(DigitalPinConfiguration&&) = delete; \
    DigitalPinConfiguration& operator=(const DigitalPinConfiguration&) = delete; \
    DigitalPinConfiguration& operator=(DigitalPinConfiguration&&) = delete; \
    static constexpr bool isSpecialized() noexcept { return true; } \
    static constexpr bool isInputPin() noexcept { return false; } \
    static constexpr bool isOutputPin() noexcept { return false; } \
    static constexpr auto isInputPullupPin() noexcept { return true; } \
    static constexpr auto isInputPulldownPin() noexcept { return false; } \
    static constexpr auto getDirection() noexcept { return INPUT_PULLUP; } \
    static constexpr auto getPin() noexcept { return pin; } \
    static constexpr auto valid() noexcept { return isValidPin960_v<pin>; } \
    static constexpr auto getAssertionState() noexcept { return assert; } \
    static constexpr auto getDeassertionState() noexcept { return deassert; } \
}
#define DefInputPulldownPin2(pin, assert, deassert) \
template<> \
struct DigitalPinConfiguration<pin> { \
    DigitalPinConfiguration() = delete; \
    ~DigitalPinConfiguration() = delete; \
    DigitalPinConfiguration(const DigitalPinConfiguration&) = delete; \
    DigitalPinConfiguration(DigitalPinConfiguration&&) = delete; \
    DigitalPinConfiguration& operator=(const DigitalPinConfiguration&) = delete; \
    DigitalPinConfiguration& operator=(DigitalPinConfiguration&&) = delete; \
    static constexpr bool isSpecialized() noexcept { return true; } \
    static constexpr bool isInputPin() noexcept { return false; } \
    static constexpr bool isOutputPin() noexcept { return false; } \
    static constexpr auto isInputPullupPin() noexcept { return false; } \
    static constexpr auto isInputPulldownPin() noexcept { return true; } \
    static constexpr auto getDirection() noexcept { return INPUT_PULLDOWN; } \
    static constexpr auto getPin() noexcept { return pin; } \
    static constexpr auto valid() noexcept { return isValidPin960_v<pin>; } \
    static constexpr auto getAssertionState() noexcept { return assert; } \
    static constexpr auto getDeassertionState() noexcept { return deassert; } \
}
#define DefBidirectionalPin2(pin, assert, deassert) \
template<> \
struct DigitalPinConfiguration<pin> { \
    DigitalPinConfiguration() = delete; \
    ~DigitalPinConfiguration() = delete; \
    DigitalPinConfiguration(const DigitalPinConfiguration&) = delete; \
    DigitalPinConfiguration(DigitalPinConfiguration&&) = delete; \
    DigitalPinConfiguration& operator=(const DigitalPinConfiguration&) = delete; \
    DigitalPinConfiguration& operator=(DigitalPinConfiguration&&) = delete; \
    static constexpr bool isSpecialized() noexcept { return true; } \
    static constexpr bool isInputPin() noexcept { return true; } \
    static constexpr bool isOutputPin() noexcept { return true; } \
    static constexpr auto isInputPullupPin() noexcept { return false; } \
    static constexpr auto isInputPulldownPin() noexcept { return false; } \
    static constexpr auto getDirection() noexcept { return INPUT; } \
    static constexpr auto getPin() noexcept { return pin; } \
    static constexpr auto valid() noexcept { return isValidPin960_v<pin>; } \
    static constexpr auto getAssertionState() noexcept { return assert; } \
    static constexpr auto getDeassertionState() noexcept { return deassert; } \
}

#define DefSPICSPin2(pin) DefOutputPin2(pin, LOW, HIGH)


DefSPICSPin2(i960Pinout::GPIOSelect);
DefSPICSPin2(i960Pinout::SD_EN);
DefOutputPin2(i960Pinout::ChipsetBooted, LOW, HIGH);
DefOutputPin2(i960Pinout::Ready, LOW, HIGH);
DefOutputPin2(i960Pinout::Reset4809, LOW, HIGH);
DefInputPin2(i960Pinout::SuccessfulBoot, HIGH, LOW);
//DefInputPin2(i960Pinout::W_R_, LOW, HIGH);
//DefInputPin2(i960Pinout::BE0, LOW, HIGH);
//DefInputPin2(i960Pinout::BE1, LOW, HIGH);
DefInputPin2(i960Pinout::INT_EN0, LOW, HIGH);
DefInputPin2(i960Pinout::DoCycle, LOW, HIGH);
DefInputPin2(i960Pinout::BurstNext, HIGH, LOW); // BLAST Pin emulation
DefInputPin2(i960Pinout::InTransaction, LOW, HIGH);
DefBidirectionalPin2(i960Pinout::Data0, LOW, HIGH);
DefBidirectionalPin2(i960Pinout::Data1, LOW, HIGH);
DefBidirectionalPin2(i960Pinout::Data2, LOW, HIGH);
DefBidirectionalPin2(i960Pinout::Data3, LOW, HIGH);
DefBidirectionalPin2(i960Pinout::Data4, LOW, HIGH);
DefBidirectionalPin2(i960Pinout::Data5, LOW, HIGH);
DefBidirectionalPin2(i960Pinout::Data6, LOW, HIGH);
DefBidirectionalPin2(i960Pinout::Data7, LOW, HIGH);
DefBidirectionalPin2(i960Pinout::Data8, LOW, HIGH);
DefBidirectionalPin2(i960Pinout::Data9, LOW, HIGH);
DefBidirectionalPin2(i960Pinout::Data10, LOW, HIGH);
DefBidirectionalPin2(i960Pinout::Data11, LOW, HIGH);
DefBidirectionalPin2(i960Pinout::Data12, LOW, HIGH);
DefBidirectionalPin2(i960Pinout::Data13, LOW, HIGH);
DefBidirectionalPin2(i960Pinout::Data14, LOW, HIGH);
DefBidirectionalPin2(i960Pinout::Data15, LOW, HIGH);
DefInputPin2(i960Pinout::MUXADR0, LOW, HIGH);
DefInputPin2(i960Pinout::MUXADR1, LOW, HIGH);
DefInputPin2(i960Pinout::MUXADR2, LOW, HIGH);
DefInputPin2(i960Pinout::MUXADR3, LOW, HIGH);
DefInputPin2(i960Pinout::MUXADR4, LOW, HIGH);
DefInputPin2(i960Pinout::MUXADR5, LOW, HIGH);
DefInputPin2(i960Pinout::MUXADR6, LOW, HIGH);
DefInputPin2(i960Pinout::MUXADR7, LOW, HIGH);
DefOutputPin2(i960Pinout::MUXSel0, HIGH, LOW);
DefOutputPin2(i960Pinout::MUXSel1, HIGH, LOW);
DefOutputPin2(i960Pinout::MUXSel2, HIGH, LOW);
DefOutputPin2(i960Pinout::MUX_EN, LOW, HIGH);
DefInputPin2(i960Pinout::Feather0_INT, LOW, HIGH);
DefInputPin2(i960Pinout::Feather1_INT, LOW, HIGH);
DefInputPin2(i960Pinout::BusHold_Acknowledge, HIGH, LOW);
DefOutputPin2(i960Pinout::BusHold, HIGH, LOW);
#undef DefOutputPin2
#undef DefInputPin2
#undef DefInputPullupPin2
#undef DefInputPulldownPin2
#undef DefBidirectionalPin2

template<i960Pinout pin>
struct DigitalPin2 {
    using Configuration = DigitalPinConfiguration<pin>;
    DigitalPin2() = delete;
    ~DigitalPin2() = delete;
    DigitalPin2(const DigitalPin2&) = delete;
    DigitalPin2(DigitalPin2&&) = delete;
    DigitalPin2& operator=(const DigitalPin2&) = delete;
    DigitalPin2& operator=(DigitalPin2&&) = delete;
    static constexpr bool isInputPin() noexcept { return Configuration::isInputPin(); }
    static constexpr bool isOutputPin() noexcept { return Configuration::isOutputPin(); }
    static constexpr bool isInputPullupPin() noexcept { return Configuration::isInputPullupPin(); }
    static constexpr bool isInputPulldownPin() noexcept { return Configuration::isInputPulldownPin(); }
    static constexpr bool isBidirectionalPin() noexcept { return isInputPin() && isOutputPin(); }
    static_assert(Configuration::isSpecialized(), "Digital Pin target is not specialized!");
    static constexpr auto getDirection() noexcept {
        if constexpr (isBidirectionalPin()) {
           return directionIsOutput_ ? OUTPUT : INPUT;
        } else {
            return Configuration::getDirection();
        }
    }
    static constexpr auto getPin() noexcept { return Configuration::getPin(); }
    static constexpr auto valid() noexcept { return Configuration::valid(); }
    static constexpr auto isSpecialized() noexcept { return Configuration::isSpecialized(); }
    static void configure() noexcept {
        if (!configured_) {
            configured_ = true;
            // do nothing if we are not specialized
            if constexpr (isSpecialized()) {
                pinMode(getPin(), getDirection());
                targetPort_ = &getPortGroup<getPin()>();
                targetPin_ = &getPinDescription<getPin()>();
                readMask_ = 1ul << targetPin_->ulPin;
            }
        }
        //return (getPortGroup<pin>().IN.reg & (bitMasks[getPinDescription<pin>().ulPin])) != 0 ? HIGH : LOW;
    }
    static constexpr auto getAssertionState() noexcept { return Configuration::getAssertionState(); }
    static constexpr auto getDeassertionState() noexcept { return Configuration::getDeassertionState(); }
    [[gnu::always_inline]]
    static inline auto read() noexcept {
        if constexpr (isSpecialized()) {
            if constexpr (isInputPin() || isInputPullupPin() || isInputPulldownPin()) {
                return (targetPort_->IN.reg & readMask_) != 0 ? HIGH : LOW;
            } else {
                // if this is specialized but an output pin then return low every time
                return LOW;
            }
        } else {
            return ::digitalRead(getPin());
        }
    }
    [[gnu::always_inline]]
    static inline auto isDeasserted() noexcept {
        if constexpr (isSpecialized()) {
            if constexpr (isInputPin() || isInputPulldownPin() || isInputPullupPin()) {
                return read() == getDeassertionState();
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
    [[gnu::always_inline]]
    static inline auto isAsserted() noexcept {
        if constexpr (isSpecialized()) {
            if constexpr (isInputPin() || isInputPulldownPin() || isInputPullupPin()) {
                return read() == getAssertionState();
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    template<decltype(LOW) value>
    [[gnu::always_inline]]
    static inline void write() noexcept {
        if constexpr (isSpecialized()) {
            if constexpr (isOutputPin()) {
                if constexpr (value == LOW) {
                    targetPort_->OUTCLR.reg = readMask_;
                } else {
                    targetPort_->OUTSET.reg = readMask_;
                }
            } else {
                // do nothing
            }
        } else {
            // oh it isn't specialized so just call the normal digitalWrite to be on the safe side
            ::digitalWrite(getPin(), value);
        }
    }
    [[gnu::always_inline]]
    static inline void write(decltype(LOW) value) noexcept {
        if (value == LOW) {
            write<LOW>();
        } else {
            write<HIGH>();
        }
    }
    [[gnu::always_inline]]
    static inline void write(bool value) noexcept {
        write(value ? HIGH : LOW);
    }

    [[gnu::always_inline]]
    static inline void assertPin() noexcept {
        if constexpr (isSpecialized()) {
            if constexpr (isOutputPin()) {
                write<getAssertionState()>();
            }
        } else {
            write(getAssertionState());
        }
    }
    [[gnu::always_inline]]
    static inline void deassertPin() noexcept {
        if constexpr (isSpecialized()) {
            if constexpr (isOutputPin()) {
                write<getDeassertionState()>();
            }
        } else {
            write(getDeassertionState());
        }
    }
    [[gnu::always_inline]]
    static inline void pulse() noexcept {
        if constexpr (isSpecialized()) {
            if constexpr (isOutputPin()) {
                write<getAssertionState()>();
                write<getDeassertionState()>();
            }
        } else {
            ::pulse<getPin(), getAssertionState()>();
        }
    }
    static inline uint32_t readInPort() noexcept {
        if constexpr (isSpecialized()) {
            return targetPort_->IN.reg;
        } else {
            return 0;
        }
    }
    static inline void writeInPort(uint32_t value) noexcept {
        if constexpr (isSpecialized()) {
            targetPort_->IN.reg = value;
        }
    }
    static inline void writeOutPort(uint32_t value) noexcept {
        if constexpr (isSpecialized()) {
            targetPort_->OUT.reg = value;
        }
    }
    static inline uint32_t readOutPort() noexcept {
        if constexpr (isSpecialized()) {
            return targetPort_->OUT.reg;
        } else {
            return 0;
        }
    }
    static inline uint32_t readPortDir() noexcept {
        if constexpr (isSpecialized()) {
            return targetPort_->DIR.reg;
        } else {
            return 0;
        }
    }
    static inline void writePortDir(uint32_t value) noexcept {
        if constexpr (isSpecialized()) {
            targetPort_->DIR.reg = value;
        }
    }
    static inline void directionOutput() noexcept {
        if constexpr (isSpecialized()) {
            if constexpr (isBidirectionalPin()) {
                if (!directionIsOutput_) {
                    directionIsOutput_ = true;
                    // do the work directly instead
                    targetPort_->PINCFG[targetPin_->ulPin].reg = static_cast<uint8_t>(PORT_PINCFG_INEN);
                    targetPort_->DIRSET.reg = readMask_;
                }
            }
        } else {
            pinMode(getPin(), OUTPUT);
        }
    }
    static inline void directionInput() noexcept {
        if constexpr (isSpecialized()) {
            if constexpr (isBidirectionalPin()) {
                if (directionIsOutput_) {
                    directionIsOutput_ = false;
                    targetPort_->PINCFG[targetPin_->ulPin].reg = static_cast<uint8_t>(PORT_PINCFG_INEN);
                    targetPort_->DIRCLR.reg = readMask_;
                }
            }
        } else {
            pinMode(getPin(), INPUT);
        }
    }
private:
    static inline volatile PortGroup* volatile targetPort_ = nullptr;
    static inline const PinDescription* targetPin_ = nullptr;
    static inline uint32_t readMask_ = 0xFFFF'FFFF;
    static inline bool configured_ = false;
    static inline bool directionIsOutput_ = false;
};
template<i960Pinout pin>
using DigitalPin = DigitalPin2<pin>;

template<i960Pinout pinId>
class PinAsserter {
public:
    static_assert(DigitalPin<pinId>::isOutputPin());
    PinAsserter() { DigitalPin<pinId>::assertPin(); }
    ~PinAsserter() { DigitalPin<pinId>::deassertPin(); }
};

template<i960Pinout ... pins>
[[gnu::always_inline]]
inline void configurePins() noexcept {
    (DigitalPin<pins>::configure(), ...);
}

[[gnu::always_inline]]
inline void attachInterrupt(i960Pinout pin, voidFuncPtr callback, uint32_t mode) noexcept {
    attachInterrupt(digitalPinToInterrupt(static_cast<uint32_t>(pin)),
                    callback,
                    mode);
}

[[gnu::always_inline]]
inline void interruptOnFallingEdge(i960Pinout pin, voidFuncPtr callback) noexcept {
    attachInterrupt(pin, callback, FALLING);
}

[[gnu::always_inline]]
inline void interruptOnRisingEdge(i960Pinout pin, voidFuncPtr callback) noexcept {
    attachInterrupt(pin, callback, RISING);
}




#endif //ARDUINO_PINOUT_H
