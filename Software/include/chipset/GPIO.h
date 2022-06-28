/*
hitagimon
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

#ifndef HITAGIMON_GPIO_H
#define HITAGIMON_GPIO_H
#include <stdint.h>
#include "IODevice.h"
#include "ChipsetInteract.h"

class GPIOEngine {
public:
    struct Port {
        /**
         * @brief Returns a mask of the pins which are actually connected, if all are zero then the port is not valid
         */
        volatile uint32_t mask;
        /**
         * @brief The value from the pins configured as output, this is the contents of the latch
         */
        volatile uint32_t output;
        /**
         * @brief The value from the pins configured as inputs
         */
        volatile uint32_t input;
        /**
         * @brief Defines if we want to enable input pullups or not (1 is enable pullups, 0 is disable them)
         */
        volatile uint32_t pullup;
        /**
         * @brief Defines the direction of the pin in the port, 1 is output, 0 is input
         */
        volatile uint32_t direction;
        /**
         * @brief Each bit corresponds to a given pin in the port, when set to 1 then activate pin change interrupts
         */
        volatile uint32_t interruptEnable;

        /**
         * @brief Setting a bit here causes the result to be inverted on that pin, clearing it will cause the input to be normal
         */
        volatile uint32_t inputPolarity;
        /**
         * @brief Describes the bits
         */
         volatile union {
            uint32_t full;
            struct {
                uint32_t inputPolaritySupported : 1;
            };
         } features;
        volatile uint32_t reserved[(256 - (8 * sizeof(uint32_t))) / sizeof(uint32_t)];
        inline bool valid() const volatile { return mask != 0; }
        inline bool invalid() const volatile { return mask == 0; }
        bool pinValid(int index) const volatile;
    } __attribute__((packed));
    // A way to do static assert but without the messaging
    static uint8_t PortViewIsAlignedCorrectlyTo256Bytes[sizeof(Port) == 256 ? 1 : -1];
    /**
     * @brief We support up to 32 ports of 32-bits each
     */
    struct Registers {
        /**
         * @brief The set of ports which are valid (a zero means not valid)
         */
        volatile uint32_t mask;
        volatile uint32_t reserved[(256 - (1 *sizeof(uint32_t))) / sizeof(uint32_t)];
        /**
         * @brief The set of ports
         */
        volatile Port ports[32];
        inline bool valid() const volatile { return mask != 0; }
        inline bool invalid() const volatile { return mask == 0; }
    } __attribute__((packed));
    static uint8_t RegisterSetIsSizedCorrectly[sizeof(Registers) == (33 * 256) ? 1 : -1];

public:
    GPIOEngine();
    ~GPIOEngine();
    void begin();
    bool available() const { return raw_.valid(); }
    inline operator bool() const { return raw_.valid(); }
    volatile Port& getPort(int index);
    const volatile Port& getPort(int index) const;
    inline bool validPort(int index) { return getPort(index).valid(); }
    inline bool validPort(int index) const { return getPort(index).valid(); }
    inline int pinToPort(int pinIndex) const { return pinIndex / 32; }
    inline int portOffset(int pinIndex) const { return pinIndex % 32; }
    bool validPin(int pinIndex) const;
private:
    volatile Registers& raw_;
    bool initialized_;
};

GPIOEngine& getGPIOEngine();
enum PinDirection{
    INPUT = 0,
    OUTPUT,
    INPUT_PULLUP,
};
void digitalWrite(int index, bool value);
bool digitalRead(int index);
void pinMode(int index, PinDirection dir);
void portWrite(int index, uint32_t value);
uint32_t portRead(int index);
bool validPort(int index);
bool validPin(int index);

#endif //HITAGIMON_GPIO_H
