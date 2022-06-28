This is a "fork" of the 1284p based Chipset code. The 1284p is a fine target but it does have some limitations including:
1) Running off of the same 20mhz signal as the i960 means it runs twice as fast but still introduces latency that I am currently unable to get under ~5-7 usec based on transaction type
2) As I add more features (display, RTC, etc) the amount of latency does increase as we go on
3) The PSRAM in the type 1 design can run at a maximum of 5MHz instead of 10 MHz. This is due to the fact that the eight element SPI breakout board I use must use HC series ICs. These ICs run at a maximum of 5 mhz at 3.3v. The tradeoff is that I can get 64-megabytes of RAM from 9 pins (really 7 if you don't count pwr/gnd)
   1) I did try different techniques to improve speed like using two spi busses in a dual channel configuration (works but is wonky) to double throughput. A nice idea but it still has some issues that I don't have the electrical engineering background to solve.
4) With only 16k of sram total on the 1284p, I found that using 8k of it for the basis of an onboard cache _really_ improves performance (8-way Rand Tree PLRU is the sweet spot between latency and bus usage when dealing with PSRAM)
   1) Fun fact, using 16-way Bit PLRU when the backing ram is a file on an SD Card really improves performance over 8-way (PSRAM will always be faster)
5) Switch statements are a real killer of performance on the 1284p due to the fact that the lookup table is found in the flash memory space and requires 3 cycles to access (even with pipelining it isn't fun).
    
There are other problems too but the main one comes down to latency and lack of SRAM to increase cache size (having a 16, 32, 64, or 128k cache will really improve performance).
The solution to this problem is the use of a much more powerful SAMD51/21 or other ARM based microcontroller. The initial target is the Adafruit Grand Central M4 microcontroller board. It has the following wonderful features:

- 1 Megabyte of Builtin Flash
- 8 Megabytes of External Flash 
- Builtin SDCard slot connected to an independent bus
- 120 MHz ARM Cortex M4F core
- 2 DACs
- 256k of builtin SRAM
- at least 86 pins
- 32-bit architecture

This is a huge improvement over the atmega1284p and would be a dream to work with. However, there is one major problem with
just using the GCM4 or other arm based microcontrollers. The clock signals that they can generate through software are 
independent of the primary cpu clock (they are generally asynchronous). In the 1284p, everything in the microcontroller was
based off of the external 20MHz crystal. In the SAMD based micrcontrollers, the cpu clock is tied to a Phase Locked Loop which 
is nice but cannot be divided down without changing the actual frequency itself (at least within the microcontroller). Because of this 
problem, I decided that it was necessary to move all of the i960Sx timing specific aspects to a separate AVR based microcontroller
that manages the i960. I call it the management engine and actually really simple. It has the following tasks:

1. Run in lock step with the i960
2. Model the boot process (tell the chipset when the i960 has fully booted)
3. Model the memory transaction state machine (very simple but very timing dependent, can easily get out of sync no matter how careful)
   1. Tell the chipset when it is allowed to perform different aspects of a memory transaction
4. Tell the chipset when a checksum failure has happened
5. Act a sink for all of the extra pins exposed by the i960 which go unused (~LOCK, HOLD, HLDA, ~INT0, INT1, INT2, ~INT3)

In 1284p based chipsets the management engine and chipset are the same microcontroller. In this version, the management engine
and chipset are separate microcontrollers. The management engine is the conductor and the chipset relies on the management engine to
stay in sync without using the same clock source. The state machine modelled in the 4809 stays in lockstep with the i960 and uses pulses
to tell the chipset when it can proceed.

This state machine is found in the aux directory. This project will continue to be worked on as time goes on. It is not working right now

This is a fork of the older gcm4 based targets because I need to make extensive
changes to how this all works
