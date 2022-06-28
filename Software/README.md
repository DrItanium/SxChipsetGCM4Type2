# HITAGIMON

A bare metal application which runs on my custom i960SB based development hardware. 
The monitor is not actually a monitor but closer to a framework that the application sits
on top of. 

Previously, the program that I was bringing up was DOOM. I ran into issues so I migrated to the i960 running CLIPS (C Language Integrated Processing System) which is a piece of software I have a considerable amount of experience with. 

The design of this program is broken up into several components:

1. boot - The code which is used to boot the i960 prior to newlib being functional
   1. This also contains references to the system procedure table and other i960 specific features
2. chipset - The code which is used to interact with the memory mapped io provided by the 1284p based chipset through a very clean C++ interface which is leveraged by sys.
3. sys - Implementations of the system specific routines required by newlibc (sbrk, read, write, etc)
4. cortex - Wrapper classes around C library functionality plus anything else that C++ makes easier
5. clips - The clips source code
6. doom - The doom source code left around in case I figure out why errors were happening (could be the compiler)
7. reference - NINDY source code reference
8. clp - Demo programs written in CLIPS to try out different features

Hitagimon does not use the system procedure table at this point and is closer to an arduino program in its design. This
choice was made due to the fact that there is no operating system nor indirect services that I have to insulate from the
program itself.

## NOTES
This project requires access to the i960-elf-gcc 3.4.6 cross compiler to build.

The reference directory contains the source code and documentation for NINDY which is where this application started
and then forked off in a better direction.

## WHAT IS HITAGI?

Hitagi is the code name for my first generation i960Sx custom development board that I have spent the last two years 
designing and bringing up from near scratch. The board uses an Atmega1284p to act as the "chipset" for the i960Sx. 
The 1284p services all memory requests and exposes peripherals through the memory space of the i960. 


