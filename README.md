This repo contains the firmware for all the chips that make up a type 2.01
i960 board. 

Type 2.01 is a mini-itx form factor board which uses the following processors
to help control the i960:

- Grand Central M4 (Chipset)
- ATMEGA4809 DIP40 (Management Engine)
- ATMEGA4809 DIP40 (Programmable Interrupt Controller [PIC])

It also provides two feather sockets meant to connect two feathers into the
board as well. They are connected to the GCM4 via serial + one interrupt pin
each so the feathers can tell the GCM4 if it needs to service a request. 

An UNO socket is provided as well which is a breakout of the remaining GCM4
pins. My goal is to support the Dazzler 3X shield because it is a very easy way
to add graphical capabilities to the board. 

The hitagimon firmware specific to this board will also be found here as well.
