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


# NOTE: this code is taken from the initialization code found in the i960Sx manual

/*
Below is the system initialization code and tables.
The code builds the PRCB (PRocessor Control Block) in memory, sets up the stack frame, the interrupt,
fault, and system procedure tables, and then vectors to a user defined routine. *main*
*/

# declare ahead of time

.global system_address_table
.global prcb_ptr
.global _prcb_ram
.global start_ip
.global cs1

.global _user_stack
.global _sup_stack # supervisor stack
.global _intr_stack # interrupt stack


# Core Initialization Block (located at address 0)
# 8 words

.text

system_address_table:
    .word system_address_table # SAT pointer
    .word prcb_ptr # prcb pointer
    .word 0
    .word start_ip # pointer to first ip
    .word cs1 # calculated at link time (bind ?cs1 (- (+ ?SAT ?PRCB ?startIP)))
    .word 0
    .word 0
    .word -1

    # now reserve 88 more bytes
    .space 88

    .word sys_proc_table        # initialization words
    .word 0x304000fb
    .space 8

    .word system_address_table
    .word 0x00fc00fb            # initialization words
    .space 8

    .word sys_proc_table
    .word 0x304000fb            # initialization words
    .space 8

    .word fault_proc_table
    .word 0x304000fb            # initialization words

# initial PRCB
# this is our startup PRCB. After initialization.
# this will be copied to RAM

prcb_ptr:
    .word 0x0 # 0 - reserved
    .word 0xc # 4 - initialize to 0xc
    .word 0x0 # 8 - reserved
    .word 0x0 # 12 - reserved
    .word 0x0 # 16 - reserved
    .word intr_table # 20 - interrupt table address
    .word _intr_stack # 24 - interrupt stack pointer
    .word 0x0 # 28 - reserved
    .word 0x000001ff  # 32 - pointer to offset zero
    .word 0x0000027f  # 36 - system procedure table pointer
    .word fault_table # 40 - fault table
    .word 0x0 # 44 - reserved
    .space 32 # 48 - reserved
    .space 92 # 80 - scratch space

# the system procedure table will _only_ be used if the user make a supervisor procedure call
    .align 6

.global sys_proc_table
sys_proc_table:
    .word 0 # Reserved
    .word 0 # Reserved
    .word 0 # Reserved
    .word (_sup_stack + 0x1) # Supervisor stack pointer
    .word 0 # Preserved
    .word 0 # Preserved
    .word 0 # Preserved
    .word 0 # Preserved
    .word 0 # Preserved
    .word 0 # Preserved
    .word 0 # Preserved
    .word 0 # Preserved
# up to 260 entries!
    # example entry
	#.word	(_console_io + 0x2)	# Calls 0 - console I/O routines
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
# up to a total of 260 entries

# below is the fault table for calls to the fault handler.
# this table is provided because the above table (supervisor table) will allow
# tracing of trace-fault events (creating an endless loop), whereas this table will
# not allow tracing of trace-fault events.

    .align 6
fault_proc_table:
    .word 0 # Reserved
    .word 0 # Reserved
    .word 0 # Reserved
    .word _sup_stack # Supervisor stack pointer
    .word 0 # Preserved
    .word 0 # Preserved
    .word 0 # Preserved
    .word 0 # Preserved
    .word 0 # Preserved
    .word 0 # Preserved
    .word 0 # Preserved
    .word 0 # Preserved
    .word (_user_reserved_core + 0x2)    # entry 0
    .word (_user_trace_core + 0x2)    # entry 1
    .word (_user_operation_core + 0x2)    # entry 2
    .word (_user_arithmetic_core + 0x2)    # entry 3
    .word (_user_real_arithmetic_core + 0x2)    # entry 4
    .word (_user_constraint_core + 0x2)    # entry 5
    .word (_user_protection_core + 0x2)    # entry 6
    .word (_user_machine_core + 0x2)    # entry 7
    .word (_user_type_core + 0x2)    # entry 8

 # processor starts execution at this spot upon power-up after self-test.
_user_reserved_core:
	lda	-48(fp), g0	/* pass fault data */
	callx _user_reserved
	flushreg
	ret
_user_trace_core:
	lda	-48(fp), g0	/* pass fault data */
	callx  _user_trace
	flushreg
	ret
_user_operation_core:
	lda	-48(fp), g0	/* pass fault data */
	callx _user_operation
	flushreg
	ret
_user_arithmetic_core:
	lda	-48(fp), g0	/* pass fault data */
	callx _user_arithmetic
	flushreg
	ret
_user_real_arithmetic_core:
	lda	-48(fp), g0	/* pass fault data */
	callx _user_real_arithmetic
	flushreg
	ret
_user_constraint_core:
	lda	-48(fp), g0	/* pass fault data */
	callx _user_constraint
	flushreg
	ret
_user_protection_core:
	lda	-48(fp), g0	/* pass fault data */
	callx _user_protection
	flushreg
	ret
_user_machine_core:
	lda	-48(fp), g0	/* pass fault data */
	callx _user_machine
	flushreg
	ret
_user_type_core:
	lda	-48(fp), g0	/* pass fault data */
	callx _user_type
	flushreg
	ret
 start_ip:
    mov 0, g14 # C compiler expects g14 = 0

# enable address debugging
    # lda 0xFE000022, g8
    # lda 0x1, g9
    # st g9, 0(g8)
    # copy the interrupt table to RAM space
    lda 1028, g0 # load length of the interrupt table
    lda 0, g4 # initialize offset to 0
    lda intr_table, g1 # load source
    lda intr_ram, g2    # load address of new table
    bal move_data # branch to move routine
    # lda 0xFE000022, g8
    # lda 0, g9
    # st g9, 0(g8)

# copy PRCB to RAM space, located at _prcb_ram

    lda 176,g0 # load length of PRCB
    lda 0, g4 # initialize offset to 0
    lda prcb_ptr, g1 # load source
    lda _prcb_ram, g2 # load destination
    bal move_data # branch to move routine
 # fix up the PRCB to point to a new interrupt table
    lda intr_ram, g12 # load address
    st g12, 20(g2) # store into PRCB

 /*
  * -- At this point, the PRCB, and interrupt table have been moved to RAM.
  *    It is time to issue a reinitialize IAC, which will start us anew with our RAM based PRCB.
  *
  * -- The IAC message, found in the 4 words located at the reinitialize_iac label, contains pointers
  *    to the current System Address Table, the new RAM based PRCB, and to the Instruction Pointer
  *    labeled start_again_ip
 */
    lda 0xff000010, g5
    lda reinitialize_iac, g6
    synmovq g5, g6
    /* FALLTHROUGH DOES NOT HAPPEN HERE!!!! */

  /*
   * The process will begin execution here after being reinitialized.
   *    We will now setup the stacks and continue.
   */

  start_again_ip:
  /* -- this would be a good place to diable board interrupts if you are using an interrupt controller.
   *
   * -- Before call to main, we need to take the processor out of the "interrupted" state.
   *    In order to do this, we will execute a call statement, then "fix up" the stack frame
   *    to cause an interrupt return to be executed.
   */
    ldconst 64, g0 # bump up stack to make
    addo sp, g0, sp # room for simulated
                    # interrupt frame

    call fix_stack  # routine to turn off int state

    lda _user_stack, fp     # setup user stack space
    lda -0x40(fp), pfp      # load pfp (just in case)
    lda 0x40(fp), sp        # set up current stack pointer

/* -- This is the point where your main code is called.
 *    If any IO needs to be set up, you should do it here before your
 *    call to main. No opens have been done for STDIN, STDOUT, or STDERR
 */
.ifdef __i960SB__
    callx _init_fp
.endif
    callx setupInterruptHandler
    #callx _activate_read_write_transactions
    mov 0, g14      # C compiler expects g14 = 0
    callx _main     # assume a main for startup

.ifdef __i960SB__
_init_fp:
    # initialize the floating point registers
    cvtir   0, fp0
    movre   fp0, fp1
    movre   fp1, fp2
    movre   fp2, fp3
    ret
.endif

setupInterruptHandler:
    # setup the interrupt handlers to work correctly
    lda 0xff000004, g5
    # give maximum priority to the interrupt handlers
    lda defaultInterruptHandlerValue, g6
    synmov g5, g6
    ret

    .align 4 # Align BEFORE the label...holy crap
reinitialize_iac:
    .word 0x93000000    # reinitialize IAC message
    .word system_address_table
    .word _prcb_ram     # use newly copied PRCB
    .word start_again_ip    # start here

defaultInterruptHandlerValue:
    .word 0xFCFDFEFF
/* -- define RAM area to copy the PRCB and interrupt table
 *    to after initial bootup from EPROM/FLASH
 */
    .bss intr_ram, 1028, 6
    .bss _prcb_ram, 176, 6
 /* -- define RAM area for stacks; size is only a suggestion your actual
  *    mileage may vary
  */
    .bss _user_stack, 0x10000, 6
    .bss _intr_stack, 0x10000, 6
    .bss _sup_stack, 0x10000, 6

/* -- Below is a software loop to move data */

move_data:
    ldq (g1)[g4*1], g8  # load 4 words into g8
    stq g8, (g2)[g4*1]  # store to RAM block
    addi g4,16, g4      # increment index
    cmpibg  g0,g4, move_data # loop until done
    bx (g14)

# setup the bss section so do giant blocks of writes

/* The routine below fixes up the stack for a flase interrupt return.
 * We have reserved area on the stack before the call to this
 * routine. We need to build a phony interrupt record here
 * to force the processor to pick it up on return. Also, we
 * will take advantage of the fact that the processor will
 * restore the PC and AC to its registers
 */

fix_stack:
    flushreg
    or  pfp, 7, pfp     # put interrupt return code into pfp

    ldconst 0x1f0002, g0
    st  g0, -16(fp)     # store contrived PC
    ldconst 0x3b001000, g0  # setup arithmetic controls
    st  g0, -12(fp)     # store contrived AC
    ret


