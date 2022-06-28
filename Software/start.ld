GROUP(-lgcc -lm -lc -liberty -lc ) /* libraries to link in */

MEMORY /* describe the partitioning of the memory space for the application */ 
{
    /* This design will two PSRAM Chips */
	/* the rom section will start at address 0 and be up to 2 megs in
	 * length*/ 
	rom (rx)  : ORIGIN = 0, l = 2M
	/* the "ram" section will start at 0x0020'0000 which comes immediately after the rom space
	 * the 32-bit address space and be 14 megs in size */
	ram (!rx) : o = 0x00200000, l = 6M

    /* This design will span the entire 64 megabyte space */
	/* the rom section will start at address 0 and be up to 8 megs in
	 * length*/
	/* rom (rx)  : ORIGIN = 0, l = 8M */
	/* the "ram" section will start at 0x0080'0000 which comes immediately after the rom space
	 * the 32-bit address space and be 56 megs in size */
	/* ram (!rx) : o = 0x00800000, l = 56M */

	/* NOTE: do not uncomment these lines, this is here as a reference to the
	 * design of the hitagi board */
	/* primary io space starts at 0xFE00'0000 and is memory mapped io*/
	/* io0 (!rx) : o = 0xFE000000, l = 16M  */
	/* cpu_internal (!rwxail) : o = 0xFF000000, l = 16M - 1 */
	/* NOTE: the chipset will catch any unmapped reads and writes, report them,
	 * and either do nothing (on a write) or return zero (on a read)
	 */


	/* the format of a memory entry is: name (permissions) : <flags> 
	ORIGIN or o is the starting location LENGTH or l is the length of the
	memory section 
	 */ 

} 

/* Define a symbol that your code can reference. The linker adds the symbol. */ 

STACK_SIZE = 0x10000; 

/* Layout the different ELF sections into memory using the above described
 * memory spaces */ 
SECTIONS
{
	/* this is the text section */ 
	.text : 
	{ 
		CREATE_OBJECT_SYMBOLS /* C++ needs this */ 

		*(.text) 	/* place the actual contents of the text section here */ 

		_etext = .; /* define a symbol named _etext and assign it to the
					   address following the end of the text section data */ 

		___CTOR_LIST__ = .; /* put a symbol __CTOR_LIST__ at this point */ 

		LONG((___CTOR_END__ - ___CTOR_LIST__) / 4 - 2) /* reserve a block of
														  memory and save the
														  result of end minus
														  list start. This will
														  show up in the final
														  executable */ 

		*(.ctors)		/* Take the contents of the .ctors elf section and
						   place it here, this is how you can request custom
						   sections to placed in specific locations */ 

		LONG(0) /* allocate a long's worth of space (4 bytes in this case) and
				   assign it zero (padding) */ 

		___CTOR_END__ = .; /* define the symbol __CTOR_END__ to be the point
							  after the previous padding */ 


		/* do the same thing again but for the destructors section */ 
		___DTOR_LIST__ = .;  
		LONG((___DTOR_END__ - ___DTOR_LIST__) / 4 - 2) 
		*(.dtors) 
		LONG(0) 
		___DTOR_END__ = .; 

	/* the arrow syntax tells the linker that the text section will go into the
	 * memory location named "rom", it is sequential as we will see in the next
	 * few sections*/ 
	} >rom 
	/* setup the bss section, tag it's start and end positions to allow the
	 * i960 to manually zero out the memory on the sdcard's ram.bin file */ 
	.bss : 
	{ 
		__bss_start__ = .; 
		*(.bss) 
		*(COMMON) /* can't find much documentation on this but it is a good idea to have this here */ 
		__bss_end__ = .; 
	} >ram 
	.data :
	{
		__data_start__ = .; 
		KEEP(*(.data)) 
		CONSTRUCTORS  /* we want to place the constructors data here */
		__data_end__ = .; 
		_edata = .; 
	} >ram 
	.heap : { 
		/* this section is synthetic, I do not actually copy the contents of the heap
		 * section over on boot, instead I just want the location of where it should
		 * start.In this case, the heap will start _after_ the .bss and .data sections
		 * in ram. It is placed linearly in order so there is no mystery to this. 
		 */
		__heap_start__ = .; 
		end = __heap_start__; 
		_end = end; 
		__end = end; 
		KEEP(*(.heap)) 
		__heap_end__ = .; 
		/* __HeapLimit = __heap_end__; */ 
	
	} >ram 
}

/* these two locations are _very_ important for getting the program to startup correctly */ 



/* Currently, I am manually decomposing the elf binary into ROM (.text) and DAT
 * (.data) files from the ELF using objcopy.  

 I have the chipset map the data section to 0x2000'0000 so we need to
 reflect that in the linker script (start_ip in the sx_init.s uses this). 

 I also compute the data section's length ahead of time to save instructions.  



 It is the responsibility of the i960 to copy the contents of the mapped data
 section into ram starting at address 0x8000'0000 on startup. Failure to do so
 leads to the inability to properly allocate memory (no malloc) and other
 various oddities. 



 The variable cs1 is specific to the i960 and is used in it's boot process.
 The processor on startup loads the first eight words (32 bytes) and performs
 a check sum with them. The processor will refuse to boot if the checksum
 algorithm does not yield a zero from combining these eight words together.
 Thus cs1 is used to make sure that 0 is the final result. The i960 SA/SB
 reference manual is where I got the cs1 line 



 The BSS Section length is used on startup to "copy" the bss into ram like the
 data section (although BSS copy happens first because it comes before data)..
 In reality this just zeroing out ram on the SDCard (ram.bin) to prevent bad
 things from happening during program execution 

 */ 

theDataSectionLength = (__data_end__ - __data_start__); 
theDataSectionROMLocation = 0x20000000; 
cs1 = -(system_address_table + prcb_ptr + start_ip) ; 
theBSSSectionLength = (__bss_end__ - __bss_start__); 

/* This directive allows you to denote the file that will go onto the front of
 * the text section, the problem is that with CMake this moves around. So
 * instead, I just make sure that sx_init.s is the first file in the list of
 * dependencies in CMakeFiles.txt. It does the same thing as the linker
 * appplies objects on a first come first serve basis. Very nice! */ 

/* STARTUP ( CMakeFiles/hitagimon.dir/sx_init.s.o ) */ 

/*

How to dump the actual boot code: 


i960-elf-objcopy --only-section=.text -O binary hitagimon hitagimon.bin 


You have to make sure that you don't dump out the data section because that can
get very large! This is because, objcopy is turning the dynamic elf into a
statically mapped binary blob. Since ram starts at 0x8000'0000 the blob must
have ram mapped at that exact location for bss and data. Thus the blob becomes
over 2 gigabytes in size! This returns the baffling error "not enough space on
device" from i960-elf-objcopy.  

 */ 
