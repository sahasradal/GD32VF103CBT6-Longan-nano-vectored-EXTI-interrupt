# GD32VF103CBT6-Longan-nano-vectored-EXTI-interrupt
GD32VF103CBT6 Longan nano , vectored EXTI interrupt example
simple example of EXTI5_9 vectored ECLIC interrupt  in RISCV assembly language.
The code is assembled with light weight BRONZEBEARD assembler. https://github.com/theandrew168/bronzebeard#setup 
tested on LONGAN NANO BOARD (GD32VF103CBT6) . PA8 (boot button ) is used to generate interrupt.
The blue LED flashes continuously and when the boot button is pressed the green LED lights up and then switches back to blue flashing.
ECLIC vectored mode is used
pack<L instruction is used to store the ISR address in the vector table on assembly .
All unused vectors are filled with 0x00000000 , all reserved addresses in the vector table are also filled with 0x00000000
The vector table address is stored in CSR register MTVT. On interrupt the core uses this register as base and finds the ISR address with the IRQn + 4 offset
All CSR instructions are manually encoded into machine language as BRONZEBEARD assembler does not support CSR instructions at this stage.

       I have added a version of the same code that can be compiled with GCC/GAS. only that the structure of the vector table differs. CSR instructions are
       supported by GCC compiler. I have installed prebuilt version of the compiler on windows1o. The prebuilt tool chain for windows is "riscv32-embecosm-win64-gcc12.1.0" can be downloaded from https://www.embecosm.com/resources/to... . Un zip the pckage and add to path the bin folder. CD to directory with assembly and linker files from windows command window to assemble and link. I used the following commands to get the results.
       riscv32-unknown-elf-as -g EXTI_GCC_example.S -o EXTI_GCC_example.o            (assembles code  to object file)
       riscv32-unknown-elf-ld -T gd32vf103cbt6.ld  -Map=final.map EXTI_GCC_example.O            (links all sections of the object file)
       riscv32-unknown-elf-objcopy -O ihex a.out EXTI_GCC_example.hex                 (converts the ELF file to INTELHEX format)
       
       use GIGADEVICE DFU programming software to upload the .HEX file via usb cable to the longan nano board
       DFU tool
       download GD32vF103 programming tool from https://www.gd32mcu.com/en/download/7...

ALL IN ONE PROGRAMMER version 2.0.3.13854 (latest)
GD32 MCU DFU tool version 3.8.2.9056 (older)
GD32 DFU drivers version 3.6.6.6167
       
       ![image](https://user-images.githubusercontent.com/36818909/186472752-79c2e45b-aa29-4c90-905c-ff9522955961.png)
       
