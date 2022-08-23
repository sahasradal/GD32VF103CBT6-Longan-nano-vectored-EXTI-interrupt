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
