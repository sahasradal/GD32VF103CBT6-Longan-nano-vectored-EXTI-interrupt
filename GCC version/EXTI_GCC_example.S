.equ CSR_MMISC_CTL, 0x7d0
.equ CSR_MNVEC, 0x7C3
.equ CSR_MTVT,  0x307
.equ CSR_MTVT2, 0x7EC
.equ CSR_MTVEC, 0x305
.equ CSR_MCAUSE,0x342
.equ CSR_MEPC,  0x341
.equ CSR_MTVEC_ECLIC, 3
.equ CSR_MSTATUS, 0x300
.equ MSTATUS_MIE, (1<<3)

.equ ECLIC_ADDR_BASE, 0xd2000000
.equ ECLIC_INT_IP_OFFSET, 0x1000
.equ ECLIC_INT_IE_OFFSET, 0x1001
.equ ECLIC_INT_ATTR_OFFSET, 0x1002
.equ USART0_IRQn, 56


.equ AFIO_BASE_ADDR,    0x40010000
.equ AFIO_EC_OFFSET,  0x00
.equ AFIO_PCF0_OFFSET,  0x04
.equ AFIO_EXTISS0_OFFSET,  0x08
.equ AFIO_EXTISS1_OFFSET,  0x0C
.equ AFIO_EXTISS2_OFFSET,  0x10
.equ AFIO_EXTISS3_OFFSET,  0x14
.equ AFIO_PCF1_OFFSET,  0x1C

.equ EXTI_BASE_ADDR,  0x40010400
.equ EXTI_INTEN_OFFSET,  0x00
.equ EXTI_EVEN_OFFSET,  0x04
.equ EXTI_RTEN_OFFSET,  0x08
.equ EXTI_FTEN_OFFSET,  0x0C
.equ EXTI_SWIEV_OFFSET,  0x10
.equ EXTI_PD_OFFSET,  0x14



.equ RCU_APB2EN, 0x40021018
.equ RCU_APB2EN_AFEN, (1<<0)
.equ RCU_APB2EN_PAEN, (1<<2)
.equ RCU_APB2EN_PBEN, (1<<3)
.equ RCU_APB2EN_USART0EN, (1<<14)
.equ GPIO_BASE_ADDR_A,  0x40010800  # GD32VF103 Manual: Section 7.5 (green and blue LEDs)
.equ GPIO_BASE_ADDR_B,  0x40010C00
.equ GPIO_BASE_ADDR_C,  0x40011000  # GD32VF103 Manual: Section 7.5 (red LED)
.equ GPIO_BASE_ADDR_D,  0x40011400 
.equ GPIO_BASE_ADDR_E,  0x40011800 
.equ GPIOA_BASE, 0x40010800
.equ GPIOB_BASE, 0x40010C00
.equ GPIOA_CTL0, (GPIOA_BASE + 0x00)
.equ GPIOA_CTL1, (GPIOA_BASE + 0x04)
.equ GPIOB_CTL0, (GPIOB_BASE + 0x00)
.equ GPIOB_CTL1, (GPIOB_BASE + 0x04)
.equ GPIOB_OCTL, (GPIOB_BASE + 0x0C)
.equ GPIOB_ISTAT,(GPIOB_BASE + 0x08)

.equ GPIO_CTL0_OFFSET,  0x00 # GPIO port control register 0
.equ GPIO_CTL1_OFFSET,  0x04 # GPIO port control register 1
.equ GPIO_ISTAT_OFFSET, 0x08 # GPIO port input status register
.equ GPIO_OCTL_OFFSET,  0x0C # GPIO port output control register
.equ GPIO_BOP_OFFSET,   0x10 # GPIO port bit operation register
.equ GPIO_BC_OFFSET,    0x14 # GPIO bit clear register 

# combined CTL[1:0], MD[1:0]  for config and mode
.equ GPIO_MODE_IN_ANALOG, 0x0 #  analog input 
.equ GPIO_MODE_PP_10MHZ,  0x1 #  push-pull output, max speed 10MHz
.equ GPIO_MODE_PP_2MHZ,   0x2 #  push-pull output, max speed 2MHz
.equ GPIO_MODE_PP_50MHZ,  0x3 #  push-pull output, max speed 50MHz
.equ GPIO_MODE_IN_FLOAT,  0x4 #  floating input 
.equ GPIO_MODE_OD_10MHZ,  0x5 #  open-drain output, max speed 10MHz
.equ GPIO_MODE_OD_2MHZ,   0x6 #  open-drain output, max speed 2MHz
.equ GPIO_MODE_OD_50MHZ,  0x7 #  open-drain output, max speed 50MHz
.equ GPIO_MODE_IN_PULL,   0x8 #  input with pull up/down
.equ GPIO_MODE_AF_PP_10MHZ, 0x9 #  alternate function push-pull, max speed 10MHz
.equ GPIO_MODE_AF_PP_2MHZ,  0xa #  alternate function push-pull, max speed 2MHz
.equ GPIO_MODE_AF_PP_50MHZ, 0xb #  alternate function push-pull, max speed 50MHz
.equ GPIO_MODE_AF_OD_10MHZ, 0xd #  alternate function open-drain, max speed 10MHz
.equ GPIO_MODE_AF_OD_2MHZ,  0xe #  alternate function open-drain, max speed 2MHz
.equ GPIO_MODE_AF_OD_50MHZ, 0xf #  alternate function open-drain, max speed

.equ GPIO_MASK,		0b1111
.equ GPIO_HIZ,		0b0100
.equ GPIO_PP50,		0b0011
.equ GPIO_APP50,	0b1011

.equ USART0_BASE, 0x40013800
.equ USART_BAUD_OFFSET, 0x08
.equ USART_CTL0_OFFSET, 0x0C
.equ USART_DATA_OFFSET, 0x04
.equ USART_STAT_OFFSET, 0x00

.equ USART_CTL0_REN, (1<<2)
.equ USART_CTL0_TEN, (1<<3)
.equ USART_CTL0_UEN, (1<<13)
.equ USART_CTL0_TBEIE, (1<<7)
.equ USART_STAT_TBE, (1<<7)
.equ USART_STAT_RBNE,(1<<5)
#GPIOA
.equ UART0_TX, 9
.equ UART0_RX, 10
#GPIOB
.equ RLED, 5
.equ YLED, 6
.equ GLED, 7
.equ SBTN, 0
.equ RBTN, 1

USART0_IRQn = 56
EXTI0_IRQn = 25


.macro push val
  addi sp, sp, -4
  sw \val, 0(sp)
.endm

.macro pop val
  lw \val, 0(sp)
  addi sp, sp, 4
.endm

.text
.section .isr_vectors
    .weak  eclic_msip_handler
    .weak  eclic_mtip_handler
    .weak  eclic_bwei_handler
    .weak  eclic_pmovi_handler
    .weak  WWDGT_IRQHandler
    .weak  LVD_IRQHandler
    .weak  TAMPER_IRQHandler
    .weak  RTC_IRQHandler
    .weak  FMC_IRQHandler
    .weak  RCU_IRQHandler
    .weak  EXTI0_IRQHandler
    .weak  EXTI1_IRQHandler
    .weak  EXTI2_IRQHandler
    .weak  EXTI3_IRQHandler
    .weak  EXTI4_IRQHandler
    .weak  DMA0_Channel0_IRQHandler
    .weak  DMA0_Channel1_IRQHandler
    .weak  DMA0_Channel2_IRQHandler
    .weak  DMA0_Channel3_IRQHandler
    .weak  DMA0_Channel4_IRQHandler
    .weak  DMA0_Channel5_IRQHandler
    .weak  DMA0_Channel6_IRQHandler
    .weak  ADC0_1_IRQHandler
    .weak  CAN0_TX_IRQHandler
    .weak  CAN0_RX0_IRQHandler
    .weak  CAN0_RX1_IRQHandler
    .weak  CAN0_EWMC_IRQHandler
    .weak  EXTI5_9_IRQHandler
    .weak  TIMER0_BRK_IRQHandler
    .weak  TIMER0_UP_IRQHandler
    .weak  TIMER0_TRG_CMT_IRQHandler
    .weak  TIMER0_Channel_IRQHandler
    .weak  TIMER1_IRQHandler
    .weak  TIMER2_IRQHandler
    .weak  TIMER3_IRQHandler
    .weak  I2C0_EV_IRQHandler
    .weak  I2C0_ER_IRQHandler
    .weak  I2C1_EV_IRQHandler
    .weak  I2C1_ER_IRQHandler
    .weak  SPI0_IRQHandler
    .weak  SPI1_IRQHandler
    .weak  USART0_IRQHandler
    .weak  USART1_IRQHandler
    .weak  USART2_IRQHandler
    .weak  EXTI10_15_IRQHandler
    .weak  RTC_Alarm_IRQHandler
    .weak  USBFS_WKUP_IRQHandler
    .weak  EXMC_IRQHandler
    .weak  TIMER4_IRQHandler
    .weak  SPI2_IRQHandler
    .weak  UART3_IRQHandler
    .weak  UART4_IRQHandler
    .weak  TIMER5_IRQHandler
    .weak  TIMER6_IRQHandler
    .weak  DMA1_Channel0_IRQHandler
    .weak  DMA1_Channel1_IRQHandler
    .weak  DMA1_Channel2_IRQHandler
    .weak  DMA1_Channel3_IRQHandler
    .weak  DMA1_Channel4_IRQHandler
    .weak  CAN1_TX_IRQHandler
    .weak  CAN1_RX0_IRQHandler
    .weak  CAN1_RX1_IRQHandler
    .weak  CAN1_EWMC_IRQHandler
    .weak  USBFS_IRQHandler
    
.align 9
vector_base:
  j _start
  .align    2
  .word  0
  .word  0
  .word  eclic_msip_handler
  .word  0
  .word  0
  .word  0
  .word  eclic_mtip_handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  eclic_bwei_handler
  .word  eclic_pmovi_handler
  .word  WWDGT_IRQHandler
  .word  LVD_IRQHandler
  .word  TAMPER_IRQHandler
  .word  RTC_IRQHandler
  .word  FMC_IRQHandler
  .word  RCU_IRQHandler
  .word  EXTI0_IRQHandler
  .word  EXTI1_IRQHandler
  .word  EXTI2_IRQHandler
  .word  EXTI3_IRQHandler
  .word  EXTI4_IRQHandler
  .word  DMA0_Channel0_IRQHandler
  .word  DMA0_Channel1_IRQHandler
  .word  DMA0_Channel2_IRQHandler
  .word  DMA0_Channel3_IRQHandler
  .word  DMA0_Channel4_IRQHandler
  .word  DMA0_Channel5_IRQHandler
  .word  DMA0_Channel6_IRQHandler
  .word  ADC0_1_IRQHandler
  .word  CAN0_TX_IRQHandler
  .word  CAN0_RX0_IRQHandler
  .word  CAN0_RX1_IRQHandler
  .word  CAN0_EWMC_IRQHandler
  .word  EXTI5_9_IRQHandler
  .word  TIMER0_BRK_IRQHandler
  .word  TIMER0_UP_IRQHandler
  .word  TIMER0_TRG_CMT_IRQHandler
  .word  TIMER0_Channel_IRQHandler
  .word  TIMER1_IRQHandler
  .word  TIMER2_IRQHandler
  .word  TIMER3_IRQHandler
  .word  I2C0_EV_IRQHandler
  .word  I2C0_ER_IRQHandler
  .word  I2C1_EV_IRQHandler
  .word  I2C1_ER_IRQHandler
  .word  SPI0_IRQHandler
  .word  SPI1_IRQHandler
  .word  USART0_IRQHandler
  .word  USART1_IRQHandler
  .word  USART2_IRQHandler
  .word  EXTI10_15_IRQHandler
  .word  RTC_Alarm_IRQHandler
  .word  USBFS_WKUP_IRQHandler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  EXMC_IRQHandler
  .word  0
  .word  TIMER4_IRQHandler
  .word  SPI2_IRQHandler
  .word  UART3_IRQHandler
  .word  UART4_IRQHandler
  .word  TIMER5_IRQHandler
  .word  TIMER6_IRQHandler
  .word  DMA1_Channel0_IRQHandler
  .word  DMA1_Channel1_IRQHandler
  .word  DMA1_Channel2_IRQHandler
  .word  DMA1_Channel3_IRQHandler
  .word  DMA1_Channel4_IRQHandler
  .word  0
  .word  0
  .word  CAN1_TX_IRQHandler
  .word  CAN1_RX0_IRQHandler
  .word  CAN1_RX1_IRQHandler
  .word  CAN1_EWMC_IRQHandler
  .word  USBFS_IRQHandler

.align 2
.text
.global _start
_start:
  la sp, _stack_end		# load stack pointer
  
  la t0, vector_base
  csrw CSR_MTVT, t0 		#alignment to a power of two, more interrupts multiplied by 4 bytes, i.e. 512
  
  la t0, trap_entry		#address alignment must be at least 64 bytes
  andi t0, t0, ~(64-1)  	#clear lower 2 bits of the address , if 64 bit aligned address the lower bits will be always 0     	
  ori t0, t0, CSR_MTVEC_ECLIC	#set the lower 2 bits of MTVEC to enable ECLIC mode , if lower bits is 0 then CLINT mode
  csrw CSR_MTVEC, t0		#write to CSR register MTVEC

######################################################
# copy text to data section , initialize BSS to 0 etc
######################################################

#copy data section
  la a0, _data_load
  la a1, _data_start
  la a2, _data_end
  bgeu a1, a2, copy_data_end
copy_data_loop:
  lw t0, (a0)
  sw t0, (a1)
  addi a0, a0, 4
  addi a1, a1, 4
  bltu a1, a2, copy_data_loop
copy_data_end:
# Clear [bss] section
  la a0, _bss_start
  la a1, _bss_end
    bgeu a0, a1, clear_bss_end
clear_bss_loop:
  sw zero, (a0)
  addi a0, a0, 4
  bltu a0, a1, clear_bss_loop
clear_bss_end:
####################################################  

#Enable port clocks & alternate function clock  
  	la t0, RCU_APB2EN
  	lw	t1, 0(t0)
  	la t2, RCU_APB2EN_AFEN | RCU_APB2EN_PAEN | RCU_APB2EN_PBEN 
  	or	t1, t1, t2
  	sw	t1, 0(t0)

#enable PA1 & PA2 for led ,PA1 LED GREEN , PA2 LED BLUE on LONGAN NANO
	la a0,GPIO_BASE_ADDR_A						
	li a1,(( GPIO_MODE_PP_50MHZ << 8 | GPIO_MODE_PP_50MHZ << 4  )) 		# PA1 LED GREEN , PA2 LED BLUE
	sw a1,GPIO_CTL0_OFFSET(a0)
	li a1,(1 << 1) | (1 << 2 )			# both led off ,set bits	
	sw a1,GPIO_BOP_OFFSET(a0) 			# store word in GPIO_BOP register , led stayy off 

#enable PA8 for button press (boot button on LONGAN NANO)
	la a0,GPIO_BASE_ADDR_A				
	la a1,(  GPIO_MODE_IN_FLOAT << 0) 		# PA8 key , 
	sw a1,GPIO_CTL1_OFFSET(a0)
	
#select EXTI source
	la a0,AFIO_BASE_ADDR
	li a1,~(0xf)					# EXTI source PA8, EXTI 8 sources selection 0000: PA8 pin (refer user manual)
	sw a1,AFIO_EXTISS2_OFFSET(a0)

#enable line 8 interrupt
	la a0,EXTI_BASE_ADDR
	li a1,(1<<8)					# 1<<INTEN8 , enable EXTI interrupt for line # 8
	sw a1,EXTI_INTEN_OFFSET(a0)

#select edge of interrupt signal
	la a0,EXTI_BASE_ADDR
	li a1,(1<<8)
	sw a1,EXTI_FTEN_OFFSET(a0)			#enble falling edge by setting bit 8 of register
	sw a1,EXTI_RTEN_OFFSET(a0)			#enable rising edge by setting bit 8 of register

#clear pending exti flags
	la a0,EXTI_BASE_ADDR
	li t0,(1<<8)					#clear any pending interrupts on exti8 line by writing a 1 to bit 8 of EXTI_PD register
	sw t0,EXTI_PD_OFFSET(a0)


#vector_mode:
	li a6,0xD20010AA    				#((ECLIC_BASE_ADDR + ECLIC_CLICINATTR_OFFSET )+ (EXTI5_9_IRQn * 4)) = 0xD20010AA is where vector is enabled
	lw a3, 0(a6)					# read contents of location 0xD20010AA to a3
	andi a3,a3,~(0xff)				# clear the lower byte with bit mask ~0XFF
	ori a3,a3,0x1					# OR in 0x1 to enable vector mode
	sb a3, 0(a6)					# store the byte back to location 0xD20010AA 
	
#clear eclic pending interrupt
	li a6,0xD20010A8				# ((ECLIC_BASE_ADDR + ECLIC_CLICINTIP_OFFSET) + (EXTI5_9_IRQn * 4)) = 0xD20010A8
	lw a3, 0(a6)
	andi a3,a3,~(0xff)				# writing 0 will clear pending EXTI5_9 interrupt in ECLIC pending flag register
	sb a3, 0(a6)

#ECLIC_IRQ_ENABLE:					# EXTI5_9_IRQn = 42, 0x1001+A8 = 0x10A9 , base address 0xD2000000. D2000000+0x1001+A8 = 0xD20010A9
	li a6,0xD20010A9				# li a6 ,(ECLIC_BASE_ADDR + ECLIC_CLICINTIE_OFFSET + (EXTI5_9_IRQn * 4))   = 0xD20010A9
	lw a4,0(a6)
	andi a4,a4,~(0xff)				# clear lower byte
	ori a4,a4,0x1					# write 1 to enable EXTI5_9 interrupt
	sb a4,0(a6)					# store in EXTI5_9 interrupt enable offset 0xD20010A9

#enble global interrupt
	csrsi	mstatus,8				#csrrsi x0 ,CSR_MSTATUS,MSTATUS_MIE , MSTATUS_MIE = 8 or 1<< bit3


#green LED flash once to confirm excecution reached here
	call LED1ON
	call delay
	call LED1OFF

ML:							# min loop blue LED flashes continouesly till boot button is pressed for interrupt
	call LED2ON
	call delay
	call LED2OFF
	call delay
	j ML



.text
.align 6
trap_entry:
mret



EXTI5_9_IRQn_handler:					# ISR for EXTI5_9 interrupt , flash green led once and return to main loop to flash blue LED
	addi sp,sp,-20					# create space in stack for 5 words
	sw t0,16(sp)					# push t0
	sw a0,12(sp)					# push a0
	sw a1,8(sp)					# push a1
	sw t1,4(sp)					# push t1
	sw ra,0(sp)					# push ra
	li t0,1<<8					# load t0 with 1<<8 , value to clear EXTI5_9 interrupt flag 
	li a0,EXTI_BASE_ADDR				# 
	sw t0,EXTI_PD_OFFSET(a0)			# clear flag by writing 1 in EXTI8 position , 1<<8
	la a0,GPIO_BASE_ADDR_A				# GPIO A base address
	li a1, 1 << 1					# value of 1<<1 loaded in a1 
	sw a1,GPIO_BC_OFFSET(a0)			# set the bit clear register position1 to turn on green led
	li t1,2000000					# load counter value 2000000 
looop:
	addi t1,t1,-1					# subtract 1 from t1 (2000000)
	bne t1,zero,looop				# if t1 not equal to 0 branch to label loop
	li a0,GPIO_BASE_ADDR_A				# GPIO A base address
	li a1, 1 << 1					# value of 1 << 1 loaded in a1
	sw a1,GPIO_BOP_OFFSET(a0) 			# set bit 1 of BOP register to turn off green led
	lw ra,0(sp)					# pop ra
	lw t1,4(sp)					# pop t1
	lw a1,8(sp)					# pop a1
	lw a0,12(sp)					# pop a0
	lw t0,16(sp)					# pop t0
	add sp,sp,20					# reset SP to base value
	mret 						#( return from interrupt)



#delay subroutine
delay:					# delay routine
	li t1,2000000			# load an arbitarary value 20000000 to t1 register		
loop:
	addi t1,t1,-1			# subtract 1 from t1
	bne t1,zero,loop		# if t1 not equal to 0 branch to label loop
ret	



LED1ON:     							#GREEN LED
	li a0,GPIO_BASE_ADDR_A					# GPIO A base address
	li a1, 1 << 1						# value of 1 lhs 2 OR 1 lhs 2 loaded in a1
	sw a1,GPIO_BC_OFFSET(a0)
	ret
LED1OFF:							#GREEN LED
	li a0,GPIO_BASE_ADDR_A					# GPIO A base address
	li a1, 1 << 1						# value of 1 lhs 2 OR 1 lhs 2 loaded in a1
	sw a1,GPIO_BOP_OFFSET(a0) 
	ret

LED2ON:								#BLUE LED
	li a0,GPIO_BASE_ADDR_A					# GPIO A base address
	li a1, 1 << 2						# value of 1 lhs 2 OR 1 lhs 2 loaded in a1
	sw a1,GPIO_BC_OFFSET(a0)
	ret
LED2OFF:							#BLUE LED
	li a0,GPIO_BASE_ADDR_A					# GPIO A base address
	li a1, 1 << 2						# value of 1 lhs 2 OR 1 lhs 2 loaded in a1
	sw a1,GPIO_BOP_OFFSET(a0) 
	ret


