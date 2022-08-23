include gd32vf103.asm


#LED_RED_PIN GPIOC_PIN_13
#LED_GRN_PIN GPIOA_PIN_1
#LED_BLU_PIN GPIOA_PIN_2
#tested working , blue led flashes, pressing boot button makes green led come on and the revert to blue flash
#use keyword "pack <l" with ISR label in the vector table , eg-- "pack <l EXTI0_IRQn_handler"
#reserved vectors should not be disturbed



vtable:
  j reset_handler
  align 4

  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
longs   0x00000000 #pack <l longs eclic_msip_handler
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
longs   0x00000000 #pack <l longs eclic_mtip_handler
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
longs   0x00000000 #pack <l longs eclic_bwei_handler
longs   0x00000000 #pack <l longs eclic_pmovi_handler
longs   0x00000000 #pack <l longs watchdog_IRQn_handler
longs   0x00000000 #pack <l LVD_IRQn_handler
longs   0x00000000 #pack <l tamper_IRQn_handler
longs   0x00000000 #pack <l RTC_IRQn_handler
longs   0x00000000 #pack <l FMC_IRQn_handler
longs   0x00000000 #pack <l RCU_IRQn_handler
longs   0x00000000 #pack <l EXTI0_IRQn_handler
longs   0x00000000 #pack <l EXTI1_IRQn_handler
longs   0x00000000 #pack <l EXTI2_IRQn_handler
longs   0x00000000 #pack <l EXTI3_IRQn_handler
longs   0x00000000 #pack <l EXTI4_IRQn_handler
longs   0x00000000 #pack <l DMA0_chan0_IRQn_handler
longs   0x00000000 #pack <l DMA0_chan1_IRQn_handler
longs   0x00000000 #pack <l DMA0_chan2_IRQn_handler
longs   0x00000000 #pack <l DMA0_chan3_IRQn_handler
longs   0x00000000 #pack <l DMA0_chan4_IRQn_handler
longs   0x00000000 #pack <l DMA0_chan5_IRQn_handler
longs   0x00000000 #pack <l DMA0_chan6_IRQn_handler
longs   0x00000000 #pack <l ADC0_1_IRQn_handler
longs   0x00000000 #pack <l CAN0_TX_IRQn_handler
longs   0x00000000 #pack <l CAN0_RX0_IRQn_handler
longs   0x00000000 #pack <l CAN0_RX1_IRQn_handler
longs   0x00000000 #pack <l CAN0_EWMC_IRQn_handler
pack <l EXTI5_9_IRQn_handler					# assembler stores the ISR address here for the core to jump on interrupt
longs   0x00000000 #pack <l TIM0_break_IRQn_handler
longs   0x00000000 #pack <l TIM0_update_IRQn_handler
longs   0x00000000 #pack <l TIM0_trigger_commutation_IRQn_handler
longs   0x00000000 #pack <l TIM0_channel_IRQn_handler
longs   0x00000000 #pack <l TIM1_IRQn_handler
longs   0x00000000 #pack <l TIM2_IRQn_handler
longs   0x00000000 #pack <l TIM3_IRQn_handler
longs   0x00000000 #pack <l I2C0_EV_IRQn_handler
longs   0x00000000 #pack <l I2C0_ER_IRQn_handler
longs   0x00000000 #pack <l I2C1_EV_IRQn_handler
longs   0x00000000 #pack <l I2C1_ER_IRQn_handler
longs   0x00000000 #pack <l SPI0_IRQn_handler
longs   0x00000000 #pack <l SPI1_IRQn_handler
longs   0x00000000 #pack <l USART0_IRQn_handler
longs   0x00000000 #pack <l USART1_IRQn_handler
longs   0x00000000 #pack <l USART2_IRQn_handler
longs   0x00000000 #pack <l EXTI10_15_IRQn_handler
longs   0x00000000 #pack <l RTC_alarm_IRQn_handler
longs   0x00000000 #pack <l USB_wakeup_IRQn_handler
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
longs   0x00000000 #pack <l EXMC_IRQn_handler
  longs 0x00000000 # RESERVED
longs   0x00000000 #pack <l TIM4_IRQn_handler
longs   0x00000000 #pack <l SPI2_IRQn_handler
longs   0x00000000 #pack <l UART3_IRQn_handler
longs   0x00000000 #pack <l UART4_IRQn_handler
longs   0x00000000 #pack <l TIM5_IRQn_handler
longs   0x00000000 #pack <l TIM6_IRQn_handler
longs   0x00000000 #pack <l DMA1_chan0_IRQn_handler
longs   0x00000000 #pack <l DMA1_chan1_IRQn_handler
longs   0x00000000 #pack <l DMA1_chan2_IRQn_handler
longs   0x00000000 #pack <l DMA1_chan3_IRQn_handler
longs   0x00000000 #pack <l DMA1_chan4_IRQn_handler
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
longs   0x00000000 #pack <l CAN1_TX_IRQn_handler
longs   0x00000000 #pack <l CAN1_RX0_IRQn_handler
longs   0x00000000 #pack <l CAN1_RX1_IRQn_handler
longs   0x00000000 #pack <l CAN1_EWMC_IRQn_handler
longs   0x00000000 #pack <l USB_IRQn_handler





align 2
reset_handler:

    	li sp, STACK			# load the stack pointer

	
	li   t0, vtable			# load t0 with address of vector table,core checks MTVT register for vector base on interrupt
longs   0x30729073			# machine code of csrrW x0 ,CSR_MTVT,t0 ( vector address loaded in t0)
 	#csrw CSR_MTVT, t0		# Set the vector table's base address.
	

	li t0, trap_entry		# The mtvec register is used to configure the entry address for interrupts and exception handlers. last 5 bits of the address should be 0 ,align accordingly
        li t1,0xFFFFFFC0
	and t0,t0,t1			# clear LSB of MTVEC to write interrupt mode
	ori t0,t0,0x00000003            # set both lower bits of MTVEC for ECLIC interrupt mode , if LSB =00 default interrupt mode (CLINT)
longs	0x30529073			# machine code of csrrW x0 ,CSR_MTVEC,t0 ( vector address loaded in t0)
  	#csrw CSR_MTVEC, t0		# machine code directly input as bronzebeard assembler does not support CSR instructions, GNU/GCC does


setup:
#Enable portA and portb clocks
            	
    	li s0, RCU_BASE_ADDR
    	lw a5, RCU_APB2EN_OFFSET(s0)
    	li a6, ( (1<<RCU_APB2EN_PAEN_BIT) | (1<<RCU_APB2EN_PBEN_BIT)  | (1<<RCU_APB2EN_AFEN_BIT))
	or a5, a5,a6
    	sw a5, RCU_APB2EN_OFFSET(s0)

#enable PA1 & PA2 for led ,PA1 LED GREEN , PA2 LED BLUE on LONGAN NANO
	li a0,GPIO_BASE_ADDR_A						
	li a1,(( GPIO_MODE_PP_50MHZ << 8 | GPIO_MODE_PP_50MHZ << 4  )) 		# PA1 LED GREEN , PA2 LED BLUE
	sw a1,GPIO_CTL0_OFFSET(a0)
	li a1,(1 << 1) | (1 << 2 )			# both led off ,set bits	
	sw a1,GPIO_BOP_OFFSET(a0) 			# store word in GPIO_BOP register , led stayy off 

#enable PA8 for button press (boot button on LONGAN NANO)
	li a0,GPIO_BASE_ADDR_A				
	li a1,(  GPIO_MODE_IN_FLOAT << 0) 		# PA8 key , 
	sw a1,GPIO_CTL1_OFFSET(a0)
	
#select EXTI source
	li a0,AFIO_BASE_ADDR
	li a1,~(0xf)					# EXTI source PA8, EXTI 8 sources selection 0000: PA8 pin (refer user manual)
	sw a1,AFIO_EXTISS2_OFFSET(a0)

#enable line 8 interrupt
	li a0,EXTI_BASE_ADDR
	li a1,(1<<8)					# 1<<INTEN8 , enable EXTI interrupt for line # 8
	sw a1,EXTI_INTEN_OFFSET(a0)

#select edge of interrupt signal
	li a0,EXTI_BASE_ADDR
	li a1,(1<<8)
	sw a1,EXTI_FTEN_OFFSET(a0)			#enble falling edge by setting bit 8 of register
	sw a1,EXTI_RTEN_OFFSET(a0)			#enable rising edge by setting bit 8 of register

#clear pending exti flags
	li a0,EXTI_BASE_ADDR
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

longs 0x30046073					# machine code of csrrsi x0 ,CSR_MSTATUS,MSTATUS_MIE , csrrsi Mstatus,bit3, enables global interrupt


	
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
	li a0,GPIO_BASE_ADDR_A				# GPIO A base address
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
longs   0x30200073					# mret ( return from interrupt)





align 64
trap_entry:
default_interrupt_handler:
    default_interrupt_loop:
    j default_interrupt_loop


          	



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


