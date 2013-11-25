
; Program - keypad.asm
;
; Written by Michael Bengtson  24-Nov-2013
;
; This code is for a PIC16F1503 processor that controls an encoder.
;
; This is a prototype program to manage an illuminated encoder to be used in
; the Tack Sa Mycket home we are building.  This is only a prototype since
; the chip used here will likely be different than the chip in the final
; keypad controller.
;
; A timer is set to trigger every 1ms.  This will read the encoder bits and
; check the led pwm status.  LEDs can be set from 0-15 for their intensity.
;
; Need a little state machine to determine the direction of the encoder.

	list	p=16f1503
	include <p16F1503.inc>

;  Set the configuration bits for the chip.  Here's what gets set:
;
;		Bit 13    :   1 : Not used, reads as 1
;		Bit 12    :   1 : Not used, reads as 1.
;		Bit 11    :   0 : Fail Safe Clock Monitor Disabled
;		Bit 10	  :   0 : Internal External Switchover Disabled
;		Bit 09-08 :  00 : Brown-Out Detection Disabled
;		Bit  7    :   1 : Data Memory Protection Disabled
;		Bit  6    :   1 : Program Memory Protection Disabled
;		Bit  5    :   1 : MCLR is reset signal.
;		Bit  4    :   0 : Power Timer Enabled
;		Bit  3    :   1 : Watch Dog Timer Enabled
;		Bit 02-00 : 010 : High Speed Oscillator
;
;  Value of the configuration word is: 3F76

;	__CONFIG	_CONFIG1, _CP_OFF & _CCP1_RB0 & _DEBUG_OFF & _WRT_PROTECT_OFF & _CPD_OFF & _LVP_OFF & _BODEN_OFF & _MCLR_ON & _PWRTE_ON & _WDT_OFF & _INTRC_IO
	__CONFIG	_CONFIG1, 0x08e0
	__CONFIG	_CONFIG2, 0x0803

;  Define parameters about this device.
DEVICE_TYPE_BUTTONS		equ		001H
DEVICE_VERSION_MINOR	equ		000H
DEVICE_VERSION_MAJOR	equ		000H

;  Define an address for this device.  This will need to be user programmable
;  at some point.

DEVICE_ADDRESS          equ     000H

;  Define the locations in RAM necessary.  Valid RAM locations are 20-5F.
;  Only 64 bytes to use them wisely.

ram		equ	0x20		; First address of available RAM

;  Define all the variables used for this code.  These should fit between
;  0x20 and 0x7f.
	cblock 0x20
	endc

; ------------------------------------------------------------------------------
;
;  Vector Table
;
;  Setup the reset and interrupt vectors for the chip.

	org		0x00		; Set program memory base at reset vector 0x00.
	goto	main		; Go to start of the main program.

	org		0x04
	goto	system_isr			; Handle interrupts.

;
; ------------------------------------------------------------------------------

; ------------------------------------------------------------------------------
;
;  System Module
;
;  Here is where we initialize all the functions on the chip.

	org		0x08

;  Routine - system_init : Initializes the processor.
system_init

	banksel INTCON
	bcf		INTCON,GIE		; Disable all interrupts.
	bcf		INTCON,INTE

    ; Set the clock to 16Mhz.
    banksel OSCCON
    movlw   0x7A
    movwf   OSCCON

    ;  We are done setting everything up, go to main.
	return		; All done so return.


;  Routine - system_isr : This routine handles the interrupts for the
;  processor.  Simply save state then call each potential interrupt
;  source.

system_isr

    ;  Check for an i2c interrupt.
    call    i2c_isr

	;  Check for a timer interrupt.
	call	timer_isr

system_isr_return
	retfie

;  Routine - system_halt : If there are any errors, then the code jumps here
;  to hopefully report the error.

system_halt
    goto    system_halt

;  System Module : END
;
; ------------------------------------------------------------------------------

; ------------------------------------------------------------------------------
;
;  Encoder Module
;
;  This module handles the encoder.  This includes the reading of the encoder,
;  the button and driving the LEDs.  The I/O positions for the encoder are as
;  follows:
;
;       Enc A   Port A4     A before B when CCW, Closed is GND.
;       Enc B   Port A5

ENCODER_A_BIT       EQU 4
ENCODER_B_BIT       EQU 5

ENCODER_DIR_CW      equ 0       ; Clockwise bit 0 set.
ENCODER_DIR_CCW     equ 1       ; CCW bit 1 set.

;  Define variables for the encoder module.
    cblock
        encoder_state   ;  State for resolving encoder events.
        encoder_direction   ;  Encoder direction.
        encoder_position    ;  Position of the encoder 0-23
    endc

;  Routine - encoder_init : Sets the port direction bits appropriately.

encoder_init

    ; Set encoder and button assignments to inputs.
    banksel TRISA
    bsf     TRISA,ENCODER_A_BIT
    bsf     TRISA,ENCODER_B_BIT

    ; Turn on weak pull up resistors.
    banksel WPUA
    bsf     WPUA,ENCODER_A_BIT
    bsf     WPUA,ENCODER_B_BIT

    ; Set variables.
    banksel encoder_state
    clrf    encoder_state
    bsf     encoder_state,ENCODER_STATE_IDLE
    clrf    encoder_position

    ; Return to the caller.
    return

;  Routine - encoder_tick : Handles code that runs on 1ms boundaries.  This code
;  should generate user events from the encoder and the button.  It should also
;  set the LEDs to the approrpriate on or off state based on the LED setting.
;
;  Encoder State Machine
;
;       Idle        If A true then CW
;                   if B true then CCW
;
;       CW_START    if A false then Idle
;                   if B true then set DIR to CW; next state WAIT
;
;       CCS_START   if B false then Idle
;                   if A true then set DIR to CCW; next state WAIT
;
;       WAIT        if A and B false then Idle

ENCODER_STATE_IDLE      equ     0
ENCODER_STATE_CW        equ     1
ENCODER_STATE_CCW       equ     2
ENCODER_STATE_WAIT      equ     3

encoder_tick

    ; Go to the encoder state.
    banksel encoder_state

    btfsc   encoder_state,ENCODER_STATE_IDLE
    goto    enc_idle
    btfsc   encoder_state,ENCODER_STATE_CW
    goto    enc_cw
    btfsc   encoder_state,ENCODER_STATE_CCW
    goto    enc_ccw
    btfsc   encoder_state,ENCODER_STATE_WAIT
    goto    enc_wait
    goto    system_halt

    ; Handle the idle state.  Next state is cw or ccw if A or B set.
enc_idle
    btfsc   PORTA,ENCODER_A_BIT
    goto    enc_idle_check_b

    clrf    encoder_state
    bsf     encoder_state,ENCODER_STATE_CW
    goto    enc_read_complete

enc_idle_check_b
    btfsc   PORTA,ENCODER_B_BIT
    goto    enc_read_complete

    clrf    encoder_state
    bsf     encoder_state,ENCODER_STATE_CCW
    goto    enc_read_complete

    ; Handle the CW state.
enc_cw
    btfss   PORTA,ENCODER_A_BIT
    goto    enc_cw_check_b

    clrf    encoder_state
    bsf     encoder_state,ENCODER_STATE_IDLE
    goto    enc_read_complete

enc_cw_check_b
    btfsc   PORTA,ENCODER_B_BIT
    goto    enc_read_complete

    clrf    encoder_direction
    bsf     encoder_direction,ENCODER_DIR_CW
    clrf    encoder_state
    bsf     encoder_state,ENCODER_STATE_WAIT
    incf    encoder_position,F
    movlw   0x18
    xorwf   encoder_position,W
    btfsc   STATUS,Z
    clrf    encoder_position
    goto    enc_read_complete

    ; Handle the CCW state.
enc_ccw
    btfss   PORTA,ENCODER_B_BIT
    goto    enc_ccw_check_a

    clrf    encoder_state
    bsf     encoder_state,ENCODER_STATE_IDLE
    goto    enc_read_complete

enc_ccw_check_a
    btfsc   PORTA,ENCODER_A_BIT
    goto    enc_read_complete

    clrf    encoder_direction
    bsf     encoder_direction,ENCODER_DIR_CCW
    clrf    encoder_state
    bsf     encoder_state,ENCODER_STATE_WAIT

    decf    encoder_position,F
    btfss   encoder_position,7
    goto    enc_read_complete

    movlw   0x17
    movwf   encoder_position
    goto    enc_read_complete

    ; Handle the WAIT state.
enc_wait
    btfss   PORTA,ENCODER_A_BIT
    goto    enc_read_complete

    btfss   PORTA,ENCODER_B_BIT
    goto    enc_read_complete

    clrf    encoder_state
    bsf     encoder_state,ENCODER_STATE_IDLE
    goto    enc_read_complete

enc_read_complete

    return

;  Encoder Module : END
;
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
;
;  I2C Slave Module
;
;  This module provides the driver for the I2C slave interface.  Data can be
;  written to the RGB leds and the status of the button and rotary encoder
;  can be read.  The packet structure for reading and writing is as follows:
;
;       Byte 0 : Packet Type : Byte Count   High 4 bits are the packet type
;                                           while the low 4 bits are the byte
;                                           count (0-15);
;
;   LED Write Packet
;
;       Byte 0 : 13H                        Packet type 1 with 3 bytes.
;       Byte 1 : Red Value                  From Master
;       Byte 2 : Green Value                From Master
;       Byte 3 : Blue Value                 From Master
;
;       The led value is as follows:
;
;           Bits 3:0    Intensity of the LED (0-15)
;           Bits 5:4    Flash rate 0=1Hz, 1=2Hz, 2=4Hz, 3=8Hz
;           Bit    6    Phase - 0=Flash In Phase, 1=Flash 180 degrees out of phase.
;           Bit    7    Mode - 0=Steady, 1=Flash
;
;   Encoder Read Packet
;
;       Byte 0 : 21H                        Packet type 2 with 1 byte.
;       Byte 1 : Encoder Value              To Master
;
;       The encoder/button value is as follows:
;
;           Bits 6:0    Current encoder position
;           Bit    7    Button - 0=Not Pressed, 1=Pressed
;
;  All other packet types are reserved.  Note that there is not a way to read
;  the value of an LED or set the encoder position.  This is a raw interface
;  but does have guarenteed delivery of packets.
;
;  Lines used for the bus are:
;
;       SCL     Port C  Bit 0       Clock Line
;       SCD     Port C  Bit 1       Data Line
;
;  These lines need to be set as inputs.
;
;  The interface is tested to run at up to 400KHz.  It can likely run faster but
;  driver overhead starts to be a significant part of packet delivery time.

I2C_CLOCK_BIT   EQU     0
I2C_DATA_BIT    EQU     1

I2C_ADDRESS     EQU     0x10        ;  Address, not byte to write.

;  Bit definitions for state.
I2C_STATE_START     EQU     0       ; Waiting for start condition.
I2C_STATE_ADDRESS   EQU     1
I2C_STATE_TYPE      EQU     2
I2C_STATE_LED       EQU     3
I2C_STATE_STOP      EQU     4
I2C_STATE_ENCODER   EQU     5

;  Define i2c variables.
    cblock
        i2c_state           ;  State for running the driver.
        i2c_data_count      ;  Counts bytes of data read.
        i2c_packet_type     ;  Holds the type of packet.
    endc

;  Routine - i2c_init : Initializes the serial bus.

i2c_init

    ; Set the serial bus lines to inputs.

    banksel CLC2CON
    clrf    CLC2CON
    banksel NCO1CON
    clrf    NCO1CON
    banksel PWM4CON
    clrf    PWM4CON
    banksel APFCON
    bsf     APFCON,NCO1SEL

    banksel TRISC
    bsf     TRISC,I2C_CLOCK_BIT
    bsf     TRISC,I2C_DATA_BIT

    banksel PIE1
    bsf     PIE1,SSP1IE     ; Enable SSP Interrupt
    bcf     PIE2,BCL1IF     ; Enable Bus Collision Interrupts

    ; Set the control register
    banksel SSP1CON1
    clrf    SSP1MSK
    clrf    SSP1CON1
    clrf    SSP1CON2
    clrf    SSP1CON3
    bsf     SSP1STAT,SMP
    bsf     SSP1STAT,CKE
    movlw   I2C_ADDRESS<<1
    movwf   SSP1ADD
    bcf     SSP1CON1,SSPM0
    bsf     SSP1CON1,SSPM1
    bsf     SSP1CON1,SSPM2
    bsf     SSP1CON1,SSPM3
    bsf     SSP1CON1,SSPEN

    ; Set up the state machine for reading the request for data.
    banksel i2c_state
    clrf    i2c_state
    bsf     i2c_state,I2C_STATE_START

    return

;  Method - i2c_isr : This is called for every interrupt that is generated
;  by the i2c hardware.

i2c_isr

    ; See if the i2c interface generated an interrupt.
    banksel PIR1
    btfss   PIR1,SSP1IF
    goto    i2c_isr_return

    ; Handle the expected state of the bus.
    btfsc   i2c_state,I2C_STATE_START
    goto    i2c_state_start
    btfsc   i2c_state,I2C_STATE_ADDRESS
    goto    i2c_state_address
    btfsc   i2c_state,I2C_STATE_TYPE
    goto    i2c_state_type
    btfsc   i2c_state,I2C_STATE_LED
    goto    i2c_state_led
    btfsc   i2c_state,I2C_STATE_STOP
    goto    i2c_state_stop

    ; If we get here, we have a state problem. We should reset the chip.
    reset

    ; State - START : We are expecting a start interrupt.  Start bit should
    ; be set.  If so, clear the interrupt bit and setup for the next state.
i2c_state_start

    ; Reset the interrupt.
    banksel PIR1
    bcf     PIR1,SSP1IF

    ; Check for the start bit.
    banksel SSP1STAT
    btfsc   SSP1STAT,S
    goto    i2c_state_start_ok

    ; If there is a problem leave the state the same and exit.  Eventually
    ; a start packet shoud come along.
    goto    i2c_state_unknown_interrupt

    ; We are good so set the next state to collect the address.
i2c_state_start_ok

    banksel i2c_state
    clrf    i2c_state
    bsf     i2c_state,I2C_STATE_ADDRESS
    goto    i2c_isr_return

    ; We are expecting an address interrupt.  This would indicate that we
    ; have a matching address.  We should read the address from the buffer
    ; We should check the D/A bit and the BF bit.
i2c_state_address

    banksel PIR1
    bcf     PIR1,SSP1IF

    call    i2c_isr_read_buffer
    btfsc   STATUS,C
    goto    i2c_state_unknown_interrupt

    btfsc   STATUS,Z
    goto    i2c_state_unknown_interrupt

    banksel i2c_state
    clrf    i2c_state
    bsf     i2c_state,I2C_STATE_TYPE
    goto    i2c_isr_return

    ; State - TYPE : Read the first byte of data to get the packet type and
    ; the byte count.
i2c_state_type

    banksel PIR1
    bcf     PIR1,SSP1IF

    call    i2c_isr_read_buffer
    btfsc   STATUS,C
    goto    i2c_state_unknown_interrupt

    btfss   STATUS,Z
    goto    i2c_state_unknown_interrupt

    ; Check to see what type of packet we have.
    banksel i2c_state
    andlw   0xf0
    movwf   i2c_packet_type
    swapf   i2c_packet_type

    movfw   i2c_packet_type
    xorlw   0x01
    btfsc   STATUS,Z
    goto    i2c_state_type_led

    movfw   i2c_packet_type
    xorlw   0x02
    btfsc   STATUS,Z
    goto    i2c_state_type_encoder

    ; Invalid packet type so we should reset to the start state.
me  goto    me

i2c_state_type_led
    banksel i2c_state
    clrf    i2c_state
    bsf     i2c_state,I2C_STATE_LED
    clrf    i2c_data_count
    goto    i2c_isr_return

i2c_state_type_encoder
    banksel i2c_state
    clrf    i2c_state
    bsf     i2c_state,I2C_STATE_ENCODER
    clrf    i2c_data_count
    goto    i2c_isr_return

    ; State - ENCODER : Reads the encoder.
i2c_state_encoder
me2  goto    me2

    ; State - LED : Reads the led data writing it to the led intensity locations.

i2c_state_led

    ; Set up the indirect addressing registers to write the data once we have it.
    banksel i2c_data_count
    movlw   led_intensity
    movwf   FSR0L
    clrf    FSR0H
    movfw   i2c_data_count
    addwf   FSR0L,F

    banksel PIR1
    bcf     PIR1,SSP1IF

    call    i2c_isr_read_buffer
    btfsc   STATUS,C
    goto    i2c_state_unknown_interrupt

    btfss   STATUS,Z
    goto    i2c_state_unknown_interrupt

    banksel i2c_data_count
    movwf   INDF0

    incf    i2c_data_count
    movfw   i2c_data_count
    xorlw   0x03
    btfss   STATUS,Z
    goto    i2c_isr_return

    ; Next state is to wait for the stop bit interrupt.
    clrf    i2c_state
    bsf     i2c_state,I2C_STATE_STOP
    goto    i2c_isr_return

    ; State - STOP : We are expecting a stop interrupt.  Stop bit should be set.
    ; If so, clear the interrupt bit and setup for the start state.
i2c_state_stop
    banksel SSP1STAT
    btfsc   SSP1STAT,P
    goto    i2c_state_stop_ok

    movlw   0xfe
    goto    i2c_state_unknown_interrupt

i2c_state_stop_ok
    banksel PIR1
    bcf     PIR1,SSP1IF

    banksel i2c_state
    clrf    i2c_state
    bsf     i2c_state,I2C_STATE_START
;    movlw   0x2a
;    call    led_write
    goto    i2c_isr_return

    ; If we get here, we have some type of problem.  Should probably report
    ; an error.
i2c_state_unknown_interrupt
    call    led_write
    banksel PIR1
    bcf     PIR1,SSP1IF
    banksel PIE1
    bcf     PIE1,SSP1IE     ; Enable SSP Interrupt
    bcf     PIE2,BCL1IF     ; Enable Bus Collision Interrupts
;    goto    system_halt

i2c_isr_return
    return;

;  Method - i2c_isr_read_buffer : Reads the data in the slave buffer.  The
;  data is returned in the W register.  Z set if address, clear if data.
;  C clear if ok, set if error.

i2c_isr_read_buffer

    ; Clear the interrupt.  At least for the address byte, this needs to be
    ; done before the BF but is set.
    banksel PIR1
    bcf     PIR1,SSP1IF

    ; Make sure there is data to read.
    banksel SSP1STAT
    btfsc   SSP1STAT,BF
    goto    i2c_isr_read_buffer_bf_ok

    ; No data so set an error.  Z is irrelevant.
    bsf     STATUS,C
    bcf     STATUS,Z
    return

    ; Get the data.
i2c_isr_read_buffer_bf_ok

    banksel SSP1BUF
    movfw   SSP1BUF

    ; Determine if this is an address or data.
    bcf     STATUS,Z
    btfsc   SSP1STAT,D_NOT_A
    bsf     STATUS,Z

    ; Return to the caller
    return

;  I2C Module : END
;
; ------------------------------------------------------------------------------

;    ifdef   master
;; ------------------------------------------------------------------------------
;;
;;  Master I2C Module
;;
;;  This module provides the driver for the I2C interface.  The I2C bus is
;;  used to send encoder and button presses to the controller and to receive
;;  LED commands from the controller.  Encoder and button presses are sent
;;  using the Master mode and LED commands are received as a slave.
;;
;;  Lines used for the bus are:
;;
;;       SCL     Port C  Bit 0       Clock Line
;;       SCD     Port C  Bit 1       Data Line
;;
;;  These lines need to be set as inputs.
;;
;;  Timing analysis - The 5 byte packet that sets the RGB leds is being sent
;;  from the master and received by the slave in 208us.  This gives more than
;;  enough time to read eight rotary encoders at 100Hz.  Assume 42us/byte.  The
;;  read encoder packet is 2 bytes * 8 encoders = 16 bytes.  This is .672 ms for
;;  a single read.  Max read rate is 1,488 / second.
;
;I2C_CLOCK_BIT   EQU     0
;I2C_DATA_BIT    EQU     1
;
;I2C_STATUS_QUEUE_RESERVED   EQU     0
;I2C_STATUS_READY            EQU     1       ;  Interface is ready to send.
;I2C_QUEUE_LENGTH     EQU     20
;
;;  Define states.
;I2C_STATE_READY             EQU     0
;I2C_STATE_START             EQU     1
;I2C_STATE_ADDRESS           EQU     2
;I2C_STATE_DATA              EQU     3
;I2C_STATE_STOP              EQU     4
;I2C_STATE_STOPPED           EQU     5
;
;;  Define i2c variables.
;    cblock
;        i2c_status          ; Status bits for the driver.
;        i2c_state           ;  State for running the driver.
;        i2c_queue:I2C_QUEUE_LENGTH-1        ; 16 bytes for write queue.
;        i2c_queue_end       ; Last byte in queue.
;        i2c_queue_head            ; Head of queue.
;        i2c_queue_tail            ; Tail of queue.
;        i2c_queue_tail_next       ; Used for queuing data.
;        i2c_queue_save      ; Used to save new byte to add to the queue.
;        i2c_data_count      ; Counts data bytes to write or read.
;        i2c_packet_pointer  ; Points to next byte to send.
;    endc
;
;;  About The Queue : The head pointer will always point to the next byte to
;;  be pulled from the queue.  The tail pointer will always point to the next
;;  free byte available in the queue.  If H == T, the queue is empty.  Code will
;;  never let the H == T if there is data in the queue.  Effectively, this
;;  disallows the last byte in the circular queue.  But that's just what it
;;  needs to be.
;
;;  Routine - i2c_write_reserve : This reserves the queue for adding a new
;;  write command.  Once data has been written, then the queue should be
;;  released.  Carry is set if the reserve failed.
;
;i2c_queue_reserve
;
;    banksel i2c_status
;    bsf     STATUS,C
;
;    btfsc   i2c_status,I2C_STATUS_QUEUE_RESERVED
;    return
;
;    bsf     i2c_status,I2C_STATUS_QUEUE_RESERVED
;    bcf     STATUS,C
;    movfw   i2c_queue_tail
;    movwf   i2c_queue_tail_next
;
;    return
;
;;  Routine - i2c_write_release : This releases the queue and sets the new
;;  queue tail to include the bytes that have been written to the queue.
;
;i2c_queue_release
;
;    banksel i2c_status
;
;    movfw   i2c_queue_tail_next
;    movwf   i2c_queue_tail
;    bcf     i2c_status,I2C_STATUS_QUEUE_RESERVED
;
;    ; Call start to start the transmission.
;
;    movlw   0x43
;    call    led_write
;    call    i2c_write_start
;    return
;
;;  Routine - i2c_write_start : Starts writing a packet of data.
;
;i2c_write_start
;
;    banksel i2c_state
;    btfss   i2c_state,I2C_STATE_READY
;    return
;
;    clrf    i2c_state
;    bsf     i2c_state,I2C_STATE_START
;
;    banksel SSP1CON2
;    bsf     SSP1CON2,SEN
;    movlw   0xaa
;    call    led_write
;    return
;
;;  Routine - i2c_write_queue : Adds the byte in W to the write queue.  Checks
;;  to make sure there is enough memory.  Carry is set if there is not room
;;  in the queue, otherwise C is clear.
;
;i2c_queue_write
;
;    banksel i2c_status
;
;    btfss   i2c_status,I2C_STATUS_QUEUE_RESERVED
;    goto    i2c_queue_write_fail
;
;    movwf   i2c_queue_save
;    movfw   i2c_queue_tail_next
;    movwf   FSR0L
;    clrf    FSR0H
;    movfw   i2c_queue_save
;    movwf   INDF0
;    incf    i2c_queue_tail_next
;
;    ; Wrap the tail pointer back to the start of the queue if necessary.
;
;    movlw   i2c_queue+I2C_QUEUE_LENGTH
;    xorwf   i2c_queue_tail_next,W
;    btfss   STATUS,Z
;    goto    i2c_queue_write_check
;
;    movlw   i2c_queue
;    movwf   i2c_queue_tail_next
;
;    ; See if there was room in the queue to write this byte.  It was ok to
;    ; write the byte to the queue since the queue operational definition will
;    ; always have an extra unused byte.
;i2c_queue_write_check
;    movfw   i2c_queue_head
;    xorwf   i2c_queue_tail_next,W
;    btfss   STATUS,Z
;    goto    i2c_queue_write_ok
;
;i2c_queue_write_fail
;    bcf     i2c_status,I2C_STATUS_QUEUE_RESERVED
;    bsf     STATUS,C
;    return
;
;i2c_queue_write_ok
;    bcf     STATUS,C
;    return
;
;;  Routine - i2c_init : Initializes the serial bus.
;
;i2c_init
;
;    ; Set the serial bus lines to inputs.
;
;    banksel CLC2CON
;    clrf    CLC2CON
;    banksel NCO1CON
;    clrf    NCO1CON
;    banksel PWM4CON
;    clrf    PWM4CON
;    banksel APFCON
;    bsf     APFCON,NCO1SEL
;
;    banksel TRISC
;    bsf     TRISC,I2C_CLOCK_BIT
;    bsf     TRISC,I2C_DATA_BIT
;
;    banksel PIE1
;    bsf     PIE1,SSP1IE     ; Enable SSP Interrupt
;    bcf     PIE2,BCL1IF     ; Enable Bus Collision Interrupts
;
;    ; Set the control register
;    banksel SSP1CON1
;    clrf    SSP1MSK
;    clrf    SSP1CON1
;    clrf    SSP1CON2
;    clrf    SSP1CON3
;    bsf     SSP1STAT,SMP
;    bsf     SSP1STAT,CKE
;;    movlw   0x28
;    movlw   0x09            ; Clock 16Mhz/ (4*(9+1)) = 400,000
;    movwf   SSP1ADD
;    bcf     SSP1CON1,SSPM0
;    bcf     SSP1CON1,SSPM1
;    bcf     SSP1CON1,SSPM2
;    bsf     SSP1CON1,SSPM3
;    bsf     SSP1CON1,SSPEN
;
;    ; Initialize the queue.
;    banksel i2c_status
;    movlw   i2c_queue
;    movwf   i2c_queue_head
;    movwf   i2c_queue_tail
;    bcf     i2c_status,I2C_STATUS_QUEUE_RESERVED
;    clrf    i2c_state
;    bsf     i2c_state,I2C_STATE_READY
;
;    return
;
;;  Method - i2c_isr : This is called for every interrupt that is generated
;;  by the i2c hardware.
;
;i2c_isr
;
;    ; See if the i2c interface generated an interrupt.
;    banksel PIR1
;    btfss   PIR1,SSP1IF
;    goto    i2c_isr_return
;
;    ; Handle the expected state of the bus.
;    btfsc   i2c_state,I2C_STATE_READY
;    goto    i2c_state_ready
;    btfsc   i2c_state,I2C_STATE_START
;    goto    i2c_state_start
;    btfsc   i2c_state,I2C_STATE_ADDRESS
;    goto    i2c_state_address
;    btfsc   i2c_state,I2C_STATE_DATA
;    goto    i2c_state_data
;    btfsc   i2c_state,I2C_STATE_STOP
;    goto    i2c_state_stop
;    btfsc   i2c_state,I2C_STATE_STOPPED
;    goto    i2c_state_stopped
;
;    ; If we get here, we have a state problem.
;    movlw   0xff
;    call    led_write
;    banksel PIR1
;    bcf     PIR1,SSP1IF
;
;    goto    i2c_isr_return
;
;    ; If we get here, we have a state problem.
;    banksel PIR1
;    bcf     PIR1,SSP1IF
;    movlw   0xff
;    call    led_write
;    goto    i2c_isr_return
;
;    ; State - READY : If we get an interrupt in the ready state then I'm not
;    ; sure what to do with it.  Flag it, turn it off and keep going.
;i2c_state_ready
;    banksel PIR1
;    bcf     PIR1,SSP1IF
;    movlw   0xdd
;    call    led_write
;    goto    i2c_isr_return
;
;    ; We are expecting a start interrupt.  Start bit should be set.  If so,
;    ; clear the interrupt bit and setup for the next state.
;i2c_state_start
;
;    banksel SSP1STAT
;    btfsc   SSP1STAT,S
;    goto    i2c_state_start_ok
;
;    movlw   0xfe
;    goto    i2c_state_unknown_interrupt
;
;i2c_state_start_ok
;    banksel PIR1
;    bcf     PIR1,SSP1IF
;
;    banksel i2c_state
;    clrf    i2c_state
;    bsf     i2c_state,I2C_STATE_ADDRESS
;
;    ; Get the address to send the data to.  This is the first byte at the
;    ; head of the queue.
;    banksel i2c_queue_head
;    movfw   i2c_queue_head
;    movwf   i2c_packet_pointer
;    movwf   FSR0L
;    clrf    FSR0H
;    movfw   INDF0
;
;    banksel SSP1BUF
;    movwf   SSP1BUF
;
;    banksel i2c_packet_pointer
;    incf    i2c_packet_pointer
;
;    movlw   i2c_queue+I2C_QUEUE_LENGTH
;    xorwf   i2c_packet_pointer,W
;    btfss   STATUS,Z
;    goto    i2c_isr_return
;
;    movlw   i2c_queue
;    movwf   i2c_packet_pointer
;
;    call    led_write
;    goto    i2c_isr_return
;
;    ; State - ADDRESS : Verify that the address cycle completed ok.  The ACKSTAT
;    ; bit should be set if we have contacted the slave.
;
;i2c_state_address
;
;    banksel SSP1CON2
;    btfss   SSP1CON2,ACKSTAT
;    goto    i2c_state_address_ok
;
;    ; We got a negative acknowledge ... probably no slave at this address.
;    movlw   0x54
;    call    led_write
;    banksel PIR1
;    bcf     PIR1,SSP1IF
;    goto    i2c_isr_return
;
;    ; The address was ok.  Set up to write the packet.
;i2c_state_address_ok
;    banksel i2c_state
;    clrf    i2c_state
;    bsf     i2c_state,I2C_STATE_DATA
;
;    ; Get the address to send the data to.  This is the first byte at the
;    ; head of the queue.
;    banksel i2c_queue_head
;    movfw   i2c_packet_pointer
;    movwf   FSR0L
;    clrf    FSR0H
;    movfw   INDF0
;
;    banksel SSP1BUF         ;  Write the packet type/length byte.
;    movwf   SSP1BUF
;
;    banksel i2c_data_count
;    andlw   0x0F            ;  Get the byte count.
;    movwf   i2c_data_count
;
;    banksel PIR1
;    bcf     PIR1,SSP1IF
;
;    banksel i2c_packet_pointer
;    incf    i2c_packet_pointer
;
;    movlw   i2c_queue+I2C_QUEUE_LENGTH
;    xorwf   i2c_packet_pointer,W
;    btfss   STATUS,Z
;    goto    i2c_isr_return
;
;    movlw   i2c_queue
;    movwf   i2c_packet_pointer
;    goto    i2c_isr_return
;
;    ; State - DATA : Check to see if the data was sent ok.  Then write the next
;    ; byte of data if there is another to write.
;
;i2c_state_data
;
;    banksel PIR1
;    bcf     PIR1,SSP1IF
;
;    banksel SSP1CON2
;    btfss   SSP1CON2,ACKSTAT
;    goto    i2c_state_data_ok
;
;    ; We got a negative acknowledge ... probably no slave at this address.
;    movlw   0x56
;    call    led_write
;    goto    i2c_isr_return
;
;    ; We need to get and send the next byte.
;
;i2c_state_data_ok
;
;    ; Get the address to send the data to.  This is the first byte at the
;    ; head of the queue.
;    banksel i2c_queue_head
;    movfw   i2c_packet_pointer
;    movwf   FSR0L
;    clrf    FSR0H
;    movfw   INDF0
;
;    banksel SSP1BUF         ;  Write the packet type/length byte.
;    movwf   SSP1BUF
;
;    banksel i2c_packet_pointer
;    incf    i2c_packet_pointer
;
;    movlw   i2c_queue+I2C_QUEUE_LENGTH
;    xorwf   i2c_packet_pointer,W
;    btfss   STATUS,Z
;    goto    i2c_state_data_loop
;
;    movlw   i2c_queue
;    movwf   i2c_packet_pointer
;
;i2c_state_data_loop
;    banksel i2c_data_count
;    decfsz  i2c_data_count
;    goto    i2c_isr_return
;
;    clrf    i2c_state
;    bsf     i2c_state,I2C_STATE_STOP
;
;    goto    i2c_isr_return
;
;    ; State - STOP : This sets the stop condition.
;i2c_state_stop
;    banksel PIR1
;    bcf     PIR1,SSP1IF
;
;    banksel SSP1CON2
;    btfss   SSP1CON2,ACKSTAT
;    goto    i2c_state_stop_ok
;
;    ; We got a negative acknowledge ... probably no slave at this address.
;    movlw   0x56
;    call    led_write
;    goto    i2c_isr_return
;
;    ; Command the stop bit.
;i2c_state_stop_ok
;    banksel SSP1CON2
;    bsf     SSP1CON2,PEN
;
;    banksel i2c_state
;    clrf    i2c_state
;    bsf     i2c_state,I2C_STATE_STOPPED
;    goto    i2c_isr_return
;
;    ; State - STOPPED : Make sure we get the stop bit.
;i2c_state_stopped
;
;    banksel PIR1
;    bcf     PIR1,SSP1IF
;
;    banksel SSP1STAT
;    btfsc   SSP1STAT,P
;    goto    i2c_state_stopped_ok
;
;;    movlw   0xf2
;    movfw   SSP1STAT
;    goto    i2c_state_unknown_interrupt
;
;i2c_state_stopped_ok
;; !!! Update head/tail and possibly start another write.
;    banksel i2c_packet_pointer
;    movfw   i2c_packet_pointer
;    movwf   i2c_queue_head
;    xorwf   i2c_queue_tail,W
;    btfss   STATUS,Z
;    goto    i2c_isr_new_start
;
;    ; We don't have anything else in the queue so stop.
;    clrf    i2c_state
;    bsf     i2c_state,I2C_STATE_READY
;;    movlw   0x33
;;    call    led_write
;    goto    i2c_isr_return
;
;i2c_isr_new_start
;    clrf    i2c_state
;    bsf     i2c_state,I2C_STATE_READY
;    call    i2c_write_start
;    goto    i2c_isr_return
;
;    ; If we get here, we have some type of problem.  Should probably report
;    ; an error.
;i2c_state_unknown_interrupt
;    call    led_write
;    banksel PIR1
;    bcf     PIR1,SSP1IF
;    banksel PIE1
;    bcf     PIE1,SSP1IE     ; Disable SSP Interrupt
;    bcf     PIE2,BCL1IF     ; Disable Bus Collision Interrupts
;
;i2c_isr_return
;    return;
;
;;  Method - i2c_write : This starts a write of the specified byte to the
;;  slave address specified.
;
;;  I2C Module : END
;;
;; ------------------------------------------------------------------------------
;    endif

; ------------------------------------------------------------------------------
;
;  Button Module
;
;  This module handles the button.  It handles initialization and the process
;  of determining it's state.  The button sends raw state information which 
;  includes the following events:
;
;       Pressed
;       Released
;
;  Additionally, the events are time stamped.  Time stamps are necessary to
;  determine high level events such as clicked, double clicked.
;
;  The I/O position for the button is:
;
;       Button  Port B5     Closed is Vcc

BUTTON_BIT  EQU 2       ; Port A bit 2.

;  Define variables for the button module.
    cblock
        button_state        ;  State for resolving button events.
        button_wait         ;  8ms counter for debouncing.
    endc

;  Routine - button_init : Sets the port direction bits appropriately.

button_init

    ; Set encoder and button assignments to inputs.
    banksel TRISA
    bsf     TRISA,BUTTON_BIT

    ; Set variables.
    banksel button_state
    clrf    button_state
    bsf     button_state,BUTTON_STATE_IDLE
    clrf    button_wait

    ; Return to the caller.
    return

;  Routine - button_tick : Handles code that runs on 1ms boundaries.  This code
;  should generate user events from the button.
;
;  Button State Machine
;
;       IDLE            Button is not pressed.
;                       If press detected next state is WAIT_PRESSED
;
;       WAIT_PRESSED    Wait for 8ms before press is confirmed.
;                       If button not pressed, next state is IDLE
;                       If button pressed for 8ms, next state is PRESSED.
;
;       PRESSED         If button not pressed, next state is WAIT_RELEASE
;
;       WAIT_RELEASED   Wait for release to be confirmed.
;                       If button is pressed, goto PRESSED.
;                       If button not pressed for 8ms, goto IDLE.

BUTTON_STATE_IDLE               equ     0
BUTTON_STATE_WAIT_PRESSED       equ     1
BUTTON_STATE_PRESSED           equ     2
BUTTON_STATE_WAIT_RELEASED     equ     3

button_tick

    ; Go to the encoder state.
    banksel button_state

    btfsc   button_state,BUTTON_STATE_IDLE
    goto    button_idle
    btfsc   button_state,BUTTON_STATE_WAIT_PRESSED
    goto    button_wait_pressed
    btfsc   button_state,BUTTON_STATE_PRESSED
    goto    button_pressed
    btfsc   button_state,BUTTON_STATE_WAIT_RELEASED
    goto    button_wait_released
    goto    system_halt

    ; Handle the idle state.  Next state is wait pressed if button is pressed.
button_idle
    btfss   PORTA,BUTTON_BIT
    goto    button_read_complete

    clrf    button_state
    bsf     button_state,BUTTON_STATE_WAIT_PRESSED
    clrf    button_wait
    goto    button_read_complete

    ; Wait 8ms for button to stabilize.
button_wait_pressed

    btfss   PORTA,BUTTON_BIT
    goto    button_press_bounce

    incf    button_wait
    btfss   button_wait,3
    goto    button_read_complete

    clrf    button_state
    bsf     button_state,BUTTON_STATE_PRESSED
    goto    button_read_complete

button_press_bounce

    clrf    button_state
    bsf     button_state,BUTTON_STATE_IDLE
    goto    button_read_complete

    ; Wait for the button to be released.
button_pressed

    btfsc   PORTA,BUTTON_BIT
    goto    button_read_complete

    clrf    button_state
    bsf     button_state,BUTTON_STATE_WAIT_RELEASED
    goto    button_read_complete

    ; Debounce the button release.
button_wait_released

    btfsc   PORTA,BUTTON_BIT
    goto    button_release_bounce

    incf    button_wait
    btfss   button_wait,3
    goto    button_read_complete

    clrf    button_state
    bsf     button_state,BUTTON_STATE_IDLE
    goto    button_read_complete

button_release_bounce

    clrf    button_state
    bsf     button_state,BUTTON_STATE_PRESSED
    goto    button_read_complete

button_read_complete

    return

;  Button Module : END
;
; ------------------------------------------------------------------------------

; ------------------------------------------------------------------------------
;
;  Timer 1 Module
;
;  This module handles the timing for buttons and led's.  The timer will be set
;  to interrupt every 2000 ticks.  This provides an interrupt every millisecond.
;  This will be the LED update rate.  With 16 brightness levels, this is

;  Define variables for the timer 1 module.
	cblock
		mstimerl	;  Low byte of millisecond timer.
		mstimerh	;  High byte of millisceond timer.
        mstimertick ;  Bit 0 set if ms tick code should run.
	endc

;  The reload value is 65536-2000 = 0FB30H.  This will cause an interrupt on
;  overflow every 1ms.
timer_reload_low	equ		080H
timer_reload_high	equ		0c1H

;  Routine - timer_init : Set up the timer to give us an interrupt every 1ms.
;  The isr code for this must be VERY lightweight.

timer_init

    banksel T1CON
    bsf     T1CON,TMR1CS0    ; Select internal clock.  System Clock.
    bcf     T1CON,TMR1CS1
    bsf     T1CON,NOT_T1SYNC  ; Not required for internal clock.
    bcf     T1CON,T1CKPS0   ; Prescaler 1:1
    bcf     T1CON,T1CKPS1

    banksel T1GCON
    bcf     T1GCON,TMR1GE

 	clrf	mstimerl
	clrf	mstimerh

    movlw   timer_reload_high
    movwf   TMR1H
    movlw   timer_reload_low
    movwf   TMR1L

	bcf		PIR1,TMR1IF				;  Clear any pending interrupt.
	banksel PIE1
	bsf		PIE1,TMR1IE				;  Enable the interrupts
	banksel T1CON
	bsf		T1CON,TMR1ON			;  Start the timer.
	return


;  Routine - timer_isr : This is called from the interrupt routine if a timer1
;  interrupt is detected.

timer_isr

	banksel PIR1
	btfss	PIR1,TMR1IF				;  Check for timer 1 interrupt.
	goto	timer_isr_return		;  If not, return.

	bcf		T1CON,TMR1ON			;  Turn off the timer.

	incf	mstimerl,F				;  Increment mstimer.
	btfsc	STATUS,Z
	incf	mstimerh,F
	movlw	timer_reload_low		;  Get low timer value.
	movwf	TMR1L					;  Set low byte.
	movlw	timer_reload_high		;  Get high timer value.
	movwf	TMR1H					;  Set high byte.

	bcf		PIR1,TMR1IF				;  Clear any pending interrupt.
	bsf		T1CON,TMR1ON			;  Turn timer back on.

    bsf     mstimertick,0           ;  Set bit to run tick code.

;    call    tick_list               ;  Handle everything that needs a tick.

timer_isr_return
	return

;  Timer Module : END
;
; ------------------------------------------------------------------------------

; ------------------------------------------------------------------------------
;
;  LED Module
;
;  This module handles the interaction with the LEDs.
;
;  Definitions
;
;       LED R   Port C3     GND is On
;       LED G   Port C4
;       LED B   Port C5

LED_R_BIT   EQU 3
LED_G_BIT   EQU 4
LED_B_BIT   EQU 5

;  Define variables for the led module.
	cblock
		led_count			;  Counter for loading
		led_shift			;  Temp locations for shifting data.
		led_flash			;  Flash values.
        led_tmp             ;  Temp location for some math.
        led_value           ;  Place to hold the bits to set at the end.
        led_write_value     ;  Value to write.
        led_write_state     ;  State for writing the 8 bit value.
        led_write_delayl    ;  Delay used in write state machine.
        led_write_delayh
        led_write_next      ;  Used to set a next state.
        led_write_mask      ;  Bit mask to determine next bit to write.
        led_intensity:3     ;  RGB values.
	endc

;  Routine - led_init : Setup the processor for writing to the LED registers.
led_init

	;  Set up the bits to control the LEDs.
	banksel LATC
    bcf     LATC,LED_R_BIT
    bcf     LATC,LED_G_BIT
    bcf     LATC,LED_B_BIT
    banksel TRISC
	bcf		TRISC,LED_R_BIT
    bcf     TRISC,LED_G_BIT
    bcf     TRISC,LED_B_BIT

    ; Clear the led's.
    banksel led_intensity
    clrf    led_intensity+0;
    clrf    led_intensity+1;
    clrf    led_intensity+2;

	;  That's it.
	return

;  Routine - led_set_intensity : This routine checks the expected state of all the
;  leds and sets their next value into the 3 led locations.
;
;  The intensity is determined by the following:
;
;		led intensity byte : mpffiiii
;
;  When m = 0 then i is the intensity of the led from 0-15.  0 is off and 15 is almost
;					always on.
;  When m = 1 then the following applies:
;			f=0 : flash at 1 Hz, f=1 : flash at 2 Hz, f=2 flash at 4 Hz, f=3 8 Hz
;  When p = 0 then the led will flash in phase.
;  When p = 1 then the led will flash out of phase.
;  The phase bit lets an led be set up to flash red then green for instance.

led_tick

	banksel mstimerl					;  This all happens in bank 0.

	; The flash is done by overlaying bits 9-6 of the mstimer.  Two bits are
    ; in mstimerl and two in mstimerh.  Put all 4 bits into the low nibble of
    ; variable led_flash.  Bit 0 will be flashing at ~8Hz while bit 3 will be
    ; flashing at ~1Hz.  Where the bit is 1, it should always turn the light
    ; off.  When it is a 0, then leave the intensity setting.

	movfw	mstimerl
	movwf	led_shift
	movfw	mstimerh
	movwf	led_flash
    andlw   0x03
    bcf     STATUS,C
	rlf		led_shift,F
	rlf		led_flash,F
	rlf		led_shift,F
	rlf		led_flash,F     ;  led_flash now has the 4 flash bits.

    ; The variable led_value will the led BGR values in bits 2:0.  The bits are
    ; shifted in as each LED state is determined.  Start with Blue.  But clear
    ; the variable to start.

    clrf    led_value       ;  Used to hold the values of the bits.

    ; Loop through each of the led intensity values, so that each led can be set.

	movlw	3                   ; Loop counter for setting each led.
	movwf	led_count           ; Save it in count.
	movlw	led_intensity+3-1	; Set the indirect memory pointer.
	movwf	FSR0L
    clrf    FSR0H
;	bcf		STATUS,IRP		;  Led intensity bytes locations are not higher than 255.

    ; Here is the loop that will set the intensity of each of the three LEDs.
    ; The blue LED is done first, then green and last red.  The intensity is
    ; determined by the low 4 bits of the led's command byte.

led_set_intensity_loop

	movfw	mstimerl		; Get low byte of the millisecond timer.
	andlw	00f				; Clear it.
	movwf	led_tmp			; Save it.
	movfw	INDF0			; Get the intensity.
	andlw	00f				; Clear all other bits.
	subwf	led_tmp,W		; Subtract mstimer-intensity.

	; If carry is set, LED should be off.  Rotate the value into the value
    ; varaible.

    rlf     led_value       ; Move the value of this led into the holding location.

    ; Now check to see if this led should be flashing.  The mode value should be
    ; 1 in bits 5:4 if the led should be flashing.

	movfw	INDF0            ; Get the led command byte.
	andlw	0x80            ; Get mode bit.
    xorlw   0x80            ; Check to see if the mode is 'flash'.
    btfss   STATUS,Z        ; Skip if we should flash.
    goto    led_loop_check  ; We are not flashing so check for end of looping.

    ; We need to isolate the correct led_flash bit based on the flash bits in
    ; the led command byte.  Trickery here to do some shifts instead of looping
    ; on a count to get the bit out the end.

    movfw   led_flash       ; Get the flash values of which we will select one.
    movwf   led_shift       ; These will get shifted so make a copy.
    swapf   led_shift       ; Put the 4 bits in the high nibble for shifting left.
    movfw   INDF0            ; Get the led command byte.
    movwf   led_tmp         ; The led_tmp variable has the command byte.
    btfsc   led_tmp,4       ; See if the low flash select bit is set.
    rlf     led_shift,f     ; Shift the flash value left one position.
    btfss   led_tmp,5       ; See if the high flash select bit is set.
    goto    led_set_flash   ; If bit is clear, go set the led for flashing.
    rlf     led_shift,f     ; Shift two places.
    rlf     led_shift,f     ; Now important bit is in position 7.

led_set_flash

    movfw   INDF0
    rrf     led_shift,f
    xorwf   led_shift,f
    rlf     led_shift,f

    rlf     led_shift,f     ; Now the flash control bit is in the carry.
    btfsc   STATUS,C        ; If the bit is clear, we let the intensity alone.
    bsf     led_value,0     ; If the bit is set, we turn off the led to make it flash.

	;  Check the loop to see if we need to do the next led.

led_loop_check

	decf	FSR0L,F                   ; Point to the next led to check.
	decfsz	led_count,F             ; Loop through all 3 leds.
	goto	led_set_intensity_loop  ; Go back to do the next led.

    ; Write the led value bits to the output pins on port A.  The led I/O bits
    ; are two bits higher than the values that have been shifted into the
    ; led_value variable.

    bcf     STATUS,C
    rlf     led_value
    rlf     led_value
    rlf     led_value
    movfw   led_value
    banksel LATC
    movwf   LATC

    ; We are done handling the led intensities. Return to the caller.

led_tick_return

	return                          ; Return to the caller.

;  Method - led_write : Starts the writing of a value to the led.  Value
;  to write should be in the W register.

led_write

    banksel led_write_state
    movwf   led_write_value
    clrf    led_write_state
    bsf     led_write_state,LED_WRITE_STATE_START
    return

led_write_off
    banksel led_write_state
    clrf    led_write_state
    return

;  Method - led_write_tick : Strobes the led such that an 8-bit value can be
;  read.
;
;   led_value   Holds the 8 bit value to write to the led.
;   led_state   Holds the state needed to write the value.
;   led_delay   Holds a delay for the current led setting.
;
;  State definitions:
;

LED_WRITE_STATE_START   EQU     0       ; Starts sequence.  Next state clock
LED_WRITE_STATE_DELAY   EQU     1       ; Delays specified ms then to next state.
LED_WRITE_STATE_DELAY2  EQU     2       ; A second delay state which has led's off.
LED_WRITE_STATE_BIT     EQU     3       ; Write the next bit in the value.
LED_WRITE_STATE_END     EQU     4       ; Check for end of bits to write.

led_write_tick

    banksel led_write_state
    movf    led_write_state
    btfsc   STATUS,Z
    goto    led_write_tick_return      ; Not writing value so return

    ;  Find the state that we need to use.
    btfsc   led_write_state,LED_WRITE_STATE_START
    goto    led_write_state_start
    btfsc   led_write_state,LED_WRITE_STATE_DELAY
    goto    led_write_state_delay
    btfsc   led_write_state,LED_WRITE_STATE_DELAY2
    goto    led_write_state_delay2
    btfsc   led_write_state,LED_WRITE_STATE_BIT
    goto    led_write_state_bit
    btfsc   led_write_state,LED_WRITE_STATE_END
    goto    led_write_state_end

    clrf    led_write_state
    goto    led_write_tick_return

    ;  Start state turns on the blue led and sets the delay for 250ms.

led_write_state_start

    movlw   0x03
    movwf   led_write_delayh
    clrf    led_write_delayl
    movlw   0x80
    movwf   led_write_mask
    clrf    led_write_next
    bsf     led_write_next,LED_WRITE_STATE_BIT
    clrf    led_write_state
    bsf     led_write_state,LED_WRITE_STATE_DELAY
    clrf    led_intensity
    clrf    led_intensity+1
    movlw   0x0f
    movwf   led_intensity+2
    goto    led_write_tick_return

    ;  Delay state simply waits the specified ms then goes to next state.

led_write_state_delay

    movlw   1
    subwf   led_write_delayl
    movlw   0
    subwfb  led_write_delayh
    btfsc   STATUS,C
    goto    led_write_tick_return

    movlw   .150
    movwf   led_write_delayl
    clrf    led_write_state
    bsf     led_write_state,LED_WRITE_STATE_DELAY2
    clrf    led_intensity
    clrf    led_intensity+1
    clrf    led_intensity+2
    goto    led_write_tick_return

    ; Delay with led off.

led_write_state_delay2

    decfsz  led_write_delayl
    goto    led_write_tick_return

    movfw   led_write_next
    movwf   led_write_state
    goto    led_write_tick_return

    ;  Write the next bit.

led_write_state_bit

    clrf    led_intensity
    clrf    led_intensity+1
    clrf    led_intensity+2
    movfw   led_write_value
    andwf   led_write_mask,W
    btfss   STATUS,Z
    goto    led_write_state_bit_on

led_write_state_bit_off
    movlw   0x02
    movwf   led_intensity
    goto    led_write_state_bit_next

led_write_state_bit_on
    movlw   0x0f
    movwf   led_intensity
    goto    led_write_state_bit_next

led_write_state_bit_next
    movlw   0x03
    movwf   led_write_delayh
    clrf    led_write_delayl
    clrf    led_write_next
    bsf     led_write_next,LED_WRITE_STATE_END
    clrf    led_write_state
    bsf     led_write_state,LED_WRITE_STATE_DELAY
    goto    led_write_tick_return

led_write_state_end
    bcf     STATUS,C
    rrf     led_write_mask
    btfss   STATUS,C
    goto    led_write_state_end_next

    clrf    led_write_state
    bsf   led_write_state,LED_WRITE_STATE_START
    goto    led_write_tick_return

led_write_state_end_next
    clrf    led_write_state
    bsf     led_write_state,LED_WRITE_STATE_BIT
    goto    led_write_tick_return

led_write_tick_return

    return              ; Return to the caller.


;  LED Module : END
;
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
;
;  Start up the main program.  Call the initialization routines for each of the
;  modules.

main

   ; Set the clock to 16Mhz.
    banksel OSCCON
    movlw   0x7a

    banksel OPTION_REG
    BCF OPTION_REG, TMR0CS
    bcf     OPTION_REG,NOT_WPUEN

    banksel ANSELC
    clrf ANSELC

    banksel ANSELA
    clrf    ANSELA

    call    system_init     ;  Initialize system.
    call    timer_init      ;  Initialize timer.
    call    encoder_init    ;  Initialize encoder.
    call    led_init        ;  Initialize led.
    call    button_init     ;  Initialize button.
    call    i2c_init        ;  Initialize serial bus.

 	;  Enable interrupts.
    banksel INTCON
    bsf     INTCON,PEIE     ; Enable peripheral interrupts.
    bsf     INTCON,GIE      ; Enable global interrupts.

    ;  Test led write.
    movlw  0xac
    call    led_write

;    call    led_write_off
    clrf    led_intensity
    clrf    led_intensity+1
    clrf    led_intensity+2

    goto    main_loop


    ;  Run the ms tick code.

main_loop

    banksel mstimertick
    btfss   mstimertick,0
    goto    main_code

    call    encoder_tick
    call    led_tick
    call    button_tick
    call    led_write_tick

    banksel mstimertick
    bcf     mstimertick,0

main_code
    goto    main_loop

;; for master testing, every time that a button is pressed, send a byte to
;; the slave.
;    ifdef   master
;    cblock
;        button
;        time
;        led1
;        led2
;        led3
;    endc
;main_code
;
;    banksel time
;    movfw   time
;    btfsc   STATUS,Z
;    goto    check_button
;
;    ; See if we are done sending the packet.
;    btfss   i2c_state,I2C_STATE_READY
;    goto    check_button
;
;    ; we are done with the packet.  Display the time.
;    banksel TMR1H
;    subwf   TMR1H,W
;    call    led_write
;    banksel time
;    clrf    time
;
;check_button
;    banksel button
;    btfss   button_state,BUTTON_STATE_PRESSED
;    goto    main_not_pressed
;
;    movfw   button
;    btfss   STATUS,Z
;    goto    main_loop
;
;    bsf     button,0
;    goto    send_packet
;
;main_not_pressed
;    clrf    button
;    goto    main_loop
;
;send_packet
;
;    ;  Test i2c reserve
;    call    i2c_queue_reserve
;    btfss   STATUS,C
;    goto    q_res_ok
;
;q_res_not_ok
;    movlw   0x90
;    call    led_write
;    goto    main_loop
;
;q_res_ok
;    movlw   0x40
;    call    led_write
;
;    banksel led1
;    ;  shift led's
;    movfw   led2
;    movwf   led3
;    movfw   led1
;    movwf   led2
;    movfw   mstimerl
;    movwf   led1
;
;
;    ; Write the address and a data value.
;    banksel i2c_queue
;    movlw   0x20
;    call    i2c_queue_write     ; Address
;    movlw   0x13                ; Type 1 (Set LEDs), Length 3
;    call    i2c_queue_write     ; Packet Type / Length
;    movlw   0x00                ; Red @ 8
;    movfw   led1
;    call    i2c_queue_write
;    movlw   0x00
;    movfw   led2              ; Green @ 8
;    call    i2c_queue_write
;    movlw   0x00                ; Blue @ 0
;    movfw   led3
;    call    i2c_queue_write
;
;    movlw   0x41
;    call    led_write
;
;    ; Release the queue.
;    movlw   0x42
;    call    led_write
;    banksel TMR1H
;    movfw   TMR1H
;    banksel time
;    movwf   time
;    call    i2c_queue_release
;    goto    main_loop
;
;    endif
;
;
;;    goto    test_leds
;;    goto    test_button
;;    goto    test_encoder
;    goto     test_leds
;    goto    test_i2c_master
;
;; Set the leds to some test values.
;test_leds
;    banksel led_intensity
;    movlw   0x9f
;    movwf   led_intensity
;    movlw   0xd8
;    movwf   led_intensity+1
;    movlw   0x00
;    movwf   led_intensity+2
;    goto    main_loop
;
;; Set the leds to some test values.
;test_led2
;    banksel led_intensity
;    movlw   0x9f
;    movwf   led_intensity
;    movlw   0x00
;    movwf   led_intensity+1
;    movlw   0xd8
;    movwf   led_intensity+2
;    goto    loop
;
;loop
;    goto    loop
;
;
;; Test the button.
;test_button
;
;    btfss   button_state,BUTTON_STATE_PRESSED
;    goto    button_not_pushed
;
;    movlw   0x0f
;    movwf   led_intensity
;    goto    test_button
;
;button_not_pushed
;
;    movlw   0x00
;    movwf   led_intensity
;    goto    test_button
;
;; Test encoder
;test_encoder
;    banksel encoder_position
;    movfw   encoder_position
;    movwf   led_intensity
;    goto    test_encoder
;
;test_i2c_master
;    banksel SSP1CON2
;    bsf     SSP1CON2,SEN
;    banksel led_intensity
;    movlw   0x02
;    movwf   led_intensity
;
;    call    wait2
;
;    banksel led_intensity
;    movlw   0x88
;    movwf   led_intensity
;
;    call    write
;    call    wait2
;    call    write
;    call    wait2
;
;    call    stop
;    call    wait2
;
;;    banksel SSP1CON1
; ;   bcf     SSP1CON1,SSPEN
;
;    goto    loop
;
;stop
;    banksel SSP1CON2
;    bsf     SSP1CON2,PEN
;    banksel led_intensity
;    clrf    led_intensity
;    clrf    led_intensity+2
;    movlw   0x88
;    movwf   led_intensity+1
;
;    return
;
;write
;    banksel SSP1BUF
;    movlw   0xD2
;    movwf   SSP1BUF
;    banksel led_intensity
;    clrf    led_intensity
;    clrf    led_intensity+2
;    movlw   0x08
;    movwf   led_intensity+1
;
;    return
;
;
;; wait
;
;wait2
;    banksel SSP1CON2
;    btfsc   SSP1CON2,SEN
;    goto    wait2
;    banksel led_intensity
;    clrf    led_intensity
;
;    movlw   0x02
;    movwf   led_intensity+2
;
;    banksel PIR1
;    bcf     PIR1,SSP1IF
;
;    return
;
;
;; Routine - tick_list : This is a list of all the routines that need to be
;; called on 1ms boundaries.
;
;tick_list
;
;    call    encoder_tick
;    call    led_tick
;    call    button_tick
;    return

; End of main program.
;
; ------------------------------------------------------------------------------

	end







