#include <xc.h>
#include <pic.h>
#include "MultiChemCharger_Main.h"
#include "MultiChemCharger_Hardware.h"
#include "MultiChemCharger_Serial.h"

// <editor-fold defaultstate="collapsed" desc="Example Status LED code for reference when adding new platforms">
/*
void update_status_leds(void)
{
	cd.user_interface_mode = 2;				// Set user interface mode (for GUI) //%%BRYAN%% run down the usefulness of this when you have time.

	// LED #1 - Charger Active
	if(cd.charger_state != CHARGER_OFF)		// If the charger is active,
		LED1_PIN = 1;						// Charger Active LED #1 on
	else
		LED1_PIN = 0;						// Charger Active LED #1 off

	// LED #2 - Charger State (CC/CV)
	if(cd.charger_state == CHARGER_LIION_CC ||	// If the charge state = CC
		cd.charger_state == CHARGER_NIMH_RAPID ||
		cd.charger_state == CHARGER_VRLA_CC ||
		cd.charger_state == CHARGER_VRLAF_RAPID ||
		cd.charger_state == CHARGER_LIFEPO4_CC)
		LED2_PIN = 1;						// Charger CC/CV State LED #2 on
	else
		LED2_PIN = 0;						// Charger CC/CV State LED #2 off

	// LED #3 - Charge Complete
	if(cd.status.complete == 1)
		LED3_PIN = 1;						// Charge Complete LED #3 on
	else
		LED3_PIN = 0;						// Charge Complete LED #3 off

	// LED #4 - Fault LED
	if((cd.status.word & 0x7fff) != 0)      // These shutdown causes are not defined yet
		LED4_PIN = 0;						// Fault LED #4 off

	else									// All other shutdown_causes are true hard-faults
		LED4_PIN = 1;						// Fault LED #4 on

} */
//</editor-fold>

// <editor-fold defaultstate="collapsed" desc="MCP19111_CHARGER_TOPOLOGY">
#ifdef MCP19111_CHARGER_TOPOLOGY

char vh, vl;
extern struct charger_settings_t cs;	// Get access to Charger_Settings structure
extern struct charger_data_t cd;		// Get access to Charger_Data structure

void interrupt isr()
{
    if (PIR2bits.VINIF == 1) 	// UVLO flag set?
	{
        /* Disable the interrupt */
        PIE2bits.VINIE = 0;

        /* PWM off */
        ATSTCONbits.DRVDIS = 1;

        /* Output FET off */
        PORTBbits.GPB7 = 0;
		
		// log the reason 
		cd.status.vinuvlo = 1;
    }
}

 
void init_hardware()	/* MCP19111 Hardware specific configuration. */
{
    OSCTUNE = 0x00;                     // Oscillator Tuning Register. Set oscillator to Calibrated Center Frequency
    // <editor-fold defaultstate="collapsed" desc="OSCTUNE Register bit Description">
                                        //bit 7-5 Unimplemented
                                        //bit 4-0 TUN<4:0> Frequency Tuning Bits
                                        //b01111 = Maximum Frequency.
                                        //b01110 thru b00001 = monotonic ramp in between maximum and center.
                                        //b00000 = Center frequency.  Oscillator Module is running at the calibrated frequency.
                                        //b11111 thru b10001 = monotonic ramp in between center and minimum.
                                        //b10000 = Minimum Frequency
    // </editor-fold>
    OPTION_REG = 0x00;                  // Prescaler (bits 2:0) set to 000 for 1:2 TMR0 Rate, 256usec per timer overflow.  
    // <editor-fold defaultstate="collapsed" desc="OPTION_REG Register Description">
                                        // bit 7 RAPU# Port GPx Pull_up Enable bit, 1=Disabled, 0=Enabled
                                        // bit 6 INTEDG Interrupt Edge Select bit, 0=Interrupt on rising edge of INT pin, 1=falling edge
                                        // bit 5 T0CS TMR0 Clock Source Select bit, 1=Transition on T0CKI pin, 0=Internal instruction cycle clock
                                        // bit 4 T0SE TMR0 Source Edge Select bit, 1=Increment on high to low transition on T0CKI pin, 0=low to high
                                        // bit 3 PSA Prescaler Assignment bit, 1=Prescalar is assigned to WDT, 0=Prescaler is assigned to Timer0 module
                                        // bit 2-0 PS<2:0> Prescalar Rate Select Bits:
                                        //  000 - 1:2 TMR0 Rate 1:1 WDT Rate
                                        //  001 - 1:4 TMR0 Rate 1:2 WDT Rate
                                        //  010 - 1:8 TMR0 Rate 1:4 WDT Rate
                                        //  011 - 1:16 TMR0 Rate 1:8 WDT Rate
                                        //  100 - 1:32 TMR0 Rate 1:16 WDT Rate
                                        //  101 - 1:64 TMR0 Rate 1:32 WDT Rate
                                        //  110 - 1:128 TMR0 Rate 1:64 WDT Rate
                                        //  111 - 1:256 TMR0 Rate 1:128 WDT Rate
    // </editor-fold>
    PIR1    = 0;                        // Peripheral Interrupt Flag Register 1. All Interrupts Cleared.
    // <editor-fold defaultstate="collapsed" desc="PIR1 Register Bit Description">
                                        // bit 7 Unimplemented, Read as 0
                                        // bit 6 ADIF ADC Interrupt Flag bit, 1=ADC Conversion Complete, 0=Conversion not complete or not started
                                        // bit 5 BCLIF MSSP Bus Collision Interrupt Flag bit, 1=Interrupt is pending, 0=Interrupt is not pending
                                        // bit 4 SSPIF Synchronous Serial Port (MSSP) Interrupt Flag bit, 1=Interrupt is pending, 0=Interrupt not pending.
                                        // bit 3-2 Unimplemented, Read as 0
                                        // bit 1 TMR2IF Timer2 to PR2 Match Interrupt Flag, 1=Timer2 to PR2 match occurred (must be cleared in SW), 0=Match did not occur
                                        // bit 0 TMR1IF Timer1 Interrupt Flag.  1=Timer1 rolled over (must be cleared in SW). 0=Timer 1 has not rolled over.
    // </editor-fold>
    PIE1    = 0;                        // Peripheral Interrupt Enable Register 1. All Interrupts Disabled.
    // <editor-fold defaultstate="collapsed" desc="PIE1 Register Bit Description">
                                        // bit 7 Unimplemented. Read as 0.
                                        // bit 6 ADC Interrupt Enable bit, 1=Enables ADC Interrupt, 0=Disables
                                        // bit 5 BCLIE MSSP Bus Collision Interrupt Enable bit, 1=Enables the MSSP bus Collision Interrupt, 0=Disables
                                        // bit 4 SSPIE synchronous Serial Port (MSSP) Interrupt Enable bit, 1=Enables MSSP Bus Collision Interrupt, 0=Disables
                                        // bit 3-2 Unimplemented
                                        // bit 1 TMR2IE Timer2 Interrupt Enable, 1=Enables the Timer2 Interrupt, 0=Disables
                                        // bit 0 TMR1IE Timer1 Interrupt Enable, 1=Enables the Timer1 Interrupt, 0=Disables
    // </editor-fold>
    PIR2    = 0;                        // Peripheral Interrupt Flag Register 2.  All interrupts Cleared.
    // <editor-fold defaultstate="collapsed" desc="PIR2 Register Bit Description">
                                        // bit 7 UVIF Output undervoltage error interrupt flag bit, 1=Output Undervoltage error has occurred, 0=Has not occured
                                        // bit 6 Unimplemented, Read as 0.
                                        // bit 5 OCIF Output overcurrent error interrupt flag bit, 1=Output OC error has occurred, 0=Has not occured
                                        // bit 4 OVIF Output overvoltage error interrupt flag bit, 1=Output OV error has occurred, 0=Has not occured
                                        // bit 3-2 Unimplemented, Read as 0
                                        // bit 1 VINIF Vin Status bit, 1=Vin is below acceptable level, 0=Vin is at acceptable level
                                        // bit 0 Unimplemented, Read as 0
    // </editor-fold>
    PIE2    = 0;                        // Peripheral Interrupt Enable Register 2. All Interrupts Disabled.
    // <editor-fold defaultstate="collapsed" desc="PIE2 Register Bit Description">
                                        // bit 7 UVIE Output Under voltage Interrupt enable bit, 1=Enables teh UV interrupt, 0=Disables
                                        // bit 6 Unimplemented, Read as 0
                                        // bit 5 OCIE Output Overcurrent Interrupt enable bit, 1=Enables the OC Interrupt, 0=Disables
                                        // bit 4 OVIE Output Overvoltage Interrupt enable bit, 1=Enables the OV Interrupt, 0=Disables
                                        // bit 3-2 Unimplemented, Read as 0
                                        // bit 1 VINIE V_in UVLO Interrupt Enable, 1=Enables teh V_IN UVLO Interrupt, 0=Disables
                                        // bit 0 Unimplemented, Read as 0
    // </editor-fold>
    INTCON  = 0xc0;                     // Global + Peripheral interrupts.  Enable all Unmasked Interrupts and Unmasked Peripheral Interrupts.  All the rest are disabled.
    // <editor-fold defaultstate="collapsed" desc="INTCON Register Bit Description">
                                        // bit 7 GIE Global Interrupt Enable bit, 1=Enables all unmasked interrupts, 0=Disables
                                        // bit 6 PEIE Peripheral Interrupt Enable bit, 1=Enables all unmasked peripheral interrupts, 0=Disables
                                        // bit 5 TMR0 Overflow Interrupt Enable bit, 1=Enables the TMR0 Interrupt, 0=Disables
                                        // bit 4 INTE The INT External Interrupt Enable bit, 1=Enables the INT external interrupt. 0=Disables
                                        // bit 3 IOCE Interrupt on Change Enable bit, 1=Enables the interrupt-on-change interrupt, 0=Disables
                                        // bit 2 T0IF TMR0 Overflow Interrupt Flag big, 1=TMR0 register has overflowed (must be cleared in software), 0=TMR0 did not overflow
                                        // bit 1 INTF External Interrupt Flag bit. 1=External Interrupt Occurred (must be cleared in software), 0=External Interrupt did not occur
                                        // bit 0 IOCF Interrupt on Change Interrupt Flag bit, 1=When at least one of the IOC pins changed state, 0=None have changed state
    // </editor-fold>
    IOCA = 0x00;                        // Interrupt on Change PortGPA Register.  Disabled on all Port A pins.  Setting a bit also requires bit 3 of INTCON be set to enable interrupt on change.
    // <editor-fold defaultstate="collapsed" desc="IOCA Register Bit Description">
                                        // bit 7-6 Interrupt on Change PortGPA Register bits, 1=enabled, 0=disabled
                                        // bit 5 Interupt on Change PortGPA Register bits, 1=enabled, 0=disabled
                                        // bit 4-0 Interrupt on Change PortGPA Register bits, 1=enabled, 0=disabled
    // </editor-fold>
    IOCB = 0x00;                        // Interrupt on Change PortGPB Register.  Disabled on all Port B pins.  Setting a bit also requires bit 3 of INTCON be set to enable interrupt on change.
    // <editor-fold defaultstate="collapsed" desc="IOCB Register Bit Description">
                                        // bit 7-4 and 2-0 Interrupt on Change PortGPA Register bits, 1=enabled, 0=disabled
                                        // bit 3 Unimplemented, Read as 0
    // </editor-fold>
    ADCON0  = ADC_MUX_VIN | 0x01;       // ADC Control Register 0.  Enables the ADC and sets the default input channel to VIN_ANA (00000)
    // <editor-fold defaultstate="collapsed" desc="ADCON0 Register Bit Description">
                                        // bit 7 Unimplemented, read as 0
                                        // bit 6-2 CHS<4:0> Analog Channel Selected bits
                                        // 00000 = VIN_ANA analog voltage proportional to Vin
                                        // 00001 = VREGREF reference voltage for V_REG output
                                        // 00010 = OV_REF feference for overvoltage comparator
                                        // 00011 = UV_REF reference for undervoltage comparator
                                        // 00100 = VBGR band gap reference
                                        // 00101 = INT_VREG internal versin of the V_REG load voltage
                                        // 00110 = CRT voltage proportional to the current in the indutor
                                        // 00111 = VZC an internal ground, voltage for zero current
                                        // 01000 = DEMAND input to current loop, output of demand mux
                                        // 01001 = RELEFF analog voltage proportional to duty cycle
                                        // 01010 = TMP_ANA analog voltage proportional to temperature
                                        // 01011 = ANA_IN demanded current from the remote master
                                        // 01100 = DCI DC Inductor valley current
                                        // 01101 = Unimplemented
                                        // 01110 = Unimplemented
                                        // 01111 = Unimplemented
                                        // 10000 = GPA0 i.e. ADDR1
                                        // 10001 = GPA1 i.e. ADDR0
                                        // 10010 = GPA2 i.e. Temperature Sensor Input
                                        // 10011 = GPA3 i.e. Tracking Voltage
                                        // 10100 = GPB1
                                        // 10101 = GPB2
                                        // 10110 = GPB4
                                        // 10111 = GPB5
                                        // 11000 = Unimplemented
                                        // 11001 = Unimplemented
                                        // 11010 = Unimplemented
                                        // 11011 = Unimplemented
                                        // 11100 = Unimplemented
                                        // 11101 = Unimplemented
                                        // 11110 = Unimplemented
                                        // 11111 = Unimplemented
    // </editor-fold>
    ADCON1  = 0x20;                     // ADC Control Register 1.  Enable F_OSC / 32 which sets 4usec per conversion.
    // <editor-fold defaultstate="collapsed" desc="ADCON1 Register Bit Description">
                                        // bit 7 Unimplemented, Read as 0
                                        // bit 6-4 ADCS<2:0> ADC Clock Select Bits
                                        // 000 = Reserved
                                        // 001 = F_OSC/8
                                        // 010 = F_OSC/32
                                        // x11 = F_RC (clock derived from internal oscillator with a divisor of 16)
                                        // 100 = Reserved
                                        // 101 = F_OSC/16
                                        // 111 = F_OSC/64
                                        // bit 3-0 Unimplemented, Read as 0
    // </editor-fold>
	PORTGPA = 0b00000000;               // Clear port A to start
    // <editor-fold defaultstate="collapsed" desc="PORTGPA Register Bit Description">
                                        // bit 7 GPA7 General Purpose Open Drain I/O Pin
                                        // bit 6 GPA6 General Purpose I/O Pin, 1=Port pin is > VIH, 0=Port Pin is < VIL
                                        // bit 5 GPA5/MCLR# General Purpose Open Drain I/O pin
                                        // bit 4 GPA7 General Purpose Open Drain I/O pin
                                        // bit 3-0 GPA<3:0> General Purpose I/O Pin, 1=Port Pin is > VIH, 0=Port Pin is < VIL
    // </editor-fold>

#ifdef ENABLE_BENCH_TEST_OUTPUT_PIN
	ATSTCON = 0b10000011;               // Analog Bench Test Control Register, Configure output MOSFET drivers (safe power-up, so do this first thing), GPA0 set for Benchtest.
    // <editor-fold defaultstate="collapsed" desc="ATSTCON Register Bit Description">
                                        // bit 7 reserved
                                        // bit 6-5 Unimplemented and Read as 0
                                        // bit 4 Reserved
                                        // bit 3 HIDIS: High Side Driver Control Bit, 1=Disabled, 0=Enabled
                                        // bit 2 LODIS: Low Side Driver Control Bit, 1=Disabled, 0=Enabled
                                        // bit 1 BNCHEN: GPA0 bench test configuration control bit, 1=GPA0 is configured for analog bench test output, 0=GPA0 Normal Operation
                                        // bit 0 DRVDIS: MOSFET Driver Disable Control Bit, 1=High and Low Side Drivers set Low, PHASE floating, 0=Normal Operation
    // </editor-fold>
    TRISGPA = 0b11111000;               // Port A Tristate Register. A7,A6,A5,A4,A3 as Input. LED outputs on A1,A2 active high. Benchtest requires A0 to be an output.
    // <editor-fold defaultstate="collapsed" desc="TRISGPA Register Bit Description">
                                        // bit 7-6 TRISA<7:6> PORTGPA Tri-State Control bit, 1=input (tri-stated), 0=output
                                        // bit 5 TRISA5 GPA5 Tri-State Control bit. This bit is always 1 as GPA5 is an input only
                                        // bit 4-0 TRISA<4:0> PORTGPA Tri-State Control Bit, 1=input (tri-state), 0=output
    // </editor-fold>
	ANSELA  = 0b00001001;               // Analog Select Port A Register.  A0 and A3 are Analog inputs.  Benchtest requires GPA0 be configured for analog.
    // <editor-fold defaultstate="collapsed" desc="ANSELA Register Bit Description">
                                        // bit 7-4 Unimplemented, Read as 0
                                        // bit 3-0 Digital I/O. Pin is assigned to port or special function.
    // </editor-fold>
    WPUGPA  = 0b00100000;               // Port A Weak Pull-Up Register for digital inputs (A7, A6, A4 are open-drain). A5 (MCLR#) has a weak pullup.
    // <editor-fold defaultstate="collapsed" desc="WPUGPA Register Bit Description">
                                        // bit 7-6 Unimplemented, Read as 0
                                        // bit 5 WPUA5 Weak Pull-Up Register bit, 1=Pull-up enabled, 0=Disabled
                                        // bit 4 Unimplemented, Read as 0
                                        // bit 3-0 WPUA<3:0> Weak Pull-up Register Bit, 1=Pull-up enabled, 0=Disabled
                                        // NOTE: The weak pull-up device is enabled only when the global RAPU# bit is enabled, the pin is in input mode (TRISGPA=1),
                                        //       the individual WPUA bit is enabled (WPUA=1), and the pin is not configured as an analog input.
    // </editor-fold>

#else
	ATSTCON = 0b10000001;               // Analog Bench Test Control Register, Configure output MOSFET drivers (safe power-up, so do this first thing), GPA0 set for Benchtest.
    // <editor-fold defaultstate="collapsed" desc="ATSTCON Register Bit Description">
                                        // bit 7 reserved
                                        // bit 6-5 Unimplemented and Read as 0
                                        // bit 4 Reserved
                                        // bit 3 HIDIS: High Side Driver Control Bit, 1=Disabled, 0=Enabled
                                        // bit 2 LODIS: Low Side Driver Control Bit, 1=Disabled, 0=Enabled
                                        // bit 1 BNCHEN: GPA0 bench test configuration control bit, 1=GPA0 is configured for analog bench test output, 0=GPA0 Normal Operation
                                        // bit 0 DRVDIS: MOSFET Driver Disable Control Bit, 1=High and Low Side Drivers set Low, PHASE floating, 0=Normal Operation
    // </editor-fold>
    TRISGPA = 0b11111000;               // Port A Tristate Register.  A7, A6, A5, A4, A3 set as Input.  LED outputs are on GPA0/1/2 active high
    // <editor-fold defaultstate="collapsed" desc="TRISGPA Register Bit Description">
                                        // bit 7-6 TRISA<7:6> PORTGPA Tri-State Control bit, 1=input (tri-stated), 0=output
                                        // bit 5 TRISA5 GPA5 Tri-State Control bit. This bit is always 1 as GPA5 is an input only
                                        // bit 4-0 TRISA<4:0> PORTGPA Tri-State Control Bit, 1=input (tri-state), 0=output
    // </editor-fold>
	ANSELA  = 0b00001000;               // Port B Analog Select Register.  A3 is an analog input. Benchtest pin disabled for LED user interfaces
    // <editor-fold defaultstate="collapsed" desc="ANSELA Register Bit Description">
                                        // bit 7-4 Unimplemented, Read as 0
                                        // bit 3-0 Digital I/O. Pin is assigned to port or special function.
    // </editor-fold>
    WPUGPA  = 0b00100000;               // Port A Weak Pull-Up Register for digital inputs (A7, A6, A4 are open-drain). A5 (MCLR#) has a weak pullup.
    // <editor-fold defaultstate="collapsed" desc="WPUGPA Register Bit Description">
                                        // bit 7-6 Unimplemented, Read as 0
                                        // bit 5 WPUA5 Weak Pull-Up Register bit, 1=Pull-up enabled, 0=Disabled
                                        // bit 4 Unimplemented, Read as 0
                                        // bit 3-0 WPUA<3:0> Weak Pull-up Register Bit, 1=Pull-up enabled, 0=Disabled
                                        // NOTE: The weak pull-up device is enabled only when the global RAPU# bit is enabled, the pin is in input mode (TRISGPA=1),
                                        //       the individual WPUA bit is enabled (WPUA=1), and the pin is not configured as an analog input.
    // </editor-fold>
#endif

    PORTGPB = 0b00000000;               // Clear port B to start (GPB3 not present as a pin on this part)
    // <editor-fold defaultstate="collapsed" desc="PORTGPB Register Bit Description">
                                        // bit 7-4 GPB<7:4> General Purpose I/O Pin bit, 1=Port pin is >VIH, 0=Port Pin is <VIL
                                        // bit 3 Unimplemented, Read as 0
                                        // bit 2-0 GPB<2:0> General Purpose I/O Pin bit, 1=Port pin is >VIH, 0=Port Pin is <VIL
    // </editor-fold>
    TRISGPB = 0b01111011;               // Port B Tristate Register.  B6, B5, B4, B3, B1, B0 set as Input. B7 output for Pack Disconnect. LED output is GPAB2 active high.
    // <editor-fold defaultstate="collapsed" desc="TRISGPB Register Bit Description">
                                        // bit 7-4 TRISB<7:4> PORTGPB Tri-State Control Bit, 1=Pin configured as input (tri-stated), 0=pin configured as output
                                        // bit 3 Unimplemented, Read as 1.
                                        // bit 2-0 TRISB<2:0> PORTGPB Tri-State Control Bit, 1=Pin configured as input (tri-stated), 0=pin configured as output
    // </editor-fold>
    ANSELB  = 0b00000010;               // Port B Analog Select Register. GPB1 is an analog input for measuring VBAT amplifier circuit.
    // <editor-fold defaultstate="collapsed" desc="ANSELB Register Bit Description">
                                        // bit 7-6 Unimplemented, Read as 0.
                                        // bit 5-4 ANSB<5:4> Analog Select PORTGPB Register Bit, 1=Analog input, 0=Digital I/O or Special Function
                                        // bit 3 Unimplemented, Read as 0.
                                        // bit 2-1 ANSB<2:1> Analog Select PORTGPB Register bit, 1=Analog input, 0=Digital I/O or Special Function
                                        // bit 0 Unimplemented, Read as 0.
    // </editor-fold>
    WPUGPB  = 0b01111100;               // Port B Weak Pull-up Register for digital inputs (B0 is an open-drain, B3 not on this part). B2,B3,B4,B5,B6 have pullups.
    // <editor-fold defaultstate="collapsed" desc="WPUGPB Register Bit Description">
                                        // bit 7-4 WPUB<7:4> Weak Pull-Up Register bit
                                        // bit 3 Unimplemented, Read as 0
                                        // bit 2-1 WPUB<2:1> Weak Pull-Up Register bit
                                        // bit 0 Unimplemented, Read as 0.
                                        // NOTE: The weak pull-up device is enabled only when the global RAPU# bit is enabled, the pin is in input mode (TRISGPB=1),
                                        //       the individual WPUB bit is enabled (WPUB=1), and the pin is not configured as an analog input.
    // </editor-fold>

/* Set up the PWM Generator with Timer2. Frequency and Maximum Duty Cycle. */
    
    T2CON   = 0x00;                     // Timer2 Control Register. Timer OFF, Prescalar at 1
    // <editor-fold defaultstate="collapsed" desc="T2CON Register Bit Description">
                                        // bit 7-3 Unimplemented, Read as 0
                                        // bit 2 TMR2ON Timer2 On bit, 1=On, 0=Off
                                        // bit 1-0 T2CKPS<1:0> Timer2 Clock Prescale Select bit
                                        // 00 = Prescaler is 1
                                        // 01 = Prescaler is 4
                                        // 10 = Prescaler is 8
                                        // 11 = Prescaler is 16
    // </editor-fold>
    TMR2    = 0x00;                     // Timer2 Time Base set to 0x00.  This is the target duty cycle.
    // <editor-fold defaultstate="collapsed" desc="TMR2 Register Bit Description">
                                        // bit 7-0 Holding Register for the 9-bit TMR2 Time Base.  Basically the target duty cycle (D = TMR2/PR2)
    // </editor-fold>
    APFCON  = 0x00;                     // Alternate Pin Function Control Register. Leave at Default 0x00.
    // <editor-fold defaultstate="collapsed" desc="APFCON Register Bit Description">
                                        // bit 7-1 Unimplemented, Read as 0
                                        // bit 0 CLKSEL Pin Selection Bit. 1=Multi-phase or multiple output clock function is on GPB5, 0=On GPA1.
    // </editor-fold>
    PWMPHL  = 0x00;                     // SLAVE Phase shift. Not used and set to 0.
    // <editor-fold defaultstate="collapsed" desc="PWMPHL Register Bit Description">
                                        // bit 7-0 Phase shift added to the SLAVE system clock received on GPA1 in a MASTER/SLAVE or multi-phase implementation. 
                                        // Not used in Single Phase.  If APFCON bit 0 is set then the clock is received on GPB5 instead.
    // </editor-fold>
    PWMRL   = 0x19;                     // Sets the maximum PWM duty cycle.  Set to 25.  So (25*296kHz)/8MHz = 93.75%
    // <editor-fold defaultstate="collapsed" desc="PWMRL Register Bit Description">
                                        // bit 7-0 PWM Register Low Byte.  Equation is PWM_Duty_Cycle = PWMRL * T_OSC * TMR2_Prescale_Value
                                        // PWMRL can be written to at any time, but it will not be latched into PWMRH until after a match between PR2 and TMR2 occurs.
    // </editor-fold>
    PR2     = 0x1B;                     // Timer2 Period Register.  Set to 300 kHz frequency: 8MHz/300kHz = 26.7. So Set 27 for 296kHz.
    // <editor-fold defaultstate="collapsed" desc="PR2 Register Bit Description">
                                        // bit 7-0 Timer2 Module Period Register
                                        // This is the PWM period.  The equation is PWM_Period = [(PR2)+1] * T_OSC * T2_Prescale_Value
    // </editor-fold>
    T2CON   = 0x04;                     // Timer 2 Control Register. Timer ON. Prescalar at 1.
    // <editor-fold defaultstate="collapsed" desc="T2CON Register Bit Description">
                                        // bit 7-3 Unimplemented, Read as 0
                                        // bit 2 TMR2ON Timer2 On bit, 1=On, 0=Off
                                        // bit 1-0 T2CKPS<1:0> Timer2 Clock Prescale Select bit
                                        // 00 = Prescaler is 1
                                        // 01 = Prescaler is 4
                                        // 10 = Prescaler is 8
                                        // 11 = Prescaler is 16
    // </editor-fold>
    
/* Analog power supply specific configurations */
    
    BUFFCON     = 0x07;                 // Unity Gain Buffer Control Register. Configured as Stand-Alone Unit and Buffer puts out Vin Divided down by 1/5.
    // <editor-fold defaultstate="collapsed" desc="BUFFCON Register Bit Description">
                                        // bit 7-5 MLTPH<2:0> System Configuration Bits
                                        // 000 = Device set as stand-alone unit
                                        // 001 = Device set as multiple output MASTER
                                        // 010 = Device set as multiple output SLAVE
                                        // 011 = Device set as multi-phase MASTER
                                        // 111 = Device set as multi-phase SLAVE
                                        // bit 4-0 ASEL<4:0> Multiplexter Output Control Bit
                                        // 00000 = Voltage proportional to current in the inductor
                                        // 00001 = Error amplifier output plus slope compensation, input to PWM comparator
                                        // 00010 = Input to slope compensation circuitry
                                        // 00011 = Band gap reference
                                        // 00100 = Output voltage reference
                                        // 00101 = Output voltage after internal differential amplifier
                                        // 00110 = Unimplemented
                                        // 00111 = voltage proportional to the internal temperature
                                        // 01000 = Internal ground for current sense circuitry
                                        // 01001 = Output overvoltage comparator reference
                                        // 01010 = Output under voltage comparator reference
                                        // 01011 = Error amplifier output
                                        // 01100 = For a multi-phase SLAVE, error amplifier signal received from MASTER
                                        // 01101 = For multi-phase SLAVE, error signal received from MASTER with gain
                                        // 01110 = Vin Divided down by 1/5
                                        // 01111 = DC Inductor Valley Current
                                        // 11101 = Over Current Reference
                                        // Everything else unimplemented.
                                        // If ATSTCON<BNCHEN> is set, the device is in Bench Test Mode and will put one of these out on GPA0. 
                                        // Be sure to add the Buffer Offset to the measured signal which can be read from memory location 0x2087.
    // </editor-fold>
    PE1         = 0x80;                 // Analog Peripheral Enable 1 Control Register. Enable diode emulation. This is a *MUST* or the battery could destroy the sync FET.
    // <editor-fold defaultstate="collapsed" desc="PE1 Register Bit Description">
                                        // bit 7 (DECON) is Diode Emulation Mode. 1(Enabled), 0(Disabled).  Enabled the synchronous rectification will disable when indutor current reaches zero.
                                        // bit 6 (DBRSTR) is High Side Drive Strength Configuration Bit.  1 (1A Sink/Source), 0 (2A Sink/Source)
                                        // bit 5 (HDLYBY) is High Side Dead Time Bypass bit
                                        // bit 4 (LDLYBY) is Low Side Dead Time Bypass bit
                                        // bit 3 (PDEN) is -V_sen Weak Pull Down Enable bit
                                        // bit 2 (PUEN) is +V_sen Weak Pull Up Enable bit
                                        // bit 1 (UVTEE) is Output Undervoltage Accelerator Enable bit
                                        // bit 0 (OVTEE) is Output Over Voltage Accelerator Enable bit
    // </editor-fold>
    DEADCON     = 0x9B;                 // Driver Dead time Control Register.  47nS high-side dead time, 48nS low-side dead time 
    //// <editor-fold defaultstate="collapsed" desc="DEADCON Bit Descriptions">
                                        // bit 7-4 HDLY<3:0> High-Side Dead Tiem Configuration Bits
                                        // 0000 = 11nsec delay
                                        // 0001 = 15nsec delay
                                        // 0010 = 19nsec delay
                                        // 0011 = 23nsec delay
                                        // 0100 = 27nsec delay
                                        // 0101 = 31nsec delay
                                        // 0110 = 35nsec delay
                                        // 0111 = 39nsec delay
                                        // 1000 = 43nsec delay
                                        // 1001 = 47nsec delay
                                        // 1010 = 51nsec delay
                                        // 1011 = 55nsec delay
                                        // 1100 = 59nsec delay
                                        // 1101 = 63nsec delay
                                        // 1110 = 67nsec delay
                                        // 1111 = 71nsec delay
                                        // bit 3-0 LDLY<3:0> Low-Side Dead Tiem Configuration Bits
                                        // 0000 = 4nsec delay
                                        // 0001 = 8nsec delay
                                        // 0010 = 12nsec delay
                                        // 0011 = 16nsec delay
                                        // 0100 = 20nsec delay
                                        // 0101 = 24nsec delay
                                        // 0110 = 28nsec delay
                                        // 0111 = 32nsec delay
                                        // 1000 = 36nsec delay
                                        // 1001 = 40nsec delay
                                        // 1010 = 44nsec delay
                                        // 1011 = 48nsec delay
                                        // 1100 = 52nsec delay
                                        // 1101 = 56nsec delay
                                        // 1110 = 60nsec delay
                                        // 1111 = 64nsec delay
    // </editor-fold>
    ABECON      = 0x0D;                 // Analog Block Enable Control Register.  Enable Signal Chain, Internal Temp Sensor, Current Measurement.
    // <editor-fold defaultstate="collapsed" desc="ABECON Register Bit Description">
                                        // bit 7 (OVDCEN) Output over voltage DAC Control bit
                                        // bit 6 (UVDCEN)Output under voltage DAC Control bit
                                        // bit 5 (MEASEN) Relative Efficiency Measurement Control bit
                                        // bit 4 (SLCPBY) Slope compensation bypass control bit. (1)Slope Comp Disabled. (0)Slope Comp Enabled.
                                        // bit 3 (CRTMEN) Current Measurement circuitry control bit
                                        // bit 2 (TMPSEN) Internal temperature sensor control bit
                                        // bit 1 (RECIREN) Relative efficiency circuit control bit
                                        // bit 0 (PATHEN) Signal chain circuitry control bit
    // </editor-fold>
    VZCCON      = 0x60;                 // Voltage for zero current register (3.28mV/bit steps). 0x60 = -104.96mV. 
    //// <editor-fold defaultstate="collapsed" desc="VZCCON Register Bit Description">
                                        // bit7-0 VZC<7:0> Voltage for Zero Current Setting Bits
                                        // 00000000 = -420mV Offset
                                        // 00000001 = -416.72mV Offset
                                        // ......
                                        // 10000000 = 0mV Offset
                                        // ......
                                        // 11111110 = +413.12mV Offset
                                        // 11111111 = +416.40mV Offset
    // </editor-fold>
    CMPZCON     = 0x0F;                 // Compensation Setting Control Register. Zero Set to 1.5kHz. Gain Set to 0dB.
    //<editor-fold defaultstate="collapsed" desc="CMPZCON Register Bit Description">
    
                                        // bit 7-4 CMPZF<3:0> Compensation Zero Frequency Setting Bits
                                        // 0000 = 1500Hz
                                        // 0001 = 1850Hz
                                        // 0010 = 2300Hz
                                        // 0011 = 2840Hz
                                        // 0100 = 3460Hz
                                        // 0101 = 4300Hz
                                        // 0110 = 5300Hz
                                        // 0111 = 6630Hz
                                        // 1000 = 8380Hz
                                        // 1001 = 9950Hz
                                        // 1010 = 12200Hz
                                        // 1011 = 14400Hz
                                        // 1100 = 18700Hz
                                        // 1101 = 23000Hz
                                        // 1110 = 28400Hz
                                        // 1111 = 35300Hz
                                        // bit 3-0 CMPZG<3:0> Compensation Gain Setting Bits
                                        // 0000 = 36.15dB
                                        // 0001 = 33.75dB
                                        // 0010 = 30.68dB
                                        // 0011 = 28.43dB
                                        // 0100 = 26.10dB
                                        // 0101 = 23.81dB
                                        // 0110 = 21.44dB
                                        // 0111 = 19.10dB
                                        // 1000 = 16.78dB
                                        // 1001 = 14.32dB
                                        // 1010 = 12.04dB
                                        // 1011 = 9.54dB
                                        // 1100 = 7.23dB
                                        // 1101 = 4.61dB
                                        // 1110 = 2.28dB
                                        // 1111 = 0dB
    //</editor-fold>
    SLPCRCON    = 0xF2;                 // Slope compensation ramp control register.  Set to 1.250 Vpk/Vpk when at 50% duty cycle. DV/Dt of 2.
    //**BRYAN** I set the SLPS above to 2 as per the datasheet. It was 0.
    // <editor-fold defaultstate="collapsed" desc="SLPCRCON Register Bit Description">
                                        // bit 7-4 SLPG<3:0> Slope Compensation Amplitude Configuration Bits
                                        // 0000 = 0.017 Vpk-pk measured from 50% duty cycle waveform
                                        // 0001 = 0.022 Vpk-pk measured from 50% duty cycle waveform
                                        // 0010 = 0.030 Vpk-pk measured from 50% duty cycle waveform
                                        // 0011 = 0.040 Vpk-pk measured from 50% duty cycle waveform
                                        // 0100 = 0.053 Vpk-pk measured from 50% duty cycle waveform
                                        // 0101 = 0.070 Vpk-pk measured from 50% duty cycle waveform
                                        // 0110 = 0.094 Vpk-pk measured from 50% duty cycle waveform
                                        // 0111 = 0.125 Vpk-pk measured from 50% duty cycle waveform
                                        // 1000 = 0.170 Vpk-pk measured from 50% duty cycle waveform
                                        // 1001 = 0.220 Vpk-pk measured from 50% duty cycle waveform
                                        // 1010 = 0.300 Vpk-pk measured from 50% duty cycle waveform
                                        // 1011 = 0.400 Vpk-pk measured from 50% duty cycle waveform
                                        // 1100 = 0.530 Vpk-pk measured from 50% duty cycle waveform
                                        // 1101 = 0.700 Vpk-pk measured from 50% duty cycle waveform
                                        // 1110 = 0.940 Vpk-pk measured from 50% duty cycle waveform
                                        // 1111 = 1.250 Vpk-pk measured from 50% duty cycle waveform
                                        // bit 3-0 SLPS<3:0> Slope Compensation Delta-V/Delta-t Configuration Bits
                                        // Set this byte proportional to the switching frequency. n = (Fsw/100000)-1
    // </editor-fold>
    SLVGNCON    = 0x00;                 // Master Error Signal Input Gain Control Register. Configured for the SLAVE device. Not used in this design so set to zero.
    // <editor-fold defaultstate="collapsed" desc="SLVGNCON Register Bit Description">
                                        // bit 7-5 Unimplemented, Read as 0
                                        // 00000 = -3.3dB
                                        // 00001 = -3.1dB
                                        // 00010 = -2.9dB
                                        // 00011 = -2.7dB
                                        // 00100 = -2.5dB
                                        // 00101 = -2.3dB
                                        // 00110 = -2.1dB
                                        // 00111 = -1.9dB
                                        // 01000 = -1.7dB
                                        // 01001 = -1.4dB
                                        // 01010 = -1.2dB
                                        // 01011 = -1.0dB
                                        // 01100 = -0.8dB
                                        // 01101 = -0.6dB
                                        // 01110 = -0.4dB
                                        // 01111 = -0.2dB
                                        // 10000 = 0.0dB
                                        // 10001 = 0.2dB
                                        // 10010 = 0.4dB
                                        // 10011 = 0.7dB
                                        // 10100 = 0.9dB
                                        // 10101 = 1.1dB
                                        // 10110 = 1.3dB
                                        // 10111 = 1.5dB
                                        // 11000 = 1.7dB
                                        // 11001 = 1.9dB
                                        // 11010 = 2.1dB
                                        // 11011 = 2.3dB
                                        // 11100 = 2.6dB
                                        // 11101 = 2.8dB
                                        // 11110 = 3.0dB
                                        // 11111 = 3.2dB
                                        // The SLVGNCON register is configured in the multi-phase SLAVE device, not the MASTER.
    
    // </editor-fold>
    CSGSCON     = 0x0d;                 // Current Sense AC Gain Control Register.  AC gain = 19dB 
    // <editor-fold defaultstate="collapsed" desc="CSGSCON Register Bit Description">
                                        // bit 3-0 CSGS<3:0> Current Sense AC Gain Setting Bits
                                        // 0000 = 0dB
                                        // 0001 = 1.0dB
                                        // 0010 = 2.5dB
                                        // 0011 = 4.0dB
                                        // 0100 = 5.5dB
                                        // 0101 = 7.0dB
                                        // 0110 = 8.5dB
                                        // 0111 = 10.0dB
                                        // 1000 = 11.5dB
                                        // 1001 = 13.0dB
                                        // 1010 = 14.5dB
                                        // 1011 = 16.0dB
                                        // 1100 = 17.5dB
                                        // 1101 = 19.0dB
                                        // 1110 = 20.5dB
                                        // 1111 = 22.0dB
    // </editor-fold>
    CSDGCON     = 0x00;                 // Current Sense DC Gain Control Register. DC gain set to 19.5dB, It's not used in control loop, but read by the ADC.
    // <editor-fold defaultstate="collapsed" desc="CSDGCON Register Bit Description">
                                        // bit7 CSDGEN Current Sense DC Gain Enable Bit: 1 = DC Gain current sense signal used in control loop. 0 = signal only read by ADC.
                                        // bit6-4 Unimplemented and read as 0
                                        // bit3 Reserved
                                        // bit2-0 CSDG<2:0> Current Sense DC Gain Setting Bits
                                        // 000 = 19.5dB
                                        // 001 = 21.8dB
                                        // 010 = 24.1dB
                                        // 011 = 26.3dB
                                        // 100 = 28.6dB
                                        // 101 = 30.9dB
                                        // 110 = 33.2dB
                                        // 111 = 35.7dB    
    // </editor-fold>
    OCCON       = 0xFF;                 // Output Overcurrent Control Register.     //**BRYAN** Go figure out what to set this to for the evaluation board.
    // <editor-fold defaultstate="collapsed" desc="OCCON Register Bit Description">
                                        // bit 7 OCEN Output Overcurrent DAC Control Bit
                                        // bit6-5 OCLEB<1:0> Leading Edge Blanking
                                        // 00 = 114ns blanking
                                        // 01 = 213ns blanking
                                        // 10 = 400ns blanking
                                        // 11 = 780ns blanking
                                        // bit4-0 OOC<4:0> Output Overcurrent Configuration bits
                                        // 00000 = 160mV drop
                                        // 00001 = 175mV drop
                                        // 00010 = 190mV drop
                                        // 00011 = 205mV drop
                                        // 00100 = 220mV drop
                                        // 00101 = 235mV drop
                                        // 00110 = 250mV drop
                                        // 00111 = 265mV drop
                                        // 01000 = 280mV drop
                                        // 01001 = 295mV drop
                                        // 01010 = 310mV drop
                                        // 01011 = 325mV drop
                                        // 01100 = 340mV drop
                                        // 01101 = 355mV drop
                                        // 01110 = 370mV drop
                                        // 01111 = 385mV drop
                                        // 10000 = 400mV drop
                                        // 10001 = 415mV drop
                                        // 10010 = 430mV drop
                                        // 10011 = 445mV drop
                                        // 10100 = 460mV drop
                                        // 10101 = 475mV drop
                                        // 10110 = 490mV drop
                                        // 10111 = 505mV drop
                                        // 11000 = 520mV drop
                                        // 11001 = 535mV drop
                                        // 11010 = 550mV drop
                                        // 11011 = 565mV drop
                                        // 11100 = 580mV drop
                                        // 11101 = 595mV drop
                                        // 11110 = 610mV drop
                                        // 11111 = 625mV drop
    // </editor-fold>
    OVCCON      = 0x00;                 // Output Voltage Set Point Coarse Control Register. Set to 0 to begin. 
    //<editor-fold defaultstate="collapsed" desc="OVCCON Register Description">
                                        // This is the output voltage error amplifier reference DAC. Coarse setting is 8bit in 15.8mV increments.
    //</editor-fold>
    OVFCON      = 0x80;                 // Output Voltage Set Point Fine Control Register.  Enable DAC and set to 0 to begin. 
    //<editor-fold defaultstate="collapsed" desc="OVFCON Register Description">
                                        // The fine setting is 5bit in 0.82mV steps. Bit 7 enables the DAC.
    // </editor-fold>
    
    //**BRYAN** Add some code for taking target voltages and addressing this setting from the configuration space if it makes sense for OUVCON and OOVCON. 
    //**BRYAN** Since these interrupts are disabled this is going to be commented out below for now.  Adding for reference code thoroughness though.
    //  OUVCON  = 0x00;                     // Output under voltage detect level control register. 
    // <editor-fold defaultstate="collapsed" desc="OUVCON Register Bit Description">
                                        // bit 7-0 OUV<7:0> Output Under Voltage Detect Level Configuration Bits
                                        // Equation is OUV<7:0> = (V_out_uv_detect_level)/15mV
    // </editor-fold>
    //  OOVCON  = 0xff;                     // Output over voltage detect level control register.
    // <editor-fold defaultstate="collapsed" desc="OOVCON Register Bit Description">
                                        // bit 7-0 OOV<7:0> Output Overvoltage Detect Level Configuration bits
                                        // Equation is OOV<7:0> = V_out_ov_detect_level / 15mV
    // </editor-fold>

    vh          = 0;                    // DAC High Byte 
    vl          = 0;                    // DAC Low Byte 

/* Enable input Under Voltage Lockout threshold */
    VINLVL = 0x80 | cs.uvlo_threshold_on;  // Input Under Voltage Lockout Control Register. Enable and set via configuration in cs.uvlo_threshold_on
    // <editor-fold defaultstate="collapsed" desc="VINLVL Register Bit Description">
                                        // bit 7 UVLOEN Under Voltage Lockout DAC Control bit, 1=Enabled, 0=Disabled
                                        // bit 6 Unimplemented = 0
                                        // bit 5-0 UVLO<5:0> Under voltage Lockout Configuration Bits = 26.5*ln(UVLO_SET-POINT/4)
    // </editor-fold>
}

void inc_iout(short increment)      // Topology Specific function - Increase Current (Combined)
{
	int i;

  for(i=0;i<increment;i++)			// Loop through the increment routine (if fast_ramp requires)
  {
    vl++;							// Increment fine register
    if (vl > IOUT_LSB_ROLL) 		// If we are over the top of the fine register
	{
		if (vh < cs.ovcfcon_max) {	// and the coarse register is not maxed out
            vh ++;					// increment coarse register
			vl = 0;					// and reset fine register to zero
        }
        else {
            vh = cs.ovcfcon_max;	// Set to max
            vl = IOUT_LSB_ROLL;		// We went one increment beyond IOUT_LSB_ROLL, so reset to max.
        }
    }
  }
    OVFCON = (unsigned short)(0x80 | vl);             //Load the DAC with the new values
    OVCCON = vh;
}

short dec_iout( short decrement)	// Topology Specific function - Decrease Current (Combined)
{
	int i;

    if ((vh == 0) && (vl == 0))     // Are we already at the bottom (off)?
        return(0);
	for (i = 0; i<decrement; i++)
	{
		if(vl > 0)					// Must check for zero because 'vl' can roll to FF if not.
			vl--;					// Reduce fine value if we are not already at the bottom of the range

		if (vl == 0) 				// If fine == 0
		{
			if (vh > cs.ovcfcon_min) // and corse > the minimum
			{
				vh--;				 // Then decrease course and reset fine to top of range
				vl = IOUT_LSB_ROLL;	 //
			}
			else
			{
				vh = cs.ovcfcon_min; // Else, we are at the bottom (vl = vf = 0)
				return(0);			 // Return a fault code that we can't decrement any more
			}
		}
	}
    OVFCON = (unsigned short)(0x80 | vl);	// Update the setpoint registers
    OVCCON = vh;		//
	return(1);			// Success
}

void zero_iout(void)
{
	/* Output current set to zero */
	vl = 0;
	vh = cs.ovcfcon_min;
	OVCCON = 0;
	OVFCON = 0x80;
}
void check_button(void)
{
}

void hardware_custom_a(void)
{
}

void disable_charger(void)
{
	/* PWM off */
	ATSTCONbits.DRVDIS = 1;

	/* Output FET off */
	PORTBbits.RB7 = 0;

	/* Adjust UVLO threashold */
	VINLVL = 0x80 | cs.uvlo_threshold_on;
    
    /* Set output current to minimum */
    zero_iout();
}

void enable_charger(void) //%%BRYAN%% need to add the output overcurrent and overvoltage interrupt enables
{
    /* Set output current to minimum */
    zero_iout();

	/* Adjust UVLO threshold */
	VINLVL = 0x80 | cs.uvlo_threshold_off;

	/* UVLO interrupt */
	//PIE2bits.VINIE = 1;

    
    /* PWM on */
	ATSTCONbits.DRVDIS = 0;

	/* Input FET on */
	connect_battery();
}

#ifdef ENABLE_FIXED_CONFIG
#include "MultiChemCharger_Values.h"
#endif

void init_cal_registers(void)
{
    PMADRH = 0x20;
    PMADRL = 0x80;
    PMCON1bits.CALSEL = 1;
    PMCON1bits.RD = 1;
    asm("nop");
    asm("nop");
    DOVCAL = PMDATH;                    // Load Device Calibration from memory (Voltage sense differential amp adjust)
    OSCCAL = PMDATL;                    // Load OSCCAL value from memory (Oscillator tune bits)

    PMADRH = 0x20;
    PMADRL = 0x82;
    PMCON1bits.CALSEL = 1;
    PMCON1bits.RD = 1;
    asm("nop");
    asm("nop");
    VROCAL = PMDATH;                    // Over-temp calibration bits
    BGRCAL = PMDATL;                    // Internal band-gap reference calibration

    PMADRH = 0x20;
    PMADRL = 0x83;
    PMCON1bits.CALSEL = 1;
    PMCON1bits.RD = 1;
    asm("nop");
    asm("nop");
    TTACAL = PMDATH;                    // Output voltage regulation reference set-point calibration
    ZROCAL = PMDATL;                    // Error amplifier offset calibration
}

/* This ifdef is meant to save a little code and prevent flash writes when this device function was locked down.  */
/* While it adds the ability to include MultiChemCharger_Values.h it locks out the ability for the GUI to change  */
/* configuration settings. Be aware of this.  If you are testing compiling with the header file and still want to */ 
/* be able to use the GUI then be sure to set the ENABLE_GUI_CONFIG and ENABLE_FIXED_CONFIG in the header file.   */
#ifdef ENABLE_GUI_CONFIG   

/* This function writes 8 bytes (4 flash words) at a time to flash. */
/* The incoming address must be 8-byte aligned.                     */

unsigned short write_flash(unsigned short addr, unsigned short counter) 
{
    unsigned char b = 0;
	if (addr < CAL_BASE_ADDR)           // Invalid address.  Should return a failure result
	{
        return 0x00; 
    }
    if (counter == 8)				// Write data if 8 bytes (4-words) have been received
    {
        while (b < 8) 
        {
            PMADRH = addr >> 8;
            PMADRL = addr;
            PMDATL = flash_write_buf[(unsigned char)(b & 0x07)];
            b++;
            PMDATH = flash_write_buf[(unsigned char)(b & 0x07)];
            b++;
            PMCON1bits.CALSEL = 0;
            PMCON1bits.WREN = 1;
            PMCON2 = 0x55;
            PMCON2 = 0xAA;
            PMCON1bits.WR = 1;
            NOP();
            NOP();
            PMCON1bits.WREN = 0;
            addr++;
        }
        return 0x04;
    }
    return 0x00;
}
#endif

unsigned short read_flash(unsigned short addr)
{
    unsigned short a;

    PMADRH = addr >> 8;
    PMADRL = addr;
    PMCON1bits.CALSEL = 0;
    PMCON1bits.RD = 1;
    asm("nop");
    asm("nop");
    a = (unsigned short)(PMDATH << 8);
    a |= PMDATL;
    return a;
}

unsigned char check_for_uvlo(void)
{
    if ((cd.adc_vin < cs.uvlo_adc_threshold_off) || (PIR2bits.VINIF == 1))
	{
		/* Attempt a reset */
		PIR2bits.VINIF = 0;
		return 1;
	}
    return 0;
}

#ifdef ENABLE_STATUS_LEDS
/************************************************************************************/
/* This board only has two LEDs so we are going to set up a Truth table as follows: */
/* State    LED #1:     LED #2:                                                     */
/* Charging ON          OFF                                                         */
/* Complete ON          ON                                                          */
/* Fault    OFF         ON                                                          */
/* Stopped  OFF         OFF                                                         */
/* If you want to use TP6 (GPA2) and TP8 (GPA1) for LED3 and LED4 respectively then */
/* replace the function below with the reference code at the top of this file in the*/
/* comments.  You could also deadbug both an LED and bias resistor onto R32 and R36.*/
/************************************************************************************/
void update_status_leds(void)
{
	// Charger Off
	if(cd.charger_state == 0 & cd.status.complete != 1)		
	{	
        if((cd.status.word & 0x7FFF) != 0)       // Fault has occurred
        {
            LED1_PIN = 0;						
            LED2_PIN = 0;
        }						
    // Fault
        else
        {
            LED1_PIN = 0;
            LED2_PIN = 1;
        }
	}
    // Charger Complete
    else if (cd.status.complete == 1)
    {
        LED1_PIN = 1;
        LED2_PIN = 1;
    }
    // Charge Active
    else
    {
        LED1_PIN = 1;
        LED2_PIN = 0;
    }
}
#endif // end Enable Status LEDS

#ifdef ENABLE_BENCH_TEST_OUTPUT_PIN
void update_buffcon(void) /* for parts with a copy of an analog value on a pin. */
{
    //The Bench Test Output Pin is set here.  Update the analog mux source here.
    //BUFFCON = cd.buffcon_reg & 0x1F;	
    // <editor-fold defaultstate="collapsed" desc="BUFFCON Register Bit Description">
                                        // bit 7-5 MLTPH<2:0> System Configuration Bits
                                        // 000 = Device set as stand-alone unit
                                        // 001 = Device set as multiple output MASTER
                                        // 010 = Device set as multiple output SLAVE
                                        // 011 = Device set as multi-phase MASTER
                                        // 111 = Device set as multi-phase SLAVE
                                        // bit 4-0 ASEL<4:0> Multiplexter Output Control Bit
                                        // 00000 = Voltage proportional to current in the inductor
                                        // 00001 = Error amplifier output plus slope compensation, input to PWM comparator
                                        // 00010 = Input to slope compensation circuitry
                                        // 00011 = Band gap reference
                                        // 00100 = Output voltage reference
                                        // 00101 = Output voltage after internal differential amplifier
                                        // 00110 = Unimplemented
                                        // 00111 = voltage proportional to the internal temperature
                                        // 01000 = Internal ground for current sense circuitry
                                        // 01001 = Output overvoltage comparator reference
                                        // 01010 = Output under voltage comparator reference
                                        // 01011 = Error amplifier output
                                        // 01100 = For a multi-phase SLAVE, error amplifier signal received from MASTER
                                        // 01101 = For multi-phase SLAVE, error signal received from MASTER with gain
                                        // 01110 = Vin Divided down by 1/5
                                        // 01111 = DC Inductor Valley Current
                                        // 11101 = Over Current Reference
                                        // Everything else unimplemented.
                                        // If ATSTCON<BNCHEN> is set, the device is in Bench Test Mode and will put one of these out on GPA0. 
                                        // Be sure to add the Buffer Offset to the measured signal which can be read from memory location 0x2087.
    // </editor-fold>
    cd.user_interface_mode = 3;		// Tell the GUI we are in bench-test mode

}
#endif

void connect_battery(void)
{
    /* Connect Battery Switch */
     PORTBbits.GPB7 = 1;
}

void disconnect_battery(void)
{
    /* PWM off: We don't want the charger running when we disconnect the battery or bad things will happen */
	ATSTCONbits.DRVDIS = 1;
    PORTBbits.GPB7 = 0;
    /* Disconnect Battery Switch */
}

#endif	// end of MCP19111 Configuration
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="MCP19118_FOUR_SWITCH_BUCK_BOOST_CHARGER">
#ifdef MCP19118_FOUR_SWITCH_BUCK_BOOST_CHARGER
unsigned char vh, vl;
extern struct charger_settings_t cs;	// Get access to Charger_Settings structure
extern struct charger_data_t cd;		// Get access to Charger_Data structure

void interrupt isr()
{
    if (PIR2bits.VINIF == 1) 	// UVLO flag set?
	{
        // Disable the interrupt 
        PIE2bits.VINIE = 0;
        
        // PWM off 
        ATSTCONbits.DRVDIS = 1;
        
        // Adjust UVLO threshold 

        // Set the charge state back to OFF
        cd.charger_state = CHARGER_OFF;
        
		// log the reason 
		cd.status.vinuvlo = 1;
    }
}

void init_hardware()	/* MCP19118 Hardware specific configuration. */
{
    OSCTUNE = 0x00;                     // Oscillator Tuning Register. Set oscillator to Calibrated Center Frequency
    // <editor-fold defaultstate="collapsed" desc="OSCTUNE Register bit Description">
                                        //bit 7-5 Unimplemented
                                        //bit 4-0 TUN<4:0> Frequency Tuning Bits
                                        //b01111 = Maximum Frequency.
                                        //b01110 thru b00001 = monotonic ramp in between maximum and center.
                                        //b00000 = Center frequency.  Oscillator Module is running at the calibrated frequency.
                                        //b11111 thru b10001 = monotonic ramp in between center and minimum.
                                        //b10000 = Minimum Frequency
    // </editor-fold>
    OPTION_REG = 0x00;                  // Prescaler (bits 2:0) set to 000 for 1:2 TMR0 Rate, 256usec per timer overflow.  
    // <editor-fold defaultstate="collapsed" desc="OPTION_REG Register Description">
                                        // bit 7 RAPU# Port GPx Pull_up Enable bit, 1=Disabled, 0=Enabled
                                        // bit 6 INTEDG Interrupt Edge Select bit, 0=Interrupt on rising edge of INT pin, 1=falling edge
                                        // bit 5 T0CS TMR0 Clock Source Select bit, 1=Transition on T0CKI pin, 0=Internal instruction cycle clock
                                        // bit 4 T0SE TMR0 Source Edge Select bit, 1=Increment on high to low transition on T0CKI pin, 0=low to high
                                        // bit 3 PSA Prescaler Assignment bit, 1=Prescalar is assigned to WDT, 0=Prescaler is assigned to Timer0 module
                                        // bit 2-0 PS<2:0> Prescalar Rate Select Bits:
                                        //  000 - 1:2 TMR0 Rate 1:1 WDT Rate
                                        //  001 - 1:4 TMR0 Rate 1:2 WDT Rate
                                        //  010 - 1:8 TMR0 Rate 1:4 WDT Rate
                                        //  011 - 1:16 TMR0 Rate 1:8 WDT Rate
                                        //  100 - 1:32 TMR0 Rate 1:16 WDT Rate
                                        //  101 - 1:64 TMR0 Rate 1:32 WDT Rate
                                        //  110 - 1:128 TMR0 Rate 1:64 WDT Rate
                                        //  111 - 1:256 TMR0 Rate 1:128 WDT Rate
    // </editor-fold>
    PIR1    = 0;                        // Peripheral Interrupt Flag Register 1. All Interrupts Cleared.
    // <editor-fold defaultstate="collapsed" desc="PIR1 Register Bit Description">
                                        // bit 7 Unimplemented, Read as 0
                                        // bit 6 ADIF ADC Interrupt Flag bit, 1=ADC Conversion Complete, 0=Conversion not complete or not started
                                        // bit 5 BCLIF MSSP Bus Collision Interrupt Flag bit, 1=Interrupt is pending, 0=Interrupt is not pending
                                        // bit 4 SSPIF Synchronous Serial Port (MSSP) Interrupt Flag bit, 1=Interrupt is pending, 0=Interrupt not pending.
                                        // bit 3-2 Unimplemented, Read as 0
                                        // bit 1 TMR2IF Timer2 to PR2 Match Interrupt Flag, 1=Timer2 to PR2 match occurred (must be cleared in SW), 0=Match did not occur
                                        // bit 0 TMR1IF Timer1 Interrupt Flag.  1=Timer1 rolled over (must be cleared in SW). 0=Timer 1 has not rolled over.
    // </editor-fold>
    PIE1    = 0;                        // Peripheral Interrupt Enable Register 1. All Interrupts Disabled.
    // <editor-fold defaultstate="collapsed" desc="PIE1 Register Bit Description">
                                        // bit 7 Unimplemented. Read as 0.
                                        // bit 6 ADC Interrupt Enable bit, 1=Enables ADC Interrupt, 0=Disables
                                        // bit 5 BCLIE MSSP Bus Collision Interrupt Enable bit, 1=Enables the MSSP bus Collision Interrupt, 0=Disables
                                        // bit 4 SSPIE synchronous Serial Port (MSSP) Interrupt Enable bit, 1=Enables MSSP Bus Collision Interrupt, 0=Disables
                                        // bit 3-2 Unimplemented
                                        // bit 1 TMR2IE Timer2 Interrupt Enable, 1=Enables the Timer2 Interrupt, 0=Disables
                                        // bit 0 TMR1IE Timer1 Interrupt Enable, 1=Enables the Timer1 Interrupt, 0=Disables
    // </editor-fold>
    PIR2    = 0;                        // Peripheral Interrupt Flag Register 2.  All interrupts Cleared.
    // <editor-fold defaultstate="collapsed" desc="PIR2 Register Bit Description">
                                        // bit 7 UVIF Output undervoltage error interrupt flag bit, 1=Output Undervoltage error has occurred, 0=Has not occured
                                        // bit 6 Unimplemented, Read as 0.
                                        // bit 5 OCIF Output overcurrent error interrupt flag bit, 1=Output OC error has occurred, 0=Has not occured
                                        // bit 4 OVIF Output overvoltage error interrupt flag bit, 1=Output OV error has occurred, 0=Has not occured
                                        // bit 3-2 Unimplemented, Read as 0
                                        // bit 1 VINIF Vin Status bit, 1=Vin is below acceptable level, 0=Vin is at acceptable level
                                        // bit 0 Unimplemented, Read as 0
    // </editor-fold>
    PIE2    = 0;                        // Peripheral Interrupt Enable Register 2. All Interrupts Disabled.
    // <editor-fold defaultstate="collapsed" desc="PIE2 Register Bit Description">
                                        // bit 7 UVIE Output Under voltage Interrupt enable bit, 1=Enables teh UV interrupt, 0=Disables
                                        // bit 6 Unimplemented, Read as 0
                                        // bit 5 OCIE Output Overcurrent Interrupt enable bit, 1=Enables the OC Interrupt, 0=Disables
                                        // bit 4 OVIE Output Overvoltage Interrupt enable bit, 1=Enables the OV Interrupt, 0=Disables
                                        // bit 3-2 Unimplemented, Read as 0
                                        // bit 1 VINIE V_in UVLO Interrupt Enable, 1=Enables teh V_IN UVLO Interrupt, 0=Disables
                                        // bit 0 Unimplemented, Read as 0
    // </editor-fold>
    INTCON  = 0xc0;                     // Global + Peripheral interrupts.  Enable all Unmasked Interrupts and Unmasked Peripheral Interrupts.  All the rest are disabled.
    // <editor-fold defaultstate="collapsed" desc="INTCON Register Bit Description">
                                        // bit 7 GIE Global Interrupt Enable bit, 1=Enables all unmasked interrupts, 0=Disables
                                        // bit 6 PEIE Peripheral Interrupt Enable bit, 1=Enables all unmasked peripheral interrupts, 0=Disables
                                        // bit 5 TMR0 Overflow Interrupt Enable bit, 1=Enables the TMR0 Interrupt, 0=Disables
                                        // bit 4 INTE The INT External Interrupt Enable bit, 1=Enables the INT external interrupt. 0=Disables
                                        // bit 3 IOCE Interrupt on Change Enable bit, 1=Enables the interrupt-on-change interrupt, 0=Disables
                                        // bit 2 T0IF TMR0 Overflow Interrupt Flag big, 1=TMR0 register has overflowed (must be cleared in software), 0=TMR0 did not overflow
                                        // bit 1 INTF External Interrupt Flag bit. 1=External Interrupt Occurred (must be cleared in software), 0=External Interrupt did not occur
                                        // bit 0 IOCF Interrupt on Change Interrupt Flag bit, 1=When at least one of the IOC pins changed state, 0=None have changed state
    // </editor-fold>
    IOCA = 0x00;                        // Interrupt on Change PortGPA Register.  Disabled on all Port A pins.  Setting a bit also requires bit 3 of INTCON be set to enable interrupt on change.
    // <editor-fold defaultstate="collapsed" desc="IOCA Register Bit Description">
                                        // bit 7-6 Interrupt on Change PortGPA Register bits, 1=enabled, 0=disabled
                                        // bit 5 Interupt on Change PortGPA Register bits, 1=enabled, 0=disabled
                                        // bit 4-0 Interrupt on Change PortGPA Register bits, 1=enabled, 0=disabled
    // </editor-fold>
    //%%BRYAN%% Investigate the pro/con of Adding the GPB3 button as an Interrupt on Change and add it to ISR routine instead of relying on a loop.
    IOCB = 0x00;                        // Interrupt on Change PortGPB Register.  Disabled on all Port B pins.  Setting a bit also requires bit 3 of INTCON be set to enable interrupt on change.
    // <editor-fold defaultstate="collapsed" desc="IOCB Register Bit Description">
                                        // bit 7-4 and 2-0 Interrupt on Change PortGPA Register bits, 1=enabled, 0=disabled
                                        // bit 3 Unimplemented read as 0
    // </editor-fold>
    ADCON0  = ADC_MUX_VIN | 0x01;       // ADC Control Register 0.  Enables the ADC and sets the default input channel to VIN_ANA (00000)
    // <editor-fold defaultstate="collapsed" desc="ADCON0 Register Bit Description">
                                        // bit 7 Unimplemented, read as 0
                                        // bit 6-2 CHS<4:0> Analog Channel Selected bits
                                        // 00000 = VIN_ANA analog voltage proportional to Vin
                                        // 00001 = VREGREF reference voltage for V_REG output
                                        // 00010 = OV_REF feference for overvoltage comparator
                                        // 00011 = UV_REF reference for undervoltage comparator
                                        // 00100 = VBGR band gap reference
                                        // 00101 = INT_VREG internal versin of the V_REG load voltage
                                        // 00110 = CRT voltage proportional to the current in the indutor
                                        // 00111 = VZC an internal ground, voltage for zero current
                                        // 01000 = DEMAND input to current loop, output of demand mux
                                        // 01001 = RELEFF analog voltage proportional to duty cycle
                                        // 01010 = TMP_ANA analog voltage proportional to temperature
                                        // 01011 = ANA_IN demanded current from the remote master
                                        // 01100 = DCI DC Inductor valley current
                                        // 01101 = Unimplemented
                                        // 01110 = Unimplemented
                                        // 01111 = Unimplemented
                                        // 10000 = GPA0 i.e. ADDR1
                                        // 10001 = GPA1 i.e. ADDR0
                                        // 10010 = GPA2 i.e. Temperature Sensor Input
                                        // 10011 = GPA3 i.e. Tracking Voltage
                                        // 10100 = GPB1
                                        // 10101 = GPB2
                                        // 10110 = GPB4
                                        // 10111 = GPB5
                                        // 11000 = Unimplemented
                                        // 11001 = Unimplemented
                                        // 11010 = Unimplemented
                                        // 11011 = Unimplemented
                                        // 11100 = Unimplemented
                                        // 11101 = Unimplemented
                                        // 11110 = Unimplemented
                                        // 11111 = Unimplemented
                                        // bit 1, GO/Done#, 1=ADC in Progress, 2=complete or not-in-progress
                                        // bit 0, ADON, 1=Enable ADC, 0=Disable ADC
    // </editor-fold>
    ADCON1  = 0x20;                     // ADC Control Register 1.  Enable F_OSC / 32 which sets 4usec per conversion.
    // <editor-fold defaultstate="collapsed" desc="ADCON1 Register Bit Description">
                                        // bit 7 Unimplemented, Read as 0
                                        // bit 6-4 ADCS<2:0> ADC Clock Select Bits
                                        // 000 = Reserved
                                        // 001 = F_OSC/8
                                        // 010 = F_OSC/32
                                        // x11 = F_RC (clock derived from internal oscillator with a divisor of 16)
                                        // 100 = Reserved
                                        // 101 = F_OSC/16
                                        // 111 = F_OSC/64
                                        // bit 3-0 Unimplemented, Read as 0
    // </editor-fold>
	PORTGPA = 0b00000000;               // Clear port A to start
    // <editor-fold defaultstate="collapsed" desc="PORTGPA Register Bit Description">
                                        // bit 7 GPA7 General Purpose Open Drain I/O Pin
                                        // bit 6 GPA6 General Purpose I/O Pin, 1=Port pin is > VIH, 0=Port Pin is < VIL
                                        // bit 5 GPA5/MCLR# General Purpose Open Drain I/O pin
                                        // bit 4 GPA7 General Purpose Open Drain I/O pin
                                        // bit 3-0 GPA<3:0> General Purpose I/O Pin, 1=Port Pin is > VIH, 0=Port Pin is < VIL
    // </editor-fold>

#ifdef ENABLE_BENCH_TEST_OUTPUT_PIN
	ATSTCON = 0b10000011;               // Analog Bench Test Control Register, Configure output MOSFET drivers (safe power-up, so do this first thing), GPA0 set for Benchtest.
    // <editor-fold defaultstate="collapsed" desc="ATSTCON Register Bit Description">
                                        // bit 7 reserved
                                        // bit 6-5 Unimplemented and Read as 0
                                        // bit 4 Reserved
                                        // bit 3 HIDIS: High Side Driver Control Bit, 1=Disabled, 0=Enabled
                                        // bit 2 LODIS: Low Side Driver Control Bit, 1=Disabled, 0=Enabled
                                        // bit 1 BNCHEN: GPA0 bench test configuration control bit, 1=GPA0 is configured for analog bench test output, 0=GPA0 Normal Operation
                                        // bit 0 DRVDIS: MOSFET Driver Disable Control Bit, 1=High and Low Side Drivers set Low, PHASE floating, 0=Normal Operation
    // </editor-fold>
    TRISGPA = 0b11101010;               // Port A Tristate Register. A7,A6,A5,A3, A1 as Input. Benchtest requires A0 to be an output. Test Pins on A4, A2.
    // <editor-fold defaultstate="collapsed" desc="TRISGPA Register Bit Description">
                                        // bit 7-6 TRISA<7:6> PORTGPA Tri-State Control bit, 1=input (tri-stated), 0=output
                                        // bit 5 TRISA5 GPA5 Tri-State Control bit. This bit is always 1 as GPA5 is an input only
                                        // bit 4-0 TRISA<4:0> PORTGPA Tri-State Control Bit, 1=input (tri-state), 0=output
    // </editor-fold>
	ANSELA  = 0b00001001;               // Port A Analog Select Register.  A0 and A3 are Analog inputs.  Benchtest requires GPA0 be configured for analog.
    // <editor-fold defaultstate="collapsed" desc="ANSELA Register Bit Description">
                                        // bit 7-4 Unimplemented, Read as 0
                                        // bit 3-0 Digital I/O. Pin is assigned to port or special function.
    // </editor-fold>
    WPUGPA  = 0b00100010;               // Port A Weak Pull-Up Register for digital inputs (A7, A6 are open-drain). A5 (MCLR#) has a weak pullup. A1 is a no connect.
    // <editor-fold defaultstate="collapsed" desc="WPUGPA Register Bit Description">
                                        // bit 7-6 Unimplemented, Read as 0
                                        // bit 5 WPUA5 Weak Pull-Up Register bit, 1=Pull-up enabled, 0=Disabled
                                        // bit 4 Unimplemented, Read as 0
                                        // bit 3-0 WPUA<3:0> Weak Pull-up Register Bit, 1=Pull-up enabled, 0=Disabled
                                        // NOTE: The weak pull-up device is enabled only when the global RAPU# bit is enabled, the pin is in input mode (TRISGPA=1),
                                        //       the individual WPUA bit is enabled (WPUA=1), and the pin is not configured as an analog input.
    // </editor-fold>

#else
	ATSTCON = 0b10000001;               // Analog Bench Test Control Register, Configure output MOSFET drivers (safe power-up, so do this early)
    // <editor-fold defaultstate="collapsed" desc="ATSTCON Register Bit Description">
                                        // bit 7 reserved
                                        // bit 6-5 Unimplemented and Read as 0
                                        // bit 4 Reserved
                                        // bit 3 HIDIS: High Side Driver Control Bit, 1=Disabled, 0=Enabled
                                        // bit 2 LODIS: Low Side Driver Control Bit, 1=Disabled, 0=Enabled
                                        // bit 1 BNCHEN: GPA0 bench test configuration control bit, 1=GPA0 is configured for analog bench test output, 0=GPA0 Normal Operation
                                        // bit 0 DRVDIS: MOSFET Driver Disable Control Bit, 1=High and Low Side Drivers set Low, PHASE floating, 0=Normal Operation
    // </editor-fold>
    TRISGPA = 0b11101010;               // Port A Tristate Register.  A7, A6, A5, A3, A1 set as Input.  LED outputs are on GPA0 active high. Test outputs are on GPA2, GPA4.
    // <editor-fold defaultstate="collapsed" desc="TRISGPA Register Bit Description">
                                        // bit 7-6 TRISA<7:6> PORTGPA Tri-State Control bit, 1=input (tri-stated), 0=output
                                        // bit 5 TRISA5 GPA5 Tri-State Control bit. This bit is always 1 as GPA5 is an input only
                                        // bit 4-0 TRISA<4:0> PORTGPA Tri-State Control Bit, 1=input (tri-state), 0=output
    // </editor-fold>
	ANSELA  = 0b00001000;               // Port A Analog Select Register.  A3 is an analog input. Benchtest pin disabled for LED user interfaces
    // <editor-fold defaultstate="collapsed" desc="ANSELA Register Bit Description">
                                        // bit 7-4 Unimplemented, Read as 0
                                        // bit 3-0 Digital I/O. Pin is assigned to port or special function.
    // </editor-fold>
    WPUGPA  = 0b00100010;               // Port A Weak Pull-Up Register for digital inputs (A7, A6 are open-drain). A5 (MCLR#) has a weak pullup. A1 is a no connect. A4,A3,A2 have external pullups
    // <editor-fold defaultstate="collapsed" desc="WPUGPA Register Bit Description">
                                        // bit 7-6 Unimplemented, Read as 0
                                        // bit 5 WPUA5 Weak Pull-Up Register bit, 1=Pull-up enabled, 0=Disabled
                                        // bit 4 Unimplemented, Read as 0
                                        // bit 3-0 WPUA<3:0> Weak Pull-up Register Bit, 1=Pull-up enabled, 0=Disabled
                                        // NOTE: The weak pull-up device is enabled only when the global RAPU# bit is enabled, the pin is in input mode (TRISGPA=1),
                                        //       the individual WPUA bit is enabled (WPUA=1), and the pin is not configured as an analog input.
    // </editor-fold>
#endif

	PORTGPB = 0b00000000;               // Clear port B to start (GPB3 not present as a pin on this part)
    // <editor-fold defaultstate="collapsed" desc="PORTGPB Register Bit Description">
                                        // bit 7-4 GPB<7:4> General Purpose I/O Pin bit, 1=Port pin is >VIH, 0=Port Pin is <VIL
                                        // bit 3 Unimplemented, Read as 0
                                        // bit 2-0 GPB<2:0> General Purpose I/O Pin bit, 1=Port pin is >VIH, 0=Port Pin is <VIL
    // </editor-fold>
    TRISGPB = 0b11111111;               // Port B Tristate Register.  B7, B6, B5, B4, B3, B1, B0 set as Input. Button input is GPB2 active high.
    // <editor-fold defaultstate="collapsed" desc="TRISGPB Register Bit Description">
                                        // bit 7-4 TRISB<7:4> PORTGPB Tri-State Control Bit, 1=Pin configured as input (tri-stated), 0=pin configured as output
                                        // bit 3 Unimplemented, Read as 1.
                                        // bit 2-0 TRISB<2:0> PORTGPB Tri-State Control Bit, 1=Pin configured as input (tri-stated), 0=pin configured as output
    // </editor-fold>
    ANSELB  = 0b00000010;               // Port B Analog Select Register. GPB1 is an analog input for measuring VBAT amplifier circuit.
    // <editor-fold defaultstate="collapsed" desc="ANSELB Register Bit Description">
                                        // bit 7-6 Unimplemented, Read as 0.
                                        // bit 5-4 ANSB<5:4> Analog Select PORTGPB Register Bit, 1=Analog input, 0=Digital I/O or Special Function
                                        // bit 3 Unimplemented, Read as 0.
                                        // bit 2-1 ANSB<2:1> Analog Select PORTGPB Register bit, 1=Analog input, 0=Digital I/O or Special Function
                                        // bit 0 Unimplemented, Read as 0.
    // </editor-fold>
    WPUGPB  = 0b01111100;               // Port B Weak Pull-up Register for digital inputs (B0 is an open-drain, B1 is an anlaog input). B2-B7 are not in circuit & have pullups.
    // <editor-fold defaultstate="collapsed" desc="WPUGPB Register Bit Description">
                                        // bit 7-4 WPUB<7:4> Weak Pull-Up Register bit
                                        // bit 3 Unimplemented, Read as 0
                                        // bit 2-1 WPUB<2:1> Weak Pull-Up Register bit
                                        // bit 0 Unimplemented, Read as 0.
                                        // NOTE: The weak pull-up device is enabled only when the global RAPU# bit is enabled, the pin is in input mode (TRISGPB=1),
                                        //       the individual WPUB bit is enabled (WPUB=1), and the pin is not configured as an analog input.
    // </editor-fold>

/* Set up the PWM Generator with Timer2. Frequency and Maximum Duty Cycle. */
    
    T2CON   = 0x00;                     // Timer2 Control Register. Timer OFF, Prescalar at 1
    // <editor-fold defaultstate="collapsed" desc="T2CON Register Bit Description">
                                        // bit 7-3 Unimplemented, Read as 0
                                        // bit 2 TMR2ON Timer2 On bit, 1=On, 0=Off
                                        // bit 1-0 T2CKPS<1:0> Timer2 Clock Prescale Select bit
                                        // 00 = Prescaler is 1
                                        // 01 = Prescaler is 4
                                        // 10 = Prescaler is 8
                                        // 11 = Prescaler is 16
    // </editor-fold>
    TMR2    = 0x00;                     // Timer2 Time Base set to 0x00.  This is the target duty cycle.
    // <editor-fold defaultstate="collapsed" desc="TMR2 Register Bit Description">
                                        // bit 7-0 Holding Register for the 9-bit TMR2 Time Base.  Basically the target duty cycle (D = TMR2/PR2)
    // </editor-fold>
    APFCON  = 0x00;                     // Alternate Pin Function Control Register. Leave at Default 0x00.
    // <editor-fold defaultstate="collapsed" desc="APFCON Register Bit Description">
                                        // bit 7-1 Unimplemented, Read as 0
                                        // bit 0 CLKSEL Pin Selection Bit. 1=Multi-phase or multiple output clock function is on GPB5, 0=On GPA1.
    // </editor-fold>
    PWMPHL  = 0x00;                     // SLAVE Phase shift. Not used and set to 0.
    // <editor-fold defaultstate="collapsed" desc="PWMPHL Register Bit Description">
                                        // bit 7-0 Phase shift added to the SLAVE system clock received on GPA1 in a MASTER/SLAVE or multi-phase implementation. 
                                        // Not used in Single Phase.  If APFCON bit 0 is set then the clock is received on GPB5 instead.
    // </editor-fold>
    PWMRL   = 0x19;                     // Sets the maximum PWM duty cycle.  Set to 25.  So (25*296kHz)/8MHz = 93.75%
    // <editor-fold defaultstate="collapsed" desc="PWMRL Register Bit Description">
                                        // bit 7-0 PWM Register Low Byte.  Equation is PWM_Duty_Cycle = PWMRL * T_OSC * TMR2_Prescale_Value
                                        // PWMRL can be written to at any time, but it will not be latched into PWMRH until after a match between PR2 and TMR2 occurs.
    // </editor-fold>
    PR2     = 0x1B;                     // Timer2 Period Register.  Set to 300 kHz frequency: 8MHz/300kHz = 26.7. So Set 27 for 296kHz.
    // <editor-fold defaultstate="collapsed" desc="PR2 Register Bit Description">
                                        // bit 7-0 Timer2 Module Period Register
                                        // This is the PWM period.  The equation is PWM_Period = [(PR2)+1] * T_OSC * T2_Prescale_Value
    // </editor-fold>
    T2CON   = 0x04;                     // Timer 2 Control Register. Timer ON. Prescalar at 1.
    // <editor-fold defaultstate="collapsed" desc="T2CON Register Bit Description">
                                        // bit 7-3 Unimplemented, Read as 0
                                        // bit 2 TMR2ON Timer2 On bit, 1=On, 0=Off
                                        // bit 1-0 T2CKPS<1:0> Timer2 Clock Prescale Select bit
                                        // 00 = Prescaler is 1
                                        // 01 = Prescaler is 4
                                        // 10 = Prescaler is 8
                                        // 11 = Prescaler is 16
    // </editor-fold>
    
/* Analog power supply specific configurations */
    
    BUFFCON     = 0x07;                 // Unity Gain Buffer Control Register. Configured as Stand-Alone Unit and Buffer puts out Temperature.
    // <editor-fold defaultstate="collapsed" desc="BUFFCON Register Bit Description">
                                        // bit 7-5 MLTPH<2:0> System Configuration Bits
                                        // 000 = Device set as stand-alone unit
                                        // 001 = Device set as multiple output MASTER
                                        // 010 = Device set as multiple output SLAVE
                                        // 011 = Device set as multi-phase MASTER
                                        // 111 = Device set as multi-phase SLAVE
                                        // bit 4-0 ASEL<4:0> Multiplexter Output Control Bit
                                        // 00000 = Voltage proportional to current in the inductor
                                        // 00001 = Error amplifier output plus slope compensation, input to PWM comparator
                                        // 00010 = Input to slope compensation circuitry
                                        // 00011 = Band gap reference
                                        // 00100 = Output voltage reference
                                        // 00101 = Output voltage after internal differential amplifier
                                        // 00110 = Unimplemented
                                        // 00111 = voltage proportional to the internal temperature
                                        // 01000 = Internal ground for current sense circuitry
                                        // 01001 = Output overvoltage comparator reference
                                        // 01010 = Output under voltage comparator reference
                                        // 01011 = Error amplifier output
                                        // 01100 = For a multi-phase SLAVE, error amplifier signal received from MASTER
                                        // 01101 = For multi-phase SLAVE, error signal received from MASTER with gain
                                        // 01110 = Vin Divided down by 1/5
                                        // 01111 = DC Inductor Valley Current
                                        // 11101 = Over Current Reference
                                        // Everything else unimplemented.
                                        // If ATSTCON<BNCHEN> is set, the device is in Bench Test Mode and will put one of these out on GPA0. 
                                        // Be sure to add the Buffer Offset to the measured signal which can be read from memory location 0x2087.
    // </editor-fold>
    PE1         = 0x80;                 // Analog Peripheral Enable 1 Control Register. Enable diode emulation. This is a *MUST* or the battery could destroy the sync FET.
    // <editor-fold defaultstate="collapsed" desc="PE1 Register Bit Description">
                                        // bit 7 (DECON) is Diode Emulation Mode. 1(Enabled), 0(Disabled).  Enabled the synchronous rectification will disable when indutor current reaches zero.
                                        // bit 6 (DBRSTR) is High Side Drive Strength Configuration Bit.  1 (1A Sink/Source), 0 (2A Sink/Source)
                                        // bit 5 (HDLYBY) is High Side Dead Time Bypass bit
                                        // bit 4 (LDLYBY) is Low Side Dead Time Bypass bit
                                        // bit 3 (PDEN) is -V_sen Weak Pull Down Enable bit
                                        // bit 2 (PUEN) is +V_sen Weak Pull Up Enable bit
                                        // bit 1 (UVTEE) is Output Undervoltage Accelerator Enable bit
                                        // bit 0 (OVTEE) is Output Over Voltage Accelerator Enable bit
    // </editor-fold>
    DEADCON     = 0x9B;                 // Driver Dead time Control Register.  47nS high-side dead time, 48nS low-side dead time 
    //// <editor-fold defaultstate="collapsed" desc="DEADCON Bit Descriptions">
                                        // bit 7-4 HDLY<3:0> High-Side Dead Tiem Configuration Bits
                                        // 0000 = 11nsec delay
                                        // 0001 = 15nsec delay
                                        // 0010 = 19nsec delay
                                        // 0011 = 23nsec delay
                                        // 0100 = 27nsec delay
                                        // 0101 = 31nsec delay
                                        // 0110 = 35nsec delay
                                        // 0111 = 39nsec delay
                                        // 1000 = 43nsec delay
                                        // 1001 = 47nsec delay
                                        // 1010 = 51nsec delay
                                        // 1011 = 55nsec delay
                                        // 1100 = 59nsec delay
                                        // 1101 = 63nsec delay
                                        // 1110 = 67nsec delay
                                        // 1111 = 71nsec delay
                                        // bit 3-0 LDLY<3:0> Low-Side Dead Tiem Configuration Bits
                                        // 0000 = 4nsec delay
                                        // 0001 = 8nsec delay
                                        // 0010 = 12nsec delay
                                        // 0011 = 16nsec delay
                                        // 0100 = 20nsec delay
                                        // 0101 = 24nsec delay
                                        // 0110 = 28nsec delay
                                        // 0111 = 32nsec delay
                                        // 1000 = 36nsec delay
                                        // 1001 = 40nsec delay
                                        // 1010 = 44nsec delay
                                        // 1011 = 48nsec delay
                                        // 1100 = 52nsec delay
                                        // 1101 = 56nsec delay
                                        // 1110 = 60nsec delay
                                        // 1111 = 64nsec delay
    // </editor-fold>
    ABECON      = 0x0D;                 // Analog Block Enable Control Register.  Enable Signal Chain, Internal Temp Sensor, Current Measurement.
    // <editor-fold defaultstate="collapsed" desc="ABECON Register Bit Description">
                                        // bit 7 (OVDCEN) Output over voltage DAC Control bit
                                        // bit 6 (UVDCEN)Output under voltage DAC Control bit
                                        // bit 5 (MEASEN) Relative Efficiency Measurement Control bit
                                        // bit 4 (SLCPBY) Slope compensation bypass control bit. (1)Slope Comp Disabled. (0)Slope Comp Enabled.
                                        // bit 3 (CRTMEN) Current Measurement circuitry control bit
                                        // bit 2 (TMPSEN) Internal temperature sensor control bit
                                        // bit 1 (RECIREN) Relative efficiency circuit control bit
                                        // bit 0 (PATHEN) Signal chain circuitry control bit
    // </editor-fold>
    VZCCON      = 0x60;                 // Voltage for zero current register (3.28mV/bit steps). 0x60 = -104.96mV. 
    //// <editor-fold defaultstate="collapsed" desc="VZCCON Register Bit Description">
                                        // bit7-0 VZC<7:0> Voltage for Zero Current Setting Bits
                                        // 00000000 = -420mV Offset
                                        // 00000001 = -416.72mV Offset
                                        // ......
                                        // 10000000 = 0mV Offset
                                        // ......
                                        // 11111110 = +413.12mV Offset
                                        // 11111111 = +416.40mV Offset
    // </editor-fold>
    CMPZCON     = 0x0F;                 // Compensation Setting Control Register. Zero Set to 1.5kHz. Gain Set to 0dB.
    //<editor-fold defaultstate="collapsed" desc="CMPZCON Register Bit Description">
    
                                        // bit 7-4 CMPZF<3:0> Compensation Zero Frequency Setting Bits
                                        // 0000 = 1500Hz
                                        // 0001 = 1850Hz
                                        // 0010 = 2300Hz
                                        // 0011 = 2840Hz
                                        // 0100 = 3460Hz
                                        // 0101 = 4300Hz
                                        // 0110 = 5300Hz
                                        // 0111 = 6630Hz
                                        // 1000 = 8380Hz
                                        // 1001 = 9950Hz
                                        // 1010 = 12200Hz
                                        // 1011 = 14400Hz
                                        // 1100 = 18700Hz
                                        // 1101 = 23000Hz
                                        // 1110 = 28400Hz
                                        // 1111 = 35300Hz
                                        // bit 3-0 CMPZG<3:0> Compensation Gain Setting Bits
                                        // 0000 = 36.15dB
                                        // 0001 = 33.75dB
                                        // 0010 = 30.68dB
                                        // 0011 = 28.43dB
                                        // 0100 = 26.10dB
                                        // 0101 = 23.81dB
                                        // 0110 = 21.44dB
                                        // 0111 = 19.10dB
                                        // 1000 = 16.78dB
                                        // 1001 = 14.32dB
                                        // 1010 = 12.04dB
                                        // 1011 = 9.54dB
                                        // 1100 = 7.23dB
                                        // 1101 = 4.61dB
                                        // 1110 = 2.28dB
                                        // 1111 = 0dB
    //</editor-fold>
    SLPCRCON    = 0xF2;                 // Slope compensation ramp control register.  Set to 1.250 Vpk/Vpk when at 50% duty cycle. DV/Dt of 2.
                                        // Set the SLPS above to 2 as per the datasheet. Formula is <3:0> = (Fsw / 100,000) -1 
    // <editor-fold defaultstate="collapsed" desc="SLPCRCON Register Bit Description">
                                        // bit 7-4 SLPG<3:0> Slope Compensation Amplitude Configuration Bits
                                        // 0000 = 0.017 Vpk-pk measured from 50% duty cycle waveform
                                        // 0001 = 0.022 Vpk-pk measured from 50% duty cycle waveform
                                        // 0010 = 0.030 Vpk-pk measured from 50% duty cycle waveform
                                        // 0011 = 0.040 Vpk-pk measured from 50% duty cycle waveform
                                        // 0100 = 0.053 Vpk-pk measured from 50% duty cycle waveform
                                        // 0101 = 0.070 Vpk-pk measured from 50% duty cycle waveform
                                        // 0110 = 0.094 Vpk-pk measured from 50% duty cycle waveform
                                        // 0111 = 0.125 Vpk-pk measured from 50% duty cycle waveform
                                        // 1000 = 0.170 Vpk-pk measured from 50% duty cycle waveform
                                        // 1001 = 0.220 Vpk-pk measured from 50% duty cycle waveform
                                        // 1010 = 0.300 Vpk-pk measured from 50% duty cycle waveform
                                        // 1011 = 0.400 Vpk-pk measured from 50% duty cycle waveform
                                        // 1100 = 0.530 Vpk-pk measured from 50% duty cycle waveform
                                        // 1101 = 0.700 Vpk-pk measured from 50% duty cycle waveform
                                        // 1110 = 0.940 Vpk-pk measured from 50% duty cycle waveform
                                        // 1111 = 1.250 Vpk-pk measured from 50% duty cycle waveform
                                        // bit 3-0 SLPS<3:0> Slope Compensation Delta-V/Delta-t Configuration Bits
                                        // Set this byte proportional to the switching frequency. n = (Fsw/100000)-1
    // </editor-fold>
    SLVGNCON    = 0x00;                 // Master Error Signal Input Gain Control Register. Configured for the SLAVE device. Not used in this design so set to zero.
    // <editor-fold defaultstate="collapsed" desc="SLVGNCON Register Bit Description">
                                        // bit 7-5 Unimplemented, Read as 0
                                        // 00000 = -3.3dB
                                        // 00001 = -3.1dB
                                        // 00010 = -2.9dB
                                        // 00011 = -2.7dB
                                        // 00100 = -2.5dB
                                        // 00101 = -2.3dB
                                        // 00110 = -2.1dB
                                        // 00111 = -1.9dB
                                        // 01000 = -1.7dB
                                        // 01001 = -1.4dB
                                        // 01010 = -1.2dB
                                        // 01011 = -1.0dB
                                        // 01100 = -0.8dB
                                        // 01101 = -0.6dB
                                        // 01110 = -0.4dB
                                        // 01111 = -0.2dB
                                        // 10000 = 0.0dB
                                        // 10001 = 0.2dB
                                        // 10010 = 0.4dB
                                        // 10011 = 0.7dB
                                        // 10100 = 0.9dB
                                        // 10101 = 1.1dB
                                        // 10110 = 1.3dB
                                        // 10111 = 1.5dB
                                        // 11000 = 1.7dB
                                        // 11001 = 1.9dB
                                        // 11010 = 2.1dB
                                        // 11011 = 2.3dB
                                        // 11100 = 2.6dB
                                        // 11101 = 2.8dB
                                        // 11110 = 3.0dB
                                        // 11111 = 3.2dB
                                        // The SLVGNCON register is configured in the multi-phase SLAVE device, not the MASTER.
    
    // </editor-fold>
    CSGSCON     = 0x0d;                 // Current Sense AC Gain Control Register.  AC gain = 19dB 
    // <editor-fold defaultstate="collapsed" desc="CSGSCON Register Bit Description">
                                        // bit 3-0 CSGS<3:0> Current Sense AC Gain Setting Bits
                                        // 0000 = 0dB
                                        // 0001 = 1.0dB
                                        // 0010 = 2.5dB
                                        // 0011 = 4.0dB
                                        // 0100 = 5.5dB
                                        // 0101 = 7.0dB
                                        // 0110 = 8.5dB
                                        // 0111 = 10.0dB
                                        // 1000 = 11.5dB
                                        // 1001 = 13.0dB
                                        // 1010 = 14.5dB
                                        // 1011 = 16.0dB
                                        // 1100 = 17.5dB
                                        // 1101 = 19.0dB
                                        // 1110 = 20.5dB
                                        // 1111 = 22.0dB
    // </editor-fold>
    CSDGCON     = 0x00;                 // Current Sense DC Gain Control Register. DC gain set to 19.5dB, It's not used in control loop, but read by the ADC.
    // <editor-fold defaultstate="collapsed" desc="CSDGCON Register Bit Description">
                                        // bit7 CSDGEN Current Sense DC Gain Enable Bit: 1 = DC Gain current sense signal used in control loop. 0 = signal only read by ADC.
                                        // bit6-4 Unimplemented and read as 0
                                        // bit3 Reserved
                                        // bit2-0 CSDG<2:0> Current Sense DC Gain Setting Bits
                                        // 000 = 19.5dB
                                        // 001 = 21.8dB
                                        // 010 = 24.1dB
                                        // 011 = 26.3dB
                                        // 100 = 28.6dB
                                        // 101 = 30.9dB
                                        // 110 = 33.2dB
                                        // 111 = 35.7dB    
    // </editor-fold>
    OCCON       = 0x82;                 // Output Overcurrent Control Register. //**BRYAN** Go figure out what to set this to for the evaluation board.
    // <editor-fold defaultstate="collapsed" desc="OCCON Register Bit Description">
                                        // bit 7 OCEN Output Overcurrent DAC Control Bit
                                        // bit6-5 OCLEB<1:0> Leading Edge Blanking
                                        // 00 = 114ns blanking
                                        // 01 = 213ns blanking
                                        // 10 = 400ns blanking
                                        // 11 = 780ns blanking
                                        // bit4-0 OOC<4:0> Output Overcurrent Configuration bits
                                        // 00000 = 160mV drop
                                        // 00001 = 175mV drop
                                        // 00010 = 190mV drop
                                        // 00011 = 205mV drop
                                        // 00100 = 220mV drop
                                        // 00101 = 235mV drop
                                        // 00110 = 250mV drop
                                        // 00111 = 265mV drop
                                        // 01000 = 280mV drop
                                        // 01001 = 295mV drop
                                        // 01010 = 310mV drop
                                        // 01011 = 325mV drop
                                        // 01100 = 340mV drop
                                        // 01101 = 355mV drop
                                        // 01110 = 370mV drop
                                        // 01111 = 385mV drop
                                        // 10000 = 400mV drop
                                        // 10001 = 415mV drop
                                        // 10010 = 430mV drop
                                        // 10011 = 445mV drop
                                        // 10100 = 460mV drop
                                        // 10101 = 475mV drop
                                        // 10110 = 490mV drop
                                        // 10111 = 505mV drop
                                        // 11000 = 520mV drop
                                        // 11001 = 535mV drop
                                        // 11010 = 550mV drop
                                        // 11011 = 565mV drop
                                        // 11100 = 580mV drop
                                        // 11101 = 595mV drop
                                        // 11110 = 610mV drop
                                        // 11111 = 625mV drop
    // </editor-fold>
    OVCCON      = 0x00;                 // Output Voltage Set Point Coarse Control Register. Set to 0 to begin. 
    //<editor-fold defaultstate="collapsed" desc="OVCCON Register Description">
                                        // This is the output voltage error amplifier reference DAC. Coarse setting is 8bit in 15.8mV increments.
    //</editor-fold>
    OVFCON      = 0x80;                 // Output Voltage Set Point Fine Control Register.  Enable DAC and set to 0 to begin. 
    //<editor-fold defaultstate="collapsed" desc="OVFCON Register Description">
                                        // The fine setting is 5bit in 0.82mV steps. Bit 7 enables the DAC.
    // </editor-fold>
    
    //**BRYAN** Add some code for taking target voltages and addressing this setting from the configuration space if it makes sense for OUVCON and OOVCON. 
    //**BRYAN** Since these interrupts are disabled this is going to be commented out below for now.  Adding for reference code thoroughness though.
    //  OUVCON  = 0x00;                     // Output under voltage detect level control register. 
    // <editor-fold defaultstate="collapsed" desc="OUVCON Register Bit Description">
                                        // bit 7-0 OUV<7:0> Output Under Voltage Detect Level Configuration Bits
                                        // Equation is OUV<7:0> = (V_out_uv_detect_level)/15mV
    // </editor-fold>
    //  OOVCON  = 0xff;                     // Output over voltage detect level control register.
    // <editor-fold defaultstate="collapsed" desc="OOVCON Register Bit Description">
                                        // bit 7-0 OOV<7:0> Output Overvoltage Detect Level Configuration bits
                                        // Equation is OOV<7:0> = V_out_ov_detect_level / 15mV
    // </editor-fold>

    vh          = 0;                    // DAC High Byte 
    vl          = 0;                    // DAC Low Byte 

/* Enable input Under Voltage Lockout threshold */
    VINLVL = 0x8B;  // Input Under Voltage Lockout Control Register. Enabled and Set to 6V.
    // <editor-fold defaultstate="collapsed" desc="VINLVL Register Bit Description">
                                        // bit 7 UVLOEN Under Voltage Lockout DAC Control bit, 1=Enabled, 0=Disabled
                                        // bit 6 Unimplemented = 0
                                        // bit 5-0 UVLO<5:0> Under voltage Lockout Configuration Bits = 26.5*ln(UVLO_SET-POINT/4)
    // </editor-fold>
}

void inc_iout(short increment)      // Topology Specific function - Increase Current (Combined)
{
  int i;
  for(i=0;i<increment;i++)			// Loop through the increment routine (if fast_ramp requires)
  {
    vl++; 						// Increment fine register
    if (vl > IOUT_LSB_ROLL) 		// If we are over the top of the fine register
	{
		if (vh < cs.ovcfcon_max) 
        {	// and the coarse register is not maxed out
            vh ++;					// increment coarse register
			vl = 0;					// and reset fine register to zero
        }
        else 
        {
            vh = cs.ovcfcon_max;	// Set to max
            vl = IOUT_LSB_ROLL;		// We went one increment beyond IOUT_LSB_ROLL, so reset to max.
        }
    }
  }
    OVFCON = (unsigned short)(0x80 | vl);             //Load the DAC with the new values
    OVCCON = vh;
}

short dec_iout(short decrement)	// Topology Specific function - Decrease Current (Combined)
{
    int i;
    TEST_PIN1 = 0;
    TEST_PIN1 = 1;
    TEST_PIN1 = 0;
    if ((vh == 0) && (vl == 0))     // Are we already at the bottom (off)
    {
        return(0);
    }
	for (i = 0; i<decrement; i++)
	{
        if(vl > 0)					// Must check for zero because 'vl' can roll to FF if not.
		{	
            vl--;					// Reduce fine value if we are not already at the bottom of the range
        }
		/*else*/ if (vl == 0)        				// If fine == 0
		{
//			if (vh > cs.ovcfcon_min) // and corse > the minimum
            if (vh > 0) // and corse > the minimum
			{
				vh--;				 // Then decrease course and reset fine to top of range
				vl = IOUT_LSB_ROLL;	 //
			}
			else
			{
//				vh = cs.ovcfcon_min; // Else, we are at the bottom (vl = vf = 0)
                vh = 0;
                return(0);			 // Return a fault code that we can't decrement any more
			}
		}
	}
    OVFCON = (unsigned char)(0x80 | vl);	// Update the setpoint registers
    OVCCON = vh;		//
	return(1);			// Success
}

void zero_iout(void)
{
	/* Output current set to zero */
	vl = 0;
	//vh = cs.ovcfcon_min; //This setting isn't actually zero on this board because it's a Voltage Mode Output.
	vh = 0;  //Let's have a true zero setting just to be safe.
    OVCCON = 0;
	OVFCON = (unsigned char)(0x80 | vl);
}

void check_button(void)
{
    extern unsigned short button_cnt;
    extern bit start;
    extern bit shutdown;
    
    button_cnt++;

    switch (button_state)
    {        
        case NOT_PRESSED:
            if (!BUTTON_PRESS_n)
            {
                button_state = DEBOUNCING;           
            }
            button_cnt = 0;
            break;
        case DEBOUNCING:
            if (BUTTON_PRESS_n)
            {
                button_state = NOT_PRESSED;
                button_cnt = 0;
            }
            else if (button_cnt > 100)
            {
                button_state = PRESSED;
                button_cnt = 0;
            }
            break;
        case PRESSED:
            if (BUTTON_PRESS_n)
            {
                button_state = NOT_PRESSED;
                button_cnt = 0;
            }
            else if (button_cnt > 1000)
            {
                button_state = COOL_DOWN;
                button_cnt = 0;
                if (cd.charger_state == CHARGER_OFF)
                {
                    start = 1;
                    cd.status.word = 0x0000; // Clear the shutdown cause
                }
                else 
                {
                    shutdown = 1;
                }
            }
            break;
        case COOL_DOWN:
            if (!BUTTON_PRESS_n)
            {
                button_cnt = 0;
            }
            else if (button_cnt > 1000)
            {
                button_state = NOT_PRESSED;
            }
            break;                 
    }        
}

void hardware_custom_a(void)
{

}

void disable_charger(void)
{
    /* PWM off */
	ATSTCONbits.DRVDIS = 1;

	/* Adjust UVLO threshold */
    
    /* Disable UVLO interrupt */
    PIE2bits.VINIE = 0;
    
    /* Set output current to minimum */
    zero_iout();
    
    disconnect_battery(); //no disconnect switch on this board.
}

void enable_charger(void) //%%BRYAN%% need to add the output overcurrent and overvoltage interrupt enables
{
    unsigned int vbat_prep = 0;
    connect_battery(); //no disconnect switch on this board.
    
    /* Set output current to minimum */
    zero_iout();

	/* Adjust UVLO threshold */
    //VINLVL = 0x80 | cs.uvlo_threshold_off;
    
	/* Enable UVLO interrupt */
	PIE2bits.VINIE = 1;
    
    /* The MCP19118_FOUR_SWITCH_BUCK_BOOST_CHARGER board has the error amplifier configured for outputing voltage               */
    /* instead of current.  So in this enable_charger() routine we're going to take advantage of it being called first          */
    /* in the state machine to read the ADC of Vbat and optimize the starting settings for OVCCON.                              */
    /* If we don't do this, then many seconds or even minutes will go by while the machine slowly iterates inc_iout             */
    /* in order to get the set point for the OUTPUT VOLTAGE higher than the battery voltage.                                    */


    vbat_prep = cd.adc_vbat + cd.adc_vbat + cd.adc_vbat + cd.adc_vbat + cd.adc_vbat;
    vbat_prep = (unsigned int)(vbat_prep >> 6); //This is basically Vref * cd.adc_vbat * 1024 / (16 * 4096) where Vref is 5.
                                                //the OVCCON register is stuff with a target value in ~15.8mV increments.
                                                //So to make the math pretty easy with simple functions we multiply the desired
                                                //target value by 1024/16 = 64.  The reciprocal is 15.625mV.
                                                //That is.  If we multiply by 1024/16, then we'll in effect be dividing the
                                                //measured ADC battery voltage by 15.625mV to get a DAC count.  For this to work
                                                //the resistor divider used for feedback and for ADC scaling MUST BE THE SAME!
    if (vbat_prep >= 0x08) //The compiler can give you some weird answers sometimes, so avoid going negative below.
    {
        vh = (vbat_prep-0x08); //Give yourself ~100mV of margin to get under any component/process/math variations.
    }

	ATSTCONbits.DRVDIS = 0;     //Turn the Gate Drivers on
    
}

#ifdef ENABLE_FIXED_CONFIG
#include "MultiChemCharger_Values.h"
#endif

void init_cal_registers(void) //%%BRYAN%% Verify and make use of the read_flash function instead of this repetition
{
    /* Calibration Word 1 */
    PMADRH = 0x20;                      // Location of the DOV<3:0> = CALWD1<11:8> = DOVCAL bits 
    PMADRL = 0x80;                      // and the FCAL<6:0> = CALWD1<6:0> = OSCCAL bits 
    PMCON1bits.CALSEL = 1;
    PMCON1bits.RD = 1;
    asm("nop");
    asm("nop");
    DOVCAL = PMDATH;                    // Load Device Calibration from memory (Output Voltage sense differential amp adjust)
    OSCCAL = PMDATL;                    // Load OSCCAL value from memory (Oscillator tune bits)

    /* Calibration Word 2 */
    PMADRH = 0x20;                      // Location of the VRO<3:0> = CALWD2<11:8> = VROCAL bits
    PMADRL = 0x81;                      // and the BGR<3:0> = CALWD2<3:0> = BGRCAL bits
    PMCON1bits.CALSEL = 1;
    PMCON1bits.RD = 1;
    asm("nop");
    asm("nop");
    VROCAL = PMDATH;                     // offset of the buffer amplifier of the output voltage regulation reference set point calibration
    BGRCAL = PMDATL;                    // Internal band-gap reference calibration

    /* Calibration Word 3 */
    PMADRH = 0x20;                      // Location of the TTA<3:0> = CALWD3<11:8> = TTACAL bits
    PMADRL = 0x82;                      // Locatino of the ZRO<3:0> = CALWD<3:0> = ZROCAL bits
    PMCON1bits.CALSEL = 1;
    PMCON1bits.RD = 1;
    asm("nop");
    asm("nop");
    TTACAL = PMDATH;                    // Over-temp shutdown threshold calibration bits
    ZROCAL = PMDATL;                    // Error amplifier offset calibration
   
    
    /* Calibration Words 4 - 12 */
}

/* This ENABLE_GUI_CONFIG ifdef is meant to save a little code and prevent flash writes when this device function */
/* was locked down. While it adds the ability to include MultiChemCharger_Values.h it locks out the ability for   */
/* the GUI to change configuration settings. Be aware of this.  If you are testing compiling with the header file */
/* and still want to be able to use the GUI then be sure to set the ENABLE_GUI_CONFIG and ENABLE_FIXED_CONFIG in  */
/* the header file.   */

#ifdef ENABLE_GUI_CONFIG   

/* This function writes 8 bytes (4 flash words) at a time to flash. */
/* The incoming address must be 8-byte aligned.                     */

unsigned short write_flash(unsigned short addr, unsigned short counter) 
{
    unsigned char b = 0;
	if (addr < CAL_BASE_ADDR)           // Invalid address.  Should return a failure result
	{
        return 0x00; 
    }
    if (counter == 8)				// Write data if 8 bytes (4-words) have been received
    {
        while (b < 8) 
        {
            PMADRH = addr >> 8;
            PMADRL = addr;
            PMDATL = flash_write_buf[(unsigned char)(b & 0x07)];
            b++;
            PMDATH = flash_write_buf[(unsigned char)(b & 0x07)];
            b++;
            PMCON1bits.CALSEL = 0;
            PMCON1bits.WREN = 1;
            PMCON2 = 0x55;
            PMCON2 = 0xAA;
            PMCON1bits.WR = 1;
            NOP();
            NOP();
            PMCON1bits.WREN = 0;
            addr++;
        }
        return 0x04;
    }
    return 0x00;
}
#endif

unsigned short read_flash(unsigned short addr)
{
    unsigned short a;

    PMADRH = addr >> 8;
    PMADRL = addr;
    PMCON1bits.CALSEL = 0;
    PMCON1bits.RD = 1;
    asm("nop");
    asm("nop");
    a = (unsigned short)(PMDATH << 8);
    a |= PMDATL;
    return a;
}

unsigned char check_for_uvlo(void)
{
    if ((cd.adc_vin < cs.uvlo_adc_threshold_off) || (PIR2bits.VINIF == 1))
	{
		/* Attempt a reset */
		PIR2bits.VINIF = 0;
		return 1;
	}
    return 0;
}

#ifdef ENABLE_BENCH_TEST_OUTPUT_PIN
void update_buffcon(void) /* for parts with a copy of an analog value on a pin. */
{
    //The Bench Test Output Pin is set here.  Update the analog mux source here.
    //BUFFCON = cd.buffcon_reg & 0x1F;	
    // <editor-fold defaultstate="collapsed" desc="BUFFCON Register Bit Description">
                                        // bit 7-5 MLTPH<2:0> System Configuration Bits
                                        // 000 = Device set as stand-alone unit
                                        // 001 = Device set as multiple output MASTER
                                        // 010 = Device set as multiple output SLAVE
                                        // 011 = Device set as multi-phase MASTER
                                        // 111 = Device set as multi-phase SLAVE
                                        // bit 4-0 ASEL<4:0> Multiplexter Output Control Bit
                                        // 00000 = Voltage proportional to current in the inductor
                                        // 00001 = Error amplifier output plus slope compensation, input to PWM comparator
                                        // 00010 = Input to slope compensation circuitry
                                        // 00011 = Band gap reference
                                        // 00100 = Output voltage reference
                                        // 00101 = Output voltage after internal differential amplifier
                                        // 00110 = Unimplemented
                                        // 00111 = voltage proportional to the internal temperature
                                        // 01000 = Internal ground for current sense circuitry
                                        // 01001 = Output overvoltage comparator reference
                                        // 01010 = Output under voltage comparator reference
                                        // 01011 = Error amplifier output
                                        // 01100 = For a multi-phase SLAVE, error amplifier signal received from MASTER
                                        // 01101 = For multi-phase SLAVE, error signal received from MASTER with gain
                                        // 01110 = Vin Divided down by 1/5
                                        // 01111 = DC Inductor Valley Current
                                        // 11101 = Over Current Reference
                                        // Everything else unimplemented.
                                        // If ATSTCON<BNCHEN> is set, the device is in Bench Test Mode and will put one of these out on GPA0. 
                                        // Be sure to add the Buffer Offset to the measured signal which can be read from memory location 0x2087.
    // </editor-fold>
    cd.user_interface_mode = 3;		// Tell the GUI we are in bench-test mode

}
#endif

#ifdef ENABLE_STATUS_LEDS
/************************************************************************************/
/* This board only has one LED so we are going to set up a Truth table as follows:  */
/* State    LED #1:                                                                 */
/* Charging ON                                                                      */
/* Complete OFF                                                                     */
/* Fault    OFF                                                                     */
/* Stopped  OFF                                                                     */
/* If you want to modify the board there is example code commented out at the top of*/
/* this file to start with for a broad use of up to four LEDs.                      */
/************************************************************************************/
void update_status_leds(void)
{
    // LED #1
    if (LED_TMR-- == 0) 
    {
        if (bLED_On) 
        {
            bLED_On = 0;
            LED1_PIN = 1;
            LED_TMR = LED_OffTime;
        } 
        else 
        {
            bLED_On = 1;
            LED1_PIN = 0;
            LED_TMR = LED_OnTime;
        }
    }
}

#endif // end Enable Status LEDS

void connect_battery(void)
{
    /* Connect Battery Switch */
    //There is no hardware for this
}

void disconnect_battery(void)
{
    /* PWM off: We don't want the charger running when we disconnect the battery or bad things will happen */
	//ATSTCONbits.DRVDIS = 1;
    
    /* Disconnect Battery Switch */
    //There is no hardware for this
}

 #endif	// end of MCP19118 Configuration
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="MCP19119_4A_Power_Tool_Charger">
#ifdef MCP19119_4A_Power_Tool_Charger

char vh, vl;
extern struct charger_settings_t cs;	// Get access to Charger_Settings structure
extern struct charger_data_t cd;		// Get access to Charger_Data structure

void interrupt isr()
{
    if (PIR2bits.VINIF == 1) 	// UVLO flag set?
	{
        /* Disable the interrupt */
        PIE2bits.VINIE = 0;

        /* PWM off */
        ATSTCONbits.DRVDIS = 1;

        /* Output FET off */
        VBAT_EN = 0;
		
		// log the reason 
		cd.status.vinuvlo = 1;
    }
}

 
void init_hardware()	/* MCP19119 Hardware specific configuration. */
{
    OSCTUNE = 0x00;                     // Oscillator Tuning Register. Set oscillator to Calibrated Center Frequency
    // <editor-fold defaultstate="collapsed" desc="OSCTUNE Register bit Description">
                                        //bit 7-5 Unimplemented
                                        //bit 4-0 TUN<4:0> Frequency Tuning Bits
                                        //b01111 = Maximum Frequency.
                                        //b01110 thru b00001 = monotonic ramp in between maximum and center.
                                        //b00000 = Center frequency.  Oscillator Module is running at the calibrated frequency.
                                        //b11111 thru b10001 = monotonic ramp in between center and minimum.
                                        //b10000 = Minimum Frequency
    // </editor-fold>
    OPTION_REG = 0x00;                  // Prescaler (bits 2:0) set to 000 for 1:2 TMR0 Rate, 256usec per timer overflow.  
    // <editor-fold defaultstate="collapsed" desc="OPTION_REG Register Description">
                                        // bit 7 RAPU# Port GPx Pull_up Enable bit, 1=Disabled, 0=Enabled
                                        // bit 6 INTEDG Interrupt Edge Select bit, 0=Interrupt on rising edge of INT pin, 1=falling edge
                                        // bit 5 T0CS TMR0 Clock Source Select bit, 1=Transition on T0CKI pin, 0=Internal instruction cycle clock
                                        // bit 4 T0SE TMR0 Source Edge Select bit, 1=Increment on high to low transition on T0CKI pin, 0=low to high
                                        // bit 3 PSA Prescaler Assignment bit, 1=Prescalar is assigned to WDT, 0=Prescaler is assigned to Timer0 module
                                        // bit 2-0 PS<2:0> Prescalar Rate Select Bits:
                                        //  000 - 1:2 TMR0 Rate 1:1 WDT Rate
                                        //  001 - 1:4 TMR0 Rate 1:2 WDT Rate
                                        //  010 - 1:8 TMR0 Rate 1:4 WDT Rate
                                        //  011 - 1:16 TMR0 Rate 1:8 WDT Rate
                                        //  100 - 1:32 TMR0 Rate 1:16 WDT Rate
                                        //  101 - 1:64 TMR0 Rate 1:32 WDT Rate
                                        //  110 - 1:128 TMR0 Rate 1:64 WDT Rate
                                        //  111 - 1:256 TMR0 Rate 1:128 WDT Rate
    // </editor-fold>
    PIR1    = 0;                        // Peripheral Interrupt Flag Register 1. All Interrupts Cleared.
    // <editor-fold defaultstate="collapsed" desc="PIR1 Register Bit Description">
                                        // bit 7 Unimplemented, Read as 0
                                        // bit 6 ADIF ADC Interrupt Flag bit, 1=ADC Conversion Complete, 0=Conversion not complete or not started
                                        // bit 5 BCLIF MSSP Bus Collision Interrupt Flag bit, 1=Interrupt is pending, 0=Interrupt is not pending
                                        // bit 4 SSPIF Synchronous Serial Port (MSSP) Interrupt Flag bit, 1=Interrupt is pending, 0=Interrupt not pending.
                                        // bit 3-2 Unimplemented, Read as 0
                                        // bit 1 TMR2IF Timer2 to PR2 Match Interrupt Flag, 1=Timer2 to PR2 match occurred (must be cleared in SW), 0=Match did not occur
                                        // bit 0 TMR1IF Timer1 Interrupt Flag.  1=Timer1 rolled over (must be cleared in SW). 0=Timer 1 has not rolled over.
    // </editor-fold>
    PIE1    = 0;                        // Peripheral Interrupt Enable Register 1. All Interrupts Disabled.
    // <editor-fold defaultstate="collapsed" desc="PIE1 Register Bit Description">
                                        // bit 7 Unimplemented. Read as 0.
                                        // bit 6 ADC Interrupt Enable bit, 1=Enables ADC Interrupt, 0=Disables
                                        // bit 5 BCLIE MSSP Bus Collision Interrupt Enable bit, 1=Enables the MSSP bus Collision Interrupt, 0=Disables
                                        // bit 4 SSPIE synchronous Serial Port (MSSP) Interrupt Enable bit, 1=Enables MSSP Bus Collision Interrupt, 0=Disables
                                        // bit 3-2 Unimplemented
                                        // bit 1 TMR2IE Timer2 Interrupt Enable, 1=Enables the Timer2 Interrupt, 0=Disables
                                        // bit 0 TMR1IE Timer1 Interrupt Enable, 1=Enables the Timer1 Interrupt, 0=Disables
    // </editor-fold>
    PIR2    = 0;                        // Peripheral Interrupt Flag Register 2.  All interrupts Cleared.
    // <editor-fold defaultstate="collapsed" desc="PIR2 Register Bit Description">
                                        // bit 7 UVIF Output undervoltage error interrupt flag bit, 1=Output Undervoltage error has occurred, 0=Has not occured
                                        // bit 6 Unimplemented, Read as 0.
                                        // bit 5 OCIF Output overcurrent error interrupt flag bit, 1=Output OC error has occurred, 0=Has not occured
                                        // bit 4 OVIF Output overvoltage error interrupt flag bit, 1=Output OV error has occurred, 0=Has not occured
                                        // bit 3-2 Unimplemented, Read as 0
                                        // bit 1 VINIF Vin Status bit, 1=Vin is below acceptable level, 0=Vin is at acceptable level
                                        // bit 0 Unimplemented, Read as 0
    // </editor-fold>
    PIE2    = 0;                        // Peripheral Interrupt Enable Register 2. All Interrupts Disabled.
    // <editor-fold defaultstate="collapsed" desc="PIE2 Register Bit Description">
                                        // bit 7 UVIE Output Under voltage Interrupt enable bit, 1=Enables teh UV interrupt, 0=Disables
                                        // bit 6 Unimplemented, Read as 0
                                        // bit 5 OCIE Output Overcurrent Interrupt enable bit, 1=Enables the OC Interrupt, 0=Disables
                                        // bit 4 OVIE Output Overvoltage Interrupt enable bit, 1=Enables the OV Interrupt, 0=Disables
                                        // bit 3-2 Unimplemented, Read as 0
                                        // bit 1 VINIE V_in UVLO Interrupt Enable, 1=Enables teh V_IN UVLO Interrupt, 0=Disables
                                        // bit 0 Unimplemented, Read as 0
    // </editor-fold>
    INTCON  = 0xc0;                     // Global + Peripheral interrupts.  Enable all Unmasked Interrupts and Unmasked Peripheral Interrupts.  All the rest are disabled.
    // <editor-fold defaultstate="collapsed" desc="INTCON Register Bit Description">
                                        // bit 7 GIE Global Interrupt Enable bit, 1=Enables all unmasked interrupts, 0=Disables
                                        // bit 6 PEIE Peripheral Interrupt Enable bit, 1=Enables all unmasked peripheral interrupts, 0=Disables
                                        // bit 5 TMR0 Overflow Interrupt Enable bit, 1=Enables the TMR0 Interrupt, 0=Disables
                                        // bit 4 INTE The INT External Interrupt Enable bit, 1=Enables the INT external interrupt. 0=Disables
                                        // bit 3 IOCE Interrupt on Change Enable bit, 1=Enables the interrupt-on-change interrupt, 0=Disables
                                        // bit 2 T0IF TMR0 Overflow Interrupt Flag big, 1=TMR0 register has overflowed (must be cleared in software), 0=TMR0 did not overflow
                                        // bit 1 INTF External Interrupt Flag bit. 1=External Interrupt Occurred (must be cleared in software), 0=External Interrupt did not occur
                                        // bit 0 IOCF Interrupt on Change Interrupt Flag bit, 1=When at least one of the IOC pins changed state, 0=None have changed state
    // </editor-fold>
    IOCA = 0x00;                        // Interrupt on Change PortGPA Register.  Disabled on all Port A pins.  Setting a bit also requires bit 3 of INTCON be set to enable interrupt on change.
    // <editor-fold defaultstate="collapsed" desc="IOCA Register Bit Description">
                                        // bit 7-6 Interrupt on Change PortGPA Register bits, 1=enabled, 0=disabled
                                        // bit 5 Interupt on Change PortGPA Register bits, 1=enabled, 0=disabled
                                        // bit 4-0 Interrupt on Change PortGPA Register bits, 1=enabled, 0=disabled
    // </editor-fold>
    IOCB = 0x00;                        // Interrupt on Change PortGPB Register.  Disabled on all Port B pins.  Setting a bit also requires bit 3 of INTCON be set to enable interrupt on change.
    // <editor-fold defaultstate="collapsed" desc="IOCB Register Bit Description">
                                        // bit 7-4 and 2-0 Interrupt on Change PortGPA Register bits, 1=enabled, 0=disabled
                                        // bit 3 Unimplemented, Read as 0
    // </editor-fold>
    ADCON0  = ADC_MUX_VIN | 0x01;       // ADC Control Register 0.  Enables the ADC and sets the default input channel to VIN_ANA (00000)
    // <editor-fold defaultstate="collapsed" desc="ADCON0 Register Bit Description">
                                        // bit 7 Unimplemented, read as 0
                                        // bit 6-2 CHS<4:0> Analog Channel Selected bits
                                        // 00000 = VIN_ANA analog voltage proportional to Vin
                                        // 00001 = VREGREF reference voltage for V_REG output
                                        // 00010 = OV_REF feference for overvoltage comparator
                                        // 00011 = UV_REF reference for undervoltage comparator
                                        // 00100 = VBGR band gap reference
                                        // 00101 = INT_VREG internal versin of the V_REG load voltage
                                        // 00110 = CRT voltage proportional to the current in the indutor
                                        // 00111 = VZC an internal ground, voltage for zero current
                                        // 01000 = DEMAND input to current loop, output of demand mux
                                        // 01001 = RELEFF analog voltage proportional to duty cycle
                                        // 01010 = TMP_ANA analog voltage proportional to temperature
                                        // 01011 = ANA_IN demanded current from the remote master
                                        // 01100 = DCI DC Inductor valley current
                                        // 01101 = Unimplemented
                                        // 01110 = Unimplemented
                                        // 01111 = Unimplemented
                                        // 10000 = GPA0 i.e. ADDR1
                                        // 10001 = GPA1 i.e. ADDR0
                                        // 10010 = GPA2 i.e. Temperature Sensor Input
                                        // 10011 = GPA3 i.e. Tracking Voltage
                                        // 10100 = GPB1
                                        // 10101 = GPB2
                                        // 10110 = GPB4
                                        // 10111 = GPB5
                                        // 11000 = Unimplemented
                                        // 11001 = Unimplemented
                                        // 11010 = Unimplemented
                                        // 11011 = Unimplemented
                                        // 11100 = Unimplemented
                                        // 11101 = Unimplemented
                                        // 11110 = Unimplemented
                                        // 11111 = Unimplemented
    // </editor-fold>
    ADCON1  = 0x20;                     // ADC Control Register 1.  Enable F_OSC / 32 which sets 4usec per conversion.
    // <editor-fold defaultstate="collapsed" desc="ADCON1 Register Bit Description">
                                        // bit 7 Unimplemented, Read as 0
                                        // bit 6-4 ADCS<2:0> ADC Clock Select Bits
                                        // 000 = Reserved
                                        // 001 = F_OSC/8
                                        // 010 = F_OSC/32
                                        // x11 = F_RC (clock derived from internal oscillator with a divisor of 16)
                                        // 100 = Reserved
                                        // 101 = F_OSC/16
                                        // 111 = F_OSC/64
                                        // bit 3-0 Unimplemented, Read as 0
    // </editor-fold>
	PORTGPA = 0b00000000;               // Clear port A to start
    // <editor-fold defaultstate="collapsed" desc="PORTGPA Register Bit Description">
                                        // bit 7 GPA7 General Purpose Open Drain I/O Pin
                                        // bit 6 GPA6 General Purpose I/O Pin, 1=Port pin is > VIH, 0=Port Pin is < VIL
                                        // bit 5 GPA5/MCLR# General Purpose Open Drain I/O pin
                                        // bit 4 GPA7 General Purpose Open Drain I/O pin
                                        // bit 3-0 GPA<3:0> General Purpose I/O Pin, 1=Port Pin is > VIH, 0=Port Pin is < VIL
    // </editor-fold>

#ifdef ENABLE_BENCH_TEST_OUTPUT_PIN
	ATSTCON = 0b10000011;               // Analog Bench Test Control Register, Configure output MOSFET drivers (safe power-up, so do this first thing), GPA0 set for Benchtest.
    // <editor-fold defaultstate="collapsed" desc="ATSTCON Register Bit Description">
                                        // bit 7 reserved
                                        // bit 6-5 Unimplemented and Read as 0
                                        // bit 4 Reserved
                                        // bit 3 HIDIS: High Side Driver Control Bit, 1=Disabled, 0=Enabled
                                        // bit 2 LODIS: Low Side Driver Control Bit, 1=Disabled, 0=Enabled
                                        // bit 1 BNCHEN: GPA0 bench test configuration control bit, 1=GPA0 is configured for analog bench test output, 0=GPA0 Normal Operation
                                        // bit 0 DRVDIS: MOSFET Driver Disable Control Bit, 1=High and Low Side Drivers set Low, PHASE floating, 0=Normal Operation
    // </editor-fold>
    TRISGPA = 0b10100011;               // Port A Tristate Register. A7,A6,A5,A4,A3 as Input. LED outputs on A1,A2 active high. Benchtest requires A0 to be an output.
    // <editor-fold defaultstate="collapsed" desc="TRISGPA Register Bit Description">
                                        // bit 7-6 TRISA<7:6> PORTGPA Tri-State Control bit, 1=input (tri-stated), 0=output
                                        // bit 5 TRISA5 GPA5 Tri-State Control bit. This bit is always 1 as GPA5 is an input only
                                        // bit 4-0 TRISA<4:0> PORTGPA Tri-State Control Bit, 1=input (tri-state), 0=output
    // </editor-fold>
	ANSELA  = 0b00000011;               // Analog Select Port A Register.  A0 and A3 are Analog inputs.  Benchtest requires GPA0 be configured for analog.
    // <editor-fold defaultstate="collapsed" desc="ANSELA Register Bit Description">
                                        // bit 7-4 Unimplemented, Read as 0
                                        // bit 3-0 Digital I/O. Pin is assigned to port or special function.
    // </editor-fold>
    WPUGPA  = 0b00000000;               // Port A Weak Pull-Up Register for digital inputs (A7, A6, A4 are open-drain). A5 (MCLR#) has a weak pullup.
    // <editor-fold defaultstate="collapsed" desc="WPUGPA Register Bit Description">
                                        // bit 7-6 Unimplemented, Read as 0
                                        // bit 5 WPUA5 Weak Pull-Up Register bit, 1=Pull-up enabled, 0=Disabled
                                        // bit 4 Unimplemented, Read as 0
                                        // bit 3-0 WPUA<3:0> Weak Pull-up Register Bit, 1=Pull-up enabled, 0=Disabled
                                        // NOTE: The weak pull-up device is enabled only when the global RAPU# bit is enabled, the pin is in input mode (TRISGPA=1),
                                        //       the individual WPUA bit is enabled (WPUA=1), and the pin is not configured as an analog input.
    // </editor-fold>

#else
	ATSTCON = 0b10000001;               // Analog Bench Test Control Register, Configure output MOSFET drivers (safe power-up, so do this first thing), GPA0 set for Benchtest.
    // <editor-fold defaultstate="collapsed" desc="ATSTCON Register Bit Description">
                                        // bit 7 reserved
                                        // bit 6-5 Unimplemented and Read as 0
                                        // bit 4 Reserved
                                        // bit 3 HIDIS: High Side Driver Control Bit, 1=Disabled, 0=Enabled
                                        // bit 2 LODIS: Low Side Driver Control Bit, 1=Disabled, 0=Enabled
                                        // bit 1 BNCHEN: GPA0 bench test configuration control bit, 1=GPA0 is configured for analog bench test output, 0=GPA0 Normal Operation
                                        // bit 0 DRVDIS: MOSFET Driver Disable Control Bit, 1=High and Low Side Drivers set Low, PHASE floating, 0=Normal Operation
    // </editor-fold>
    TRISGPA = 0b10100010;               // Port A Tristate Register.  A7, A6, A5, A4, A3 set as Input.  LED outputs are on GPA0/1/2 active high
    // <editor-fold defaultstate="collapsed" desc="TRISGPA Register Bit Description">
                                        // bit 7-6 TRISA<7:6> PORTGPA Tri-State Control bit, 1=input (tri-stated), 0=output
                                        // bit 5 TRISA5 GPA5 Tri-State Control bit. This bit is always 1 as GPA5 is an input only
                                        // bit 4-0 TRISA<4:0> PORTGPA Tri-State Control Bit, 1=input (tri-state), 0=output
    // </editor-fold>
	ANSELA  = 0b00000010;               // Port B Analog Select Register.  A3 is an analog input. Benchtest pin disabled for LED user interfaces
    // <editor-fold defaultstate="collapsed" desc="ANSELA Register Bit Description">
                                        // bit 7-4 Unimplemented, Read as 0
                                        // bit 3-0 Digital I/O. Pin is assigned to port or special function.
    // </editor-fold>
    WPUGPA  = 0b00000000;               // Port A Weak Pull-Up Register for digital inputs (A7, A6, A4 are open-drain). A5 (MCLR#) has a weak pullup.
    // <editor-fold defaultstate="collapsed" desc="WPUGPA Register Bit Description">
                                        // bit 7-6 Unimplemented, Read as 0
                                        // bit 5 WPUA5 Weak Pull-Up Register bit, 1=Pull-up enabled, 0=Disabled
                                        // bit 4 Unimplemented, Read as 0
                                        // bit 3-0 WPUA<3:0> Weak Pull-up Register Bit, 1=Pull-up enabled, 0=Disabled
                                        // NOTE: The weak pull-up device is enabled only when the global RAPU# bit is enabled, the pin is in input mode (TRISGPA=1),
                                        //       the individual WPUA bit is enabled (WPUA=1), and the pin is not configured as an analog input.
    // </editor-fold>
#endif

    PORTGPB = 0b00000000;               // Clear port B to start (GPB3 not present as a pin on this part)
    // <editor-fold defaultstate="collapsed" desc="PORTGPB Register Bit Description">
                                        // bit 7-4 GPB<7:4> General Purpose I/O Pin bit, 1=Port pin is >VIH, 0=Port Pin is <VIL
                                        // bit 3 Unimplemented, Read as 0
                                        // bit 2-0 GPB<2:0> General Purpose I/O Pin bit, 1=Port pin is >VIH, 0=Port Pin is <VIL
    // </editor-fold>
    TRISGPB = 0b00000111;               // Port B Tristate Register.  B3, B1, B0 set as Input. B7, B6, B5, B4, output for LEDs and Pack Disconnect. ICSP lines set to outputs to prevent floating.
    // <editor-fold defaultstate="collapsed" desc="TRISGPB Register Bit Description">
                                        // bit 7-4 TRISB<7:4> PORTGPB Tri-State Control Bit, 1=Pin configured as input (tri-stated), 0=pin configured as output
                                        // bit 3 Unimplemented, Read as 1.
                                        // bit 2-0 TRISB<2:0> PORTGPB Tri-State Control Bit, 1=Pin configured as input (tri-stated), 0=pin configured as output
    // </editor-fold>
    ANSELB  = 0b00000110;               // Port B Analog Select Register. GPB1 is an analog input for measuring VBAT amplifier circuit.
    // <editor-fold defaultstate="collapsed" desc="ANSELB Register Bit Description">
                                        // bit 7-6 Unimplemented, Read as 0.
                                        // bit 5-4 ANSB<5:4> Analog Select PORTGPB Register Bit, 1=Analog input, 0=Digital I/O or Special Function
                                        // bit 3 Unimplemented, Read as 0.
                                        // bit 2-1 ANSB<2:1> Analog Select PORTGPB Register bit, 1=Analog input, 0=Digital I/O or Special Function
                                        // bit 0 Unimplemented, Read as 0.
    // </editor-fold>
    WPUGPB  = 0b00000000;               // Port B Weak Pull-up Register for digital inputs (B0 is an open-drain, B3 not on this part). B2,B3,B4,B5,B6 have pullups.
    // <editor-fold defaultstate="collapsed" desc="WPUGPB Register Bit Description">
                                        // bit 7-4 WPUB<7:4> Weak Pull-Up Register bit
                                        // bit 3 Unimplemented, Read as 0
                                        // bit 2-1 WPUB<2:1> Weak Pull-Up Register bit
                                        // bit 0 Unimplemented, Read as 0.
                                        // NOTE: The weak pull-up device is enabled only when the global RAPU# bit is enabled, the pin is in input mode (TRISGPB=1),
                                        //       the individual WPUB bit is enabled (WPUB=1), and the pin is not configured as an analog input.
    // </editor-fold>

/* Set up the PWM Generator with Timer2. Frequency and Maximum Duty Cycle. */
    
    T2CON   = 0x00;                     // Timer2 Control Register. Timer OFF, Prescalar at 1
    // <editor-fold defaultstate="collapsed" desc="T2CON Register Bit Description">
                                        // bit 7-3 Unimplemented, Read as 0
                                        // bit 2 TMR2ON Timer2 On bit, 1=On, 0=Off
                                        // bit 1-0 T2CKPS<1:0> Timer2 Clock Prescale Select bit
                                        // 00 = Prescaler is 1
                                        // 01 = Prescaler is 4
                                        // 10 = Prescaler is 8
                                        // 11 = Prescaler is 16
    // </editor-fold>
    TMR2    = 0x00;                     // Timer2 Time Base set to 0x00.  This is the target duty cycle.
    // <editor-fold defaultstate="collapsed" desc="TMR2 Register Bit Description">
                                        // bit 7-0 Holding Register for the 9-bit TMR2 Time Base.  Basically the target duty cycle (D = TMR2/PR2)
    // </editor-fold>
    APFCON  = 0x00;                     // Alternate Pin Function Control Register. Leave at Default 0x00.
    // <editor-fold defaultstate="collapsed" desc="APFCON Register Bit Description">
                                        // bit 7-1 Unimplemented, Read as 0
                                        // bit 0 CLKSEL Pin Selection Bit. 1=Multi-phase or multiple output clock function is on GPB5, 0=On GPA1.
    // </editor-fold>
    PWMPHL  = 0x00;                     // SLAVE Phase shift. Not used and set to 0.
    // <editor-fold defaultstate="collapsed" desc="PWMPHL Register Bit Description">
                                        // bit 7-0 Phase shift added to the SLAVE system clock received on GPA1 in a MASTER/SLAVE or multi-phase implementation. 
                                        // Not used in Single Phase.  If APFCON bit 0 is set then the clock is received on GPB5 instead.
    // </editor-fold>
    PWMRL   = 0x19;                     // Sets the maximum PWM duty cycle.  Set to 25.  So (25*296kHz)/8MHz = 93.75%
    // <editor-fold defaultstate="collapsed" desc="PWMRL Register Bit Description">
                                        // bit 7-0 PWM Register Low Byte.  Equation is PWM_Duty_Cycle = PWMRL * T_OSC * TMR2_Prescale_Value
                                        // PWMRL can be written to at any time, but it will not be latched into PWMRH until after a match between PR2 and TMR2 occurs.
    // </editor-fold>
    PR2     = 0x1B;                     // Timer2 Period Register.  Set to 300 kHz frequency: 8MHz/300kHz = 26.7. So Set 27 for 296kHz.
    // <editor-fold defaultstate="collapsed" desc="PR2 Register Bit Description">
                                        // bit 7-0 Timer2 Module Period Register
                                        // This is the PWM period.  The equation is PWM_Period = [(PR2)+1] * T_OSC * T2_Prescale_Value
    // </editor-fold>
    T2CON   = 0x04;                     // Timer 2 Control Register. Timer ON. Prescalar at 1.
    // <editor-fold defaultstate="collapsed" desc="T2CON Register Bit Description">
                                        // bit 7-3 Unimplemented, Read as 0
                                        // bit 2 TMR2ON Timer2 On bit, 1=On, 0=Off
                                        // bit 1-0 T2CKPS<1:0> Timer2 Clock Prescale Select bit
                                        // 00 = Prescaler is 1
                                        // 01 = Prescaler is 4
                                        // 10 = Prescaler is 8
                                        // 11 = Prescaler is 16
    // </editor-fold>
    
/* Analog power supply specific configurations */
    
    BUFFCON     = 0x07;                 // 0x108 Unity Gain Buffer Control Register. Configured as Stand-Alone Unit and Buffer puts out Vin Divided down by 1/5.
    // <editor-fold defaultstate="collapsed" desc="BUFFCON Register Bit Description">
                                        // bit 7-5 MLTPH<2:0> System Configuration Bits
                                        // 000 = Device set as stand-alone unit
                                        // 001 = Device set as multiple output MASTER
                                        // 010 = Device set as multiple output SLAVE
                                        // 011 = Device set as multi-phase MASTER
                                        // 111 = Device set as multi-phase SLAVE
                                        // bit 4-0 ASEL<4:0> Multiplexter Output Control Bit
                                        // 00000 = Voltage proportional to current in the inductor
                                        // 00001 = Error amplifier output plus slope compensation, input to PWM comparator
                                        // 00010 = Input to slope compensation circuitry
                                        // 00011 = Band gap reference
                                        // 00100 = Output voltage reference
                                        // 00101 = Output voltage after internal differential amplifier
                                        // 00110 = Unimplemented
                                        // 00111 = voltage proportional to the internal temperature
                                        // 01000 = Internal ground for current sense circuitry
                                        // 01001 = Output overvoltage comparator reference
                                        // 01010 = Output under voltage comparator reference
                                        // 01011 = Error amplifier output
                                        // 01100 = For a multi-phase SLAVE, error amplifier signal received from MASTER
                                        // 01101 = For multi-phase SLAVE, error signal received from MASTER with gain
                                        // 01110 = Vin Divided down by 1/5
                                        // 01111 = DC Inductor Valley Current
                                        // 11101 = Over Current Reference
                                        // Everything else unimplemented.
                                        // If ATSTCON<BNCHEN> is set, the device is in Bench Test Mode and will put one of these out on GPA0. 
                                        // Be sure to add the Buffer Offset to the measured signal which can be read from memory location 0x2087.
    // </editor-fold>
    PE1         = 0x80;                 // 0x87 Analog Peripheral Enable 1 Control Register. Enable diode emulation. This is a *MUST* or the battery could destroy the sync FET.
    // <editor-fold defaultstate="collapsed" desc="PE1 Register Bit Description">
                                        // bit 7 (DECON) is Diode Emulation Mode. 1(Enabled), 0(Disabled).  Enabled the synchronous rectification will disable when indutor current reaches zero.
                                        // bit 6 (DBRSTR) is High Side Drive Strength Configuration Bit.  1 (1A Sink/Source), 0 (2A Sink/Source)
                                        // bit 5 (HDLYBY) is High Side Dead Time Bypass bit
                                        // bit 4 (LDLYBY) is Low Side Dead Time Bypass bit
                                        // bit 3 (PDEN) is -V_sen Weak Pull Down Enable bit
                                        // bit 2 (PUEN) is +V_sen Weak Pull Up Enable bit
                                        // bit 1 (UVTEE) is Output Undervoltage Accelerator Enable bit
                                        // bit 0 (OVTEE) is Output Over Voltage Accelerator Enable bit
    // </editor-fold>
    DEADCON     = 0x9B;                 // 0x9B Driver Dead time Control Register.  47nS high-side dead time, 48nS low-side dead time 
    //// <editor-fold defaultstate="collapsed" desc="DEADCON Bit Descriptions">
                                        // bit 7-4 HDLY<3:0> High-Side Dead Time Configuration Bits
                                        // 0000 = 11nsec delay
                                        // 0001 = 15nsec delay
                                        // 0010 = 19nsec delay
                                        // 0011 = 23nsec delay
                                        // 0100 = 27nsec delay
                                        // 0101 = 31nsec delay
                                        // 0110 = 35nsec delay
                                        // 0111 = 39nsec delay
                                        // 1000 = 43nsec delay
                                        // 1001 = 47nsec delay
                                        // 1010 = 51nsec delay
                                        // 1011 = 55nsec delay
                                        // 1100 = 59nsec delay
                                        // 1101 = 63nsec delay
                                        // 1110 = 67nsec delay
                                        // 1111 = 71nsec delay
                                        // bit 3-0 LDLY<3:0> Low-Side Dead Time Configuration Bits
                                        // 0000 = 4nsec delay
                                        // 0001 = 8nsec delay
                                        // 0010 = 12nsec delay
                                        // 0011 = 16nsec delay
                                        // 0100 = 20nsec delay
                                        // 0101 = 24nsec delay
                                        // 0110 = 28nsec delay
                                        // 0111 = 32nsec delay
                                        // 1000 = 36nsec delay
                                        // 1001 = 40nsec delay
                                        // 1010 = 44nsec delay
                                        // 1011 = 48nsec delay
                                        // 1100 = 52nsec delay
                                        // 1101 = 56nsec delay
                                        // 1110 = 60nsec delay
                                        // 1111 = 64nsec delay
    // </editor-fold>
    ABECON      = 0x0D;                 // 0x109 Analog Block Enable Control Register.  Enable Signal Chain, Internal Temp Sensor, Current Measurement.
    ABECON      = 0x05;                 // 0x109 Analog Block Enable Control Register.  Enable Signal Chain, Internal Temp Sensor, Current Measurement.
    // <editor-fold defaultstate="collapsed" desc="ABECON Register Bit Description">
                                        // bit 7 (OVDCEN) Output over voltage DAC Control bit
                                        // bit 6 (UVDCEN) Output under voltage DAC Control bit
                                        // bit 5 (MEASEN) Relative Efficiency Measurement Control bit
                                        // bit 4 (SLCPBY) Slope compensation bypass control bit. (1)Slope Comp Disabled. (0)Slope Comp Enabled.
                                        // bit 3 (CRTMEN) Current Measurement circuitry control bit
                                        // bit 2 (TMPSEN) Internal temperature sensor control bit
                                        // bit 1 (RECIREN) Relative efficiency circuit control bit
                                        // bit 0 (PATHEN) Signal chain circuitry control bit
    // </editor-fold>
    VZCCON      = 0x60;                 // 0x97 Voltage for zero current register (3.28mV/bit steps). 0x60 = -104.96mV. 
    //// <editor-fold defaultstate="collapsed" desc="VZCCON Register Bit Description">
                                        // bit7-0 VZC<7:0> Voltage for Zero Current Setting Bits
                                        // 00000000 = -420mV Offset
                                        // 00000001 = -416.72mV Offset
                                        // ......
                                        // 10000000 = 0mV Offset
                                        // ......
                                        // 11111110 = +413.12mV Offset
                                        // 11111111 = +416.40mV Offset
    // </editor-fold>
    CMPZCON     = 0x0E;                 // 0x98 Compensation Setting Control Register. Zero Set to 12.2kHz. Gain Set to 0dB.
    //<editor-fold defaultstate="collapsed" desc="CMPZCON Register Bit Description">
    
                                        // bit 7-4 CMPZF<3:0> Compensation Zero Frequency Setting Bits
                                        // 0000 = 1500Hz
                                        // 0001 = 1850Hz
                                        // 0010 = 2300Hz
                                        // 0011 = 2840Hz
                                        // 0100 = 3460Hz
                                        // 0101 = 4300Hz
                                        // 0110 = 5300Hz
                                        // 0111 = 6630Hz
                                        // 1000 = 8380Hz
                                        // 1001 = 9950Hz
                                        // 1010 = 12200Hz
                                        // 1011 = 14400Hz
                                        // 1100 = 18700Hz
                                        // 1101 = 23000Hz
                                        // 1110 = 28400Hz
                                        // 1111 = 35300Hz
                                        // bit 3-0 CMPZG<3:0> Compensation Gain Setting Bits
                                        // 0000 = 36.15dB
                                        // 0001 = 33.75dB
                                        // 0010 = 30.68dB
                                        // 0011 = 28.43dB
                                        // 0100 = 26.10dB
                                        // 0101 = 23.81dB
                                        // 0110 = 21.44dB
                                        // 0111 = 19.10dB
                                        // 1000 = 16.78dB
                                        // 1001 = 14.32dB
                                        // 1010 = 12.04dB
                                        // 1011 = 9.54dB
                                        // 1100 = 7.23dB
                                        // 1101 = 4.61dB
                                        // 1110 = 2.28dB
                                        // 1111 = 0dB
    //</editor-fold>
    SLPCRCON    = 0x82;                 // 0x9C Slope compensation ramp control register.  
    //**BRYAN** I set the SLPS above to 2 as per the datasheet. It was 0.
    // <editor-fold defaultstate="collapsed" desc="SLPCRCON Register Bit Description">
                                        // bit 7-4 SLPG<3:0> Slope Compensation Amplitude Configuration Bits
                                        // 0000 = 0.017 Vpk-pk measured from 50% duty cycle waveform
                                        // 0001 = 0.022 Vpk-pk measured from 50% duty cycle waveform
                                        // 0010 = 0.030 Vpk-pk measured from 50% duty cycle waveform
                                        // 0011 = 0.040 Vpk-pk measured from 50% duty cycle waveform
                                        // 0100 = 0.053 Vpk-pk measured from 50% duty cycle waveform
                                        // 0101 = 0.070 Vpk-pk measured from 50% duty cycle waveform
                                        // 0110 = 0.094 Vpk-pk measured from 50% duty cycle waveform
                                        // 0111 = 0.125 Vpk-pk measured from 50% duty cycle waveform
                                        // 1000 = 0.170 Vpk-pk measured from 50% duty cycle waveform
                                        // 1001 = 0.220 Vpk-pk measured from 50% duty cycle waveform
                                        // 1010 = 0.300 Vpk-pk measured from 50% duty cycle waveform
                                        // 1011 = 0.400 Vpk-pk measured from 50% duty cycle waveform
                                        // 1100 = 0.530 Vpk-pk measured from 50% duty cycle waveform
                                        // 1101 = 0.700 Vpk-pk measured from 50% duty cycle waveform
                                        // 1110 = 0.940 Vpk-pk measured from 50% duty cycle waveform
                                        // 1111 = 1.250 Vpk-pk measured from 50% duty cycle waveform
                                        // bit 3-0 SLPS<3:0> Slope Compensation Delta-V/Delta-t Configuration Bits
                                        // Set this byte proportional to the switching frequency. n = (Fsw/100000)-1
    // </editor-fold>
    SLVGNCON    = 0x00;                 // 0x9D Master Error Signal Input Gain Control Register. Configured for the SLAVE device. Not used in this design so set to zero.
    // <editor-fold defaultstate="collapsed" desc="SLVGNCON Register Bit Description">
                                        // bit 7-5 Unimplemented, Read as 0
                                        // 00000 = -3.3dB
                                        // 00001 = -3.1dB
                                        // 00010 = -2.9dB
                                        // 00011 = -2.7dB
                                        // 00100 = -2.5dB
                                        // 00101 = -2.3dB
                                        // 00110 = -2.1dB
                                        // 00111 = -1.9dB
                                        // 01000 = -1.7dB
                                        // 01001 = -1.4dB
                                        // 01010 = -1.2dB
                                        // 01011 = -1.0dB
                                        // 01100 = -0.8dB
                                        // 01101 = -0.6dB
                                        // 01110 = -0.4dB
                                        // 01111 = -0.2dB
                                        // 10000 = 0.0dB
                                        // 10001 = 0.2dB
                                        // 10010 = 0.4dB
                                        // 10011 = 0.7dB
                                        // 10100 = 0.9dB
                                        // 10101 = 1.1dB
                                        // 10110 = 1.3dB
                                        // 10111 = 1.5dB
                                        // 11000 = 1.7dB
                                        // 11001 = 1.9dB
                                        // 11010 = 2.1dB
                                        // 11011 = 2.3dB
                                        // 11100 = 2.6dB
                                        // 11101 = 2.8dB
                                        // 11110 = 3.0dB
                                        // 11111 = 3.2dB
                                        // The SLVGNCON register is configured in the multi-phase SLAVE device, not the MASTER.
    
    // </editor-fold>
    CSGSCON     = 0x0F;                 // 0x93 Current Sense AC Gain Control Register.  AC gain = 22dB 
    // <editor-fold defaultstate="collapsed" desc="CSGSCON Register Bit Description">
                                        // bit 3-0 CSGS<3:0> Current Sense AC Gain Setting Bits
                                        // 0000 = 0dB
                                        // 0001 = 1.0dB
                                        // 0010 = 2.5dB
                                        // 0011 = 4.0dB
                                        // 0100 = 5.5dB
                                        // 0101 = 7.0dB
                                        // 0110 = 8.5dB
                                        // 0111 = 10.0dB
                                        // 1000 = 11.5dB
                                        // 1001 = 13.0dB
                                        // 1010 = 14.5dB
                                        // 1011 = 16.0dB
                                        // 1100 = 17.5dB
                                        // 1101 = 19.0dB
                                        // 1110 = 20.5dB
                                        // 1111 = 22.0dB
    // </editor-fold>
    CSDGCON     = 0x00;                 // 0x95 Current Sense DC Gain Control Register. DC gain set to 19.5dB, It's not used in control loop, but read by the ADC.
    // <editor-fold defaultstate="collapsed" desc="CSDGCON Register Bit Description">
                                        // bit7 CSDGEN Current Sense DC Gain Enable Bit: 1 = DC Gain current sense signal used in control loop. 0 = signal only read by ADC.
                                        // bit6-4 Unimplemented and read as 0
                                        // bit3 Reserved
                                        // bit2-0 CSDG<2:0> Current Sense DC Gain Setting Bits
                                        // 000 = 19.5dB
                                        // 001 = 21.8dB
                                        // 010 = 24.1dB
                                        // 011 = 26.3dB
                                        // 100 = 28.6dB
                                        // 101 = 30.9dB
                                        // 110 = 33.2dB
                                        // 111 = 35.7dB    
    // </editor-fold>
    OCCON       = 0xFF;                 // 0x91 Output Overcurrent Control Register.     //**BRYAN** Go figure out what to set this to for the evaluation board.
    // <editor-fold defaultstate="collapsed" desc="OCCON Register Bit Description">
                                        // bit 7 OCEN Output Overcurrent DAC Control Bit Enable
                                        // bit6-5 OCLEB<1:0> Leading Edge Blanking
                                        // 00 = 114ns blanking
                                        // 01 = 213ns blanking
                                        // 10 = 400ns blanking
                                        // 11 = 780ns blanking
                                        // bit4-0 OOC<4:0> Output Overcurrent Configuration bits
                                        // 00000 = 160mV drop
                                        // 00001 = 175mV drop
                                        // 00010 = 190mV drop
                                        // 00011 = 205mV drop
                                        // 00100 = 220mV drop
                                        // 00101 = 235mV drop
                                        // 00110 = 250mV drop
                                        // 00111 = 265mV drop
                                        // 01000 = 280mV drop
                                        // 01001 = 295mV drop
                                        // 01010 = 310mV drop
                                        // 01011 = 325mV drop
                                        // 01100 = 340mV drop
                                        // 01101 = 355mV drop
                                        // 01110 = 370mV drop
                                        // 01111 = 385mV drop
                                        // 10000 = 400mV drop
                                        // 10001 = 415mV drop
                                        // 10010 = 430mV drop
                                        // 10011 = 445mV drop
                                        // 10100 = 460mV drop
                                        // 10101 = 475mV drop
                                        // 10110 = 490mV drop
                                        // 10111 = 505mV drop
                                        // 11000 = 520mV drop
                                        // 11001 = 535mV drop
                                        // 11010 = 550mV drop
                                        // 11011 = 565mV drop
                                        // 11100 = 580mV drop
                                        // 11101 = 595mV drop
                                        // 11110 = 610mV drop
                                        // 11111 = 625mV drop
    // </editor-fold>
    OVCCON      = 0x00;                 // 0x19 Output Voltage Set Point Coarse Control Register. Set to 0 to begin. 
    //<editor-fold defaultstate="collapsed" desc="OVCCON Register Description">
                                        // This is the output voltage error amplifier reference DAC. Coarse setting is 8bit in 15.8mV increments.
    //</editor-fold>
    OVFCON      = 0x80;                 // 0x1A Output Voltage Set Point Fine Control Register.  Enable DAC and set to 0 to begin. 
    //<editor-fold defaultstate="collapsed" desc="OVFCON Register Description">
                                        // The fine setting is 5bit in 0.82mV steps. Bit 7 enables the DAC.
    // </editor-fold>
    
    //**BRYAN** Add some code for taking target voltages and addressing this setting from the configuration space if it makes sense for OUVCON and OOVCON. 
    //**BRYAN** Since these interrupts are disabled this is going to be commented out below for now.  Adding for reference code thoroughness though.
    //  OUVCON  = 0x00;                     // Output under voltage detect level control register. 
    // <editor-fold defaultstate="collapsed" desc="OUVCON Register Bit Description">
                                        // bit 7-0 OUV<7:0> Output Under Voltage Detect Level Configuration Bits
                                        // Equation is OUV<7:0> = (V_out_uv_detect_level)/15mV
    // </editor-fold>
    //  OOVCON  = 0xff;                     // Output over voltage detect level control register.
    // <editor-fold defaultstate="collapsed" desc="OOVCON Register Bit Description">
                                        // bit 7-0 OOV<7:0> Output Overvoltage Detect Level Configuration bits
                                        // Equation is OOV<7:0> = V_out_ov_detect_level / 15mV
    // </editor-fold>

    vh          = 0;                    // DAC High Byte 
    vl          = 0;                    // DAC Low Byte 

/* Enable input Under Voltage Lockout threshold */
    VINLVL = 0x80 | cs.uvlo_threshold_on;  // Input Under Voltage Lockout Control Register. Enable and set via configuration in cs.uvlo_threshold_on
    // <editor-fold defaultstate="collapsed" desc="VINLVL Register Bit Description">
                                        // bit 7 UVLOEN Under Voltage Lockout DAC Control bit, 1=Enabled, 0=Disabled
                                        // bit 6 Unimplemented = 0
                                        // bit 5-0 UVLO<5:0> Under voltage Lockout Configuration Bits = 26.5*ln(UVLO_SET-POINT/4)
    // </editor-fold>

}

void inc_iout(short increment)      // Topology Specific function - Increase Current (Combined)
{
	int i;

  for(i=0;i<increment;i++)			// Loop through the increment routine (if fast_ramp requires)
  {
    vl++;							// Increment fine register
    if (vl > IOUT_LSB_ROLL) 		// If we are over the top of the fine register
	{
		if (vh < cs.ovcfcon_max) {	// and the coarse register is not maxed out
            vh ++;					// increment coarse register
			vl = 0;					// and reset fine register to zero
        }
        else {
            vh = cs.ovcfcon_max;	// Set to max
            vl = IOUT_LSB_ROLL;		// We went one increment beyond IOUT_LSB_ROLL, so reset to max.
        }
    }
  }
    OVFCON = (unsigned short)(0x80 | vl);             //Load the DAC with the new values
    OVCCON = vh;
}

short dec_iout( short decrement)	// Topology Specific function - Decrease Current (Combined)
{
	int i;

    if ((vh == 0) && (vl == 0))     // Are we already at the bottom (off)?
        return(0);
	for (i = 0; i<decrement; i++)
	{
		if(vl > 0)					// Must check for zero because 'vl' can roll to FF if not.
			vl--;					// Reduce fine value if we are not already at the bottom of the range

		if (vl == 0) 				// If fine == 0
		{
			if (vh > cs.ovcfcon_min) // and corse > the minimum
			{
				vh--;				 // Then decrease course and reset fine to top of range
				vl = IOUT_LSB_ROLL;	 //
			}
			else
			{
				vh = cs.ovcfcon_min; // Else, we are at the bottom (vl = vf = 0)
				return(0);			 // Return a fault code that we can't decrement any more
			}
		}
	}
    OVFCON = (unsigned short)(0x80 | vl);	// Update the setpoint registers
    OVCCON = vh;		//
	return(1);			// Success
}

void zero_iout(void)
{
	/* Output current set to zero */
	vl = 0;
	vh = cs.ovcfcon_min;
	OVCCON = 0;
	OVFCON = 0x80;
}

void disable_charger(void)
{
	/* PWM off */
	ATSTCONbits.DRVDIS = 1;

	/* Adjust UVLO threashold */
	VINLVL = 0x80 | cs.uvlo_threshold_on;
    
    /* Set output current to minimum */
    zero_iout();
}

void enable_charger(void) //%%BRYAN%% need to add the output overcurrent and overvoltage interrupt enables
{
    /* Set output current to minimum */
    zero_iout();

	/* Adjust UVLO threshold */
	VINLVL = 0x80 | cs.uvlo_threshold_off;

	/* UVLO interrupt */
	PIE2bits.VINIE = 1;

	/* PWM on */
	ATSTCONbits.DRVDIS = 0;

	/* Input FET on */
	VBAT_EN = 1;
}

#ifdef ENABLE_FIXED_CONFIG
#include "MultiChemCharger_Values.h"
#endif

void init_cal_registers(void)
{
    PMADRH = 0x20;
    PMADRL = 0x80;
    PMCON1bits.CALSEL = 1;
    PMCON1bits.RD = 1;
    asm("nop");
    asm("nop");
    DOVCAL = PMDATH;                    // Load Device Calibration from memory (Voltage sense differential amp adjust)
    OSCCAL = PMDATL;                    // Load OSCCAL value from memory (Oscillator tune bits)

    PMADRH = 0x20;
    PMADRL = 0x82;
    PMCON1bits.CALSEL = 1;
    PMCON1bits.RD = 1;
    asm("nop");
    asm("nop");
    VROCAL = PMDATH;                    // Over-temp calibration bits
    BGRCAL = PMDATL;                    // Internal band-gap reference calibration

    PMADRH = 0x20;
    PMADRL = 0x83;
    PMCON1bits.CALSEL = 1;
    PMCON1bits.RD = 1;
    asm("nop");
    asm("nop");
    TTACAL = PMDATH;                    // Output voltage regulation reference set-point calibration
    ZROCAL = PMDATL;                    // Error amplifier offset calibration
    
}

/* This ifdef is meant to save a little code and prevent flash writes when this device function was locked down.  */
/* While it adds the ability to include MultiChemCharger_Values.h it locks out the ability for the GUI to change  */
/* configuration settings. Be aware of this.  If you are testing compiling with the header file and still want to */ 
/* be able to use the GUI then be sure to set the ENABLE_GUI_CONFIG and ENABLE_FIXED_CONFIG in the header file.   */
#ifdef ENABLE_GUI_CONFIG   

/* This function writes 8 bytes (4 flash words) at a time to flash. */
/* The incoming address must be 8-byte aligned.                     */

unsigned short write_flash(unsigned short addr, unsigned short counter) 
{
    unsigned char b = 0;
	if (addr < CAL_BASE_ADDR)           // Invalid address.  Should return a failure result
	{
        return 0x00; 
    }
    if (counter == 8)				// Write data if 8 bytes (4-words) have been received
    {
        while (b < 8) 
        {
            PMADRH = addr >> 8;
            PMADRL = addr;
            PMDATL = flash_write_buf[(unsigned char)(b & 0x07)];
            b++;
            PMDATH = flash_write_buf[(unsigned char)(b & 0x07)];
            b++;
            PMCON1bits.CALSEL = 0;
            PMCON1bits.WREN = 1;
            PMCON2 = 0x55;
            PMCON2 = 0xAA;
            PMCON1bits.WR = 1;
            NOP();
            NOP();
            PMCON1bits.WREN = 0;
            addr++;
        }
        return 0x04;
    }
    return 0x00;
}
#endif

unsigned short read_flash(unsigned short addr)
{
    unsigned short a;

    PMADRH = addr >> 8;
    PMADRL = addr;
    PMCON1bits.CALSEL = 0;
    PMCON1bits.RD = 1;
    asm("nop");
    asm("nop");
    a = (unsigned short)(PMDATH << 8);
    a |= PMDATL;
    return a;
}

unsigned char check_for_uvlo(void)
{
    if ((cd.adc_vin < cs.uvlo_adc_threshold_off) || (PIR2bits.VINIF == 1))
	{
		/* Attempt a reset */
		PIR2bits.VINIF = 0;
		return 1;
	}
    return 0;
}

#ifdef ENABLE_STATUS_LEDS
/************************************************************************************/
/* This board only has two LEDs so we are going to set up a Truth table as follows: */
/* State    LED #1:     LED #2:                                                     */
/* Charging ON          OFF                                                         */
/* Complete ON          ON                                                          */
/* Fault    OFF         ON                                                          */
/* Stopped  OFF         OFF                                                         */
/* If you want to use TP6 (GPA2) and TP8 (GPA1) for LED3 and LED4 respectively then */
/* replace the function below with the reference code at the top of this file in the*/
/* comments.  You could also deadbug both an LED and bias resistor onto R32 and R36.*/
/************************************************************************************/
void update_status_leds(void)
{
    // Charger Off
	if(cd.charger_state == CHARGER_OFF & cd.status.complete != 1)		
	{	
        if((cd.status.word & 0x7FFF) != 0)                         // These shutdown causes are not defined yet
        {
            LED1_PIN = 0;						
            LED2_PIN = 0;
        }						
    // Fault
        else
        {
            LED1_PIN = 0;
            LED2_PIN = 1;
        }
	}
    // Charger Complete
    else if (cd.status.complete == 1)
    {
        LED1_PIN = 1;
        LED2_PIN = 1;
    }
    // Charge Active
    else
    {
        if (LED_TMR-- == 0) 
        {
            if (bLED_On) 
            {
                bLED_On = 0;
                LED1_PIN = 1;
                LED2_PIN = 0;
                LED_TMR = LED_OffTime;
            } 
            else 
            {
                bLED_On = 1;
                LED1_PIN = 0;
                LED2_PIN = 0;
                LED_TMR = LED_OnTime;
            }
        }
    }
}
#endif // end Enable Status LEDS

#ifdef ENABLE_BENCH_TEST_OUTPUT_PIN
void update_buffcon(void) /* for parts with a copy of an analog value on a pin. */
{
    //The Bench Test Output Pin is set here.  Update the analog mux source here.
    //BUFFCON = cd.buffcon_reg & 0x1F;	
    // <editor-fold defaultstate="collapsed" desc="BUFFCON Register Bit Description">
                                        // bit 7-5 MLTPH<2:0> System Configuration Bits
                                        // 000 = Device set as stand-alone unit
                                        // 001 = Device set as multiple output MASTER
                                        // 010 = Device set as multiple output SLAVE
                                        // 011 = Device set as multi-phase MASTER
                                        // 111 = Device set as multi-phase SLAVE
                                        // bit 4-0 ASEL<4:0> Multiplexter Output Control Bit
                                        // 00000 = Voltage proportional to current in the inductor
                                        // 00001 = Error amplifier output plus slope compensation, input to PWM comparator
                                        // 00010 = Input to slope compensation circuitry
                                        // 00011 = Band gap reference
                                        // 00100 = Output voltage reference
                                        // 00101 = Output voltage after internal differential amplifier
                                        // 00110 = Unimplemented
                                        // 00111 = voltage proportional to the internal temperature
                                        // 01000 = Internal ground for current sense circuitry
                                        // 01001 = Output overvoltage comparator reference
                                        // 01010 = Output under voltage comparator reference
                                        // 01011 = Error amplifier output
                                        // 01100 = For a multi-phase SLAVE, error amplifier signal received from MASTER
                                        // 01101 = For multi-phase SLAVE, error signal received from MASTER with gain
                                        // 01110 = Vin Divided down by 1/5
                                        // 01111 = DC Inductor Valley Current
                                        // 11101 = Over Current Reference
                                        // Everything else unimplemented.
                                        // If ATSTCON<BNCHEN> is set, the device is in Bench Test Mode and will put one of these out on GPA0. 
                                        // Be sure to add the Buffer Offset to the measured signal which can be read from memory location 0x2087.
    // </editor-fold>
    cd.user_interface_mode = 3;		// Tell the GUI we are in bench-test mode

}
#endif

void check_button(void)             // This hardware doesn't have a reset button
{
//    extern unsigned short button_cnt;
//    extern bit start;
//    extern bit shutdown;
    
}

void hardware_custom_a(void)
{
    //Placeholder for any custom code.
    //!!DAN  added to debug bat enable
    if(VBAT_EN)
        LED4_PIN = 1;
    else
        LED4_PIN = 0;
}

void connect_battery(void)
{
    /* Connect Battery Switch */
    VBAT_EN = 1;
}

void disconnect_battery(void)
{
    /* PWM off: We don't want the charger running when we disconnect the battery or bad things will happen */
	ATSTCONbits.DRVDIS = 1;
    
    /* Disconnect Battery Switch */
    VBAT_EN = 0;
}



#endif	// end of MCP19119 Configuration
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="MCP19123_BUCK_BOOST_CHARGER">
#ifdef MCP19123_BUCK_BOOST_CHARGER
char vh, vl;
extern struct charger_settings_t cs;	// Get access to Charger_Settings structure
extern struct charger_data_t cd;		// Get access to Charger_Data structure

void interrupt isr()
{
    if (VINCONbits.UVLOOUT == 1 ) // UVLO flag set?
	{
        /* Disable the interrupt */
        PIE2bits.UVIE = 0;

        /* PWM off */
        PE1bits.LODIS = 1;
        PE1bits.HIDIS = 1;
        
		// log the reason 
		cd.status.vinuvlo = 1;
    }
    if (VINCONbits.OVLOOUT == 1 ) // OVLO flag set?
    {
        /* Disable the interrupt */
        PIE2bits.OVIE = 0;

        /* PWM off */
        PE1bits.LODIS = 1;
        PE1bits.HIDIS = 1;
        
		/* log the reason */
		cd.status.voutovlo = 1;
    }
}

void init_hardware()	/* MCP19123 Hardware specific configuration. */
{
    OSCTUNE = 0x00;                     // Oscillator Tuning Register. Set oscillator to Calibrated Center Frequency
    // <editor-fold defaultstate="collapsed" desc="OSCTUNE Register bit Description">
                                        //bit 7-5 Unimplemented
                                        //bit 4-0 TUN<4:0> Frequency Tuning Bits
                                        //b01111 = Maximum Frequency.
                                        //b01110 thru b00001 = monotonic ramp in between maximum and center.
                                        //b00000 = Center frequency.  Oscillator Module is running at the calibrated frequency.
                                        //b11111 thru b10001 = monotonic ramp in between center and minimum.
                                        //b10000 = Minimum Frequency
    // </editor-fold>
    OPTION_REG = 0x00;                  // Prescaler (bits 2:0) set to 000 for 1:2 TMR0 Rate, 256usec per timer overflow.  
    // <editor-fold defaultstate="collapsed" desc="OPTION_REG Register Description">
                                        // bit 7 RAPU# Port GPx Pull_up Enable bit, 1=Disabled, 0=Enabled
                                        // bit 6 INTEDG Interrupt Edge Select bit, 0=Interrupt on rising edge of INT pin, 1=falling edge
                                        // bit 5 T0CS TMR0 Clock Source Select bit, 1=Transition on T0CKI pin, 0=Internal instruction cycle clock
                                        // bit 4 T0SE TMR0 Source Edge Select bit, 1=Increment on high to low transition on T0CKI pin, 0=low to high
                                        // bit 3 PSA Prescaler Assignment bit, 1=Prescalar is assigned to WDT, 0=Prescaler is assigned to Timer0 module
                                        // bit 2-0 PS<2:0> Prescalar Rate Select Bits:
                                        //  000 - 1:2 TMR0 Rate 1:1 WDT Rate
                                        //  001 - 1:4 TMR0 Rate 1:2 WDT Rate
                                        //  010 - 1:8 TMR0 Rate 1:4 WDT Rate
                                        //  011 - 1:16 TMR0 Rate 1:8 WDT Rate
                                        //  100 - 1:32 TMR0 Rate 1:16 WDT Rate
                                        //  101 - 1:64 TMR0 Rate 1:32 WDT Rate
                                        //  110 - 1:128 TMR0 Rate 1:64 WDT Rate
                                        //  111 - 1:256 TMR0 Rate 1:128 WDT Rate
    // </editor-fold>
    PIR1    = 0;                        // Peripheral Interrupt Flag Register 1. All Interrupts Cleared.
    // <editor-fold defaultstate="collapsed" desc="PIR1 Register Bit Description">
                                        // bit 7 TMR1GIF Timer1 GAte Interrupt Flag bit, 1=Interrupt is pending, 0=not pending
                                        // bit 6 ADIF ADC Interrupt Flag bit, 1=ADC Conversion Complete, 0=Conversion not complete or not started
                                        // bit 5 BCLIF MSSP Bus Collision Interrupt Flag bit, 1=Interrupt is pending, 0=Interrupt is not pending
                                        // bit 4 SSPIF Synchronous Serial Port (MSSP) Interrupt Flag bit, 1=Interrupt is pending, 0=Interrupt not pending.
                                        // bit 3 CC2IF Capture2/Compare2 Interrupt Flag bit, 1=Capture or Compare has occurred, 0=not occurred
                                        // bit 2 CC1IF Capture1/Compare1 Interrupt Flag bit, 1=Capture or Compare has occurred, 0=not occurred
                                        // bit 1 TMR2IF Timer2 to PR2 Match Interrupt Flag, 1=Timer2 to PR2 match occurred (must be cleared in SW), 0=Match did not occur
                                        // bit 0 TMR1IF Timer1 Interrupt Flag.  1=Timer1 rolled over (must be cleared in SW). 0=Timer 1 has not rolled over.
    // </editor-fold>
    PIE1    = 0;                        // Peripheral Interrupt Enable Register 1. All Interrupts Disabled.
    // <editor-fold defaultstate="collapsed" desc="PIE1 Register Bit Description">
                                        // bit 7 TMR1GIE Timer1 Gate Interrupt Enable bit, 1-Enables the Timer1 gate interrupt, 0=disables
                                        // bit 6 ADC Interrupt Enable bit, 1=Enables ADC Interrupt, 0=Disables
                                        // bit 5 BCLIE MSSP Bus Collision Interrupt Enable bit, 1=Enables the MSSP bus Collision Interrupt, 0=Disables
                                        // bit 4 SSPIE synchronous Serial Port (MSSP) Interrupt Enable bit, 1=Enables MSSP Bus Collision Interrupt, 0=Disables
                                        // bit 3 CC2IE Capture2/Compare2 Interupt Enable bit, 1=Enables the Capture2/Compare2 interrupt, 0=disables
                                        // bit 2 CC1IE Capture1/Compare1 Interrupt Enable bit, 1=Enables the capture1/compare1 interrupt, 0=disables
                                        // bit 1 TMR2IE Timer2 Interrupt Enable, 1=Enables the Timer2 Interrupt, 0=Disables
                                        // bit 0 TMR1IE Timer1 Interrupt Enable, 1=Enables the Timer1 Interrupt, 0=Disables
    // </editor-fold>
    PIR2    = 0;                        // Peripheral Interrupt Flag Register 2.  All interrupts Cleared.
    // <editor-fold defaultstate="collapsed" desc="PIR2 Register Bit Description">
                                        // bit 7 UVIF Output undervoltage error interrupt flag bit, 1=Output Undervoltage error has occurred, 0=Has not occured
                                        // bit 6 OTIF Overtermperature interupt flag bit, 1=overtermperature error has occurred, 0=has not occurred
                                        // bit 5 OCIF Output overcurrent error interrupt flag bit, 1=Output OC error has occurred, 0=Has not occured
                                        // bit 4 OVIF Output overvoltage error interrupt flag bit, 1=Output OV error has occurred, 0=Has not occured
                                        // bit 3-2 Unimplemented, Read as 0
                                        // bit 1 OVLOIF Vin overvoltage lock interrupt flag bit
                                        //      When OVLOINTP bit is set 1= A Vin NOT overvoltage to VIN overvoltage edge has been detected, 0=has not been detected
                                        //      When OVLOINTN bit is set 1= A Vin overvoltage to Vin NOT overvoltage edge has been detected, 0=has not been detected
                                        // bit 0 UVLOIF Vin Undervoltage Lock Out Interrupt flag bit
                                        //      When UVLOINTP bit is set 1= A Vin NOT undervoltage to Vin undervoltage edge has been detected, 0=has not been detected
                                        //      When UVLOINTN bit is set 1= A Vin undervoltage to VIN NOT undervoltage edge has been detected, 0=has not been detected
    // </editor-fold>
    PIE2    = 0x00;//%%BRYAN%% 0x03                     // Peripheral Interrupt Enable Register 2. Interrupts for input UVLO and OVLO enabled.
    // <editor-fold defaultstate="collapsed" desc="PIE2 Register Bit Description">
                                        // bit 7 UVIE Output Under voltage Interrupt enable bit, 1=Enables teh UV interrupt, 0=Disables
                                        // bit 6 OTIE Overtemperature Interrupt enable bit, 1=Enables over temperature interrupt, 0=Disables
                                        // bit 5 OCIE Output Overcurrent Interrupt enable bit, 1=Enables the OC Interrupt, 0=Disables
                                        // bit 4 OVIE Output Overvoltage Interrupt enable bit, 1=Enables the OV Interrupt, 0=Disables
                                        // bit 3-2 Unimplemented, Read as 0
                                        // bit 1 OVLOIE Vin Overvoltage Lock Out Interrupt Enable bit, 1=Enables the Vin OVLO Interrupt, 0=Disables
                                        // bit 0 UVLOIE Vin Undervoltage Lock Out Interrupt Enable bit, 1=Enables the Vin UVLO Interrupt, 0=Disables
    // </editor-fold>
    INTCON  = 0x00;//%%BRYAN%% 0xc0                     // Global + Peripheral interrupts.  Enable all Unmasked Interrupts and Unmasked Peripheral Interrupts.  All the rest are disabled.
    // <editor-fold defaultstate="collapsed" desc="INTCON Register Bit Description">
                                        // bit 7 GIE Global Interrupt Enable bit, 1=Enables all unmasked interrupts, 0=Disables
                                        // bit 6 PEIE Peripheral Interrupt Enable bit, 1=Enables all unmasked peripheral interrupts, 0=Disables
                                        // bit 5 T0IE TMR0 Overflow Interrupt Enable bit, 1=Enables the TMR0 Interrupt, 0=Disables
                                        // bit 4 INTE The INT External Interrupt Enable bit, 1=Enables the INT external interrupt. 0=Disables
                                        // bit 3 IOCE Interrupt on Change Enable bit, 1=Enables the interrupt-on-change interrupt, 0=Disables
                                        // bit 2 T0IF TMR0 Overflow Interrupt Flag big, 1=TMR0 register has overflowed (must be cleared in software), 0=TMR0 did not overflow
                                        // bit 1 INTF External Interrupt Flag bit. 1=External Interrupt Occurred (must be cleared in software), 0=External Interrupt did not occur
                                        // bit 0 IOCF Interrupt on Change Interrupt Flag bit, 1=When at least one of the IOC pins changed state, 0=None have changed state
    // </editor-fold>
    IOCA = 0x00;                        // Interrupt on Change PortGPA Register.  Disabled on all Port A pins.  Setting a bit also requires bit 3 of INTCON be set to enable interrupt on change.
    // <editor-fold defaultstate="collapsed" desc="IOCA Register Bit Description">
                                        // bit 7-6 Interrupt on Change PortGPA Register bits, 1=enabled, 0=disabled
                                        // bit 5 Interupt on Change PortGPA Register bits, 1=enabled, 0=disabled
                                        // bit 4-0 Interrupt on Change PortGPA Register bits, 1=enabled, 0=disabled
    // </editor-fold>
    //%%BRYAN%% Investigate the pro/con of Adding the GPB3 button as an Interrupt on Change and add it to ISR routine instead of relying on a loop.
    IOCB = 0x00;                        // Interrupt on Change PortGPB Register.  Disabled on all Port B pins.  Setting a bit also requires bit 3 of INTCON be set to enable interrupt on change.
    // <editor-fold defaultstate="collapsed" desc="IOCB Register Bit Description">
                                        // bit 7-0 Interrupt on Change PortGPA Register bits, 1=enabled, 0=disabled
    // </editor-fold>
    ADCON0  = ADC_MUX_VIN | 0x01;       // ADC Control Register 0.  Enables the ADC and sets the default input channel to VIN (001011)
    // <editor-fold defaultstate="collapsed" desc="ADCON0 Register Bit Description">
                                        // bit 7:2 CHS<5:0> Analog Channel Selected bits
                                        // 000000 = GPA0
                                        // 000001 = GPA1
                                        // 000010 = GPA2
                                        // 000011 = GPA3
                                        // 000100 = GPB1
                                        // 000101 = GPB2
                                        // 000110 = GPB4
                                        // 000111 = GPB5
                                        // 001000 = RELEFF
                                        // 001001 = Ratio of input voltage (Vin/16)
                                        // 001010 = Output voltage measured after differential amplifier
                                        // 001011 = Vref_reg
                                        // 001100 = Error amplifier output
                                        // 001101 = Average current after Sample and Hold and gain trim
                                        // 001110 = Isense signal after gain and slope compensation signal
                                        // 001111 = Average output current with +6dB gain added
                                        // 010000 = Internal temperature sensor
                                        // 010001 = Band gap voltage reference
                                        // 010010 = Vref_rep; Center of the two Vout floating references
                                        // 010011 = Output under voltage comparator reference
                                        // 010100 = Output over voltage comparator reference
                                        // 010101 = Input under voltage comparator reference
                                        // 010110 = Input over voltage compartor reference
                                        // 010111 = Over current reference
                                        // 011000 = Master's current sense signal input (measured on slave unit))
                                        // 011001 = V_BGR_REP; DAC reference voltage amplifier output
                                        // 011010 = GPA3 Buffered
                                        // 011011 = Internal virtual ground (~500mV)
                                        // 011100 = RAWI after Sample & Hold and added gain, but before slope is added
                                        // 011101 = Error Amplifier output after the clamp, input to the PWM comparator
                                        // 011110 = Raw Isense (input to Sample & Hold)
                                        // 011111 = Error Amplifier clamp reference level shifted by 500mV
                                        // 111111 = Unimplemented
                                        // bit 1, GO/Done#, 1=ADC in Progress, 2=complete or not-in-progress
                                        // bit 0, ADON, 1=Enable ADC, 0=Disable ADC
                                        // If BUFFCON<BNCHEN> is set, the device is in Bench Test Mode and will put one of these out on GPA0. 
                                        // Be sure to add the Buffer Offset to the measured signal which can be read from memory location 0x2087.
                                        // When BNCHEN and DIGOEN are both set, then DIGOEN takes priority.
    // </editor-fold>
    ADCON1  = 0x27;                     // ADC Control Register 1.  Enable F_OSC / 32 which sets 4usec per conversion. External VREF used. Right Justified.
    // <editor-fold defaultstate="collapsed" desc="ADCON1 Register Bit Description">
                                        // bit 7 Unimplemented, Read as 0
                                        // bit 6-4 ADCS<2:0> ADC Clock Select Bits
                                        // 000 = Reserved
                                        // 001 = F_OSC/8
                                        // 010 = F_OSC/32
                                        // x11 = F_RC (clock derived from internal oscillator with a divisor of 16)
                                        // 100 = Reserved
                                        // 101 = F_OSC/16
                                        // 111 = F_OSC/64
                                        // bit 3 Unimplemented, Read as 0
                                        // bit 2 ADFM: A/D Result Format Select, 1=Right justified, 0=Left justified
                                        // bit 1-0 VCFG<1:0> A/D Voltage Reference Bit
                                        //      11 = ADref pin
                                        //      10 = ADref pin
                                        //      01 = Internal Vdd Reference
                                        //      00 = Internal A/D Reference
    // </editor-fold>
	PORTGPA = 0b00000000;               // Clear port A to start
    // <editor-fold defaultstate="collapsed" desc="PORTGPA Register Bit Description">
                                        // bit 7 GPA7 General Purpose Open Drain I/O Pin
                                        // bit 6 GPA6 General Purpose I/O Pin, 1=Port pin is > VIH, 0=Port Pin is < VIL
                                        // bit 5 GPA5/MCLR# General Purpose Open Drain I/O pin
                                        // bit 4 GPA4 General Purpose Open Drain I/O pin
                                        // bit 3-0 GPA<3:0> General Purpose I/O Pin, 1=Port Pin is > VIH, 0=Port Pin is < VIL
    // </editor-fold>

#ifdef ENABLE_BENCH_TEST_OUTPUT_PIN //%%BRYAN%% this is going to need to be rewritten I think for the analog and digital modes.
	BUFFCON = 0b10000000;               // Unity Gain Buffer Control Register. Configured as Stand-Alone Unit and Buffer puts out Digital or Analog signal.
    // <editor-fold defaultstate="collapsed" desc="BUFFCON Register Bit Description">
                                        // bit 7 BNCHEN - GPA0 analog multiplexer configuration control bit, 1=GPA0 is configured to be analog multiplexer output, 0=normal operation
                                        // bit 6 DIGOEN - GPA0 digital multiplexer configuration control bit, 1=configured to be digital multiplexer output, 0=normal operation
                                        // bit 5 Unimplemented Read as 0
                                        // bit 4-0 DSEL<4:0> Multiplexer output control bit
                                        //      00000 = 50% period signal
                                        //      00001 = System Clock
                                        //      00010 = Inductor current Sample signal
                                        //      00011 = OV Comparator Output
                                        //      00100 = UV Comparator Output
                                        //      00101 = OVLO Comparator Output
                                        //      00110 = UVLO Comparator Output
                                        //      00111 = OC Comparator Output
                                        //      01000 = High_on Signal
                                        //      01001 = Low-side on signal before the delay block
                                        //      10000 = Output of PWM comparator 
                                        //      10001 = SWFRQ Signal 
                                        //      10010 = T2_EQ_PR2 Signal 
                                        //      10011 = PWM_OUT Signal   
                                        //      10100 = Clock Select / Switchover Waveform
                                        //      10101 = DEM Comparator Output
                                        //      10110 = DEM Blanking Time
                                        //      10111 = Auto Zero Time Or'ed Signal
                                        // Everything else unimplemented.
                                        // If BUFFCON<BNCHEN> is set, the device is in Bench Test Mode and will put one of these out on GPA0. 
                                        // Be sure to add the Buffer Offset to the measured signal which can be read from memory location 0x2087.
                                        // When BNCHEN and DIGOEN are both set, then DIGOEN takes priority.
    // </editor-fold> 
    TRISGPA = 0b10101110;               // Port A Tristate Register.  A7, A5, A3, A2, A1 set as Input.  LED outputs are on GPA6,GPA4 active high. BUFFCON output is on GPA0.
    // <editor-fold defaultstate="collapsed" desc="TRISGPA Register Bit Description">
                                        // bit 7-6 TRISA<7:6> PORTGPA Tri-State Control bit, 1=input (tri-stated), 0=output
                                        // bit 5 TRISA5 GPA5 Tri-State Control bit. This bit is always 1 as GPA5 is an input only
                                        // bit 4-0 TRISA<4:0> PORTGPA Tri-State Control Bit, 1=input (tri-state), 0=output
    // </editor-fold>
	ANSELA  = 0b00001111;               // Port A Analog Select Register.  A1 thru A3 are Analog inputs.  Benchtest requires GPA0 be configured for analog.
    // <editor-fold defaultstate="collapsed" desc="ANSELA Register Bit Description">
                                        // bit 7-4 Unimplemented, Read as 0
                                        // bit 3-0 Digital I/O. Pin is assigned to port or special function.
    // </editor-fold>
    WPUGPA  = 0b00100000;               // Port A Weak Pull-Up Register for digital inputs (A7, A5, A4 are open-drain). A5 (MCLR#) has a weak pullup. A4, A6 have external pullups.
    // <editor-fold defaultstate="collapsed" desc="WPUGPA Register Bit Description">
                                        // bit 7-6 Unimplemented, Read as 0
                                        // bit 5 WPUA5 Weak Pull-Up Register bit, 1=Pull-up enabled, 0=Disabled
                                        // bit 4 Unimplemented, Read as 0
                                        // bit 3-2 WCS<1:0> Weak current source bit, 1=pull-up enabled, 0=disabled
                                        // bit 1-0 WPUA<1:0> Weak Pull-up Register bit, 1=pull-up enabled, 0=disabled
                                        // NOTE: The weak pull-up device is enabled only when the global RAPU# bit is enabled, the pin is in input mode (TRISGPA=1),
                                        //       the individual WPUA bit is enabled (WPUA=1), and the pin is not configured as an analog input.
    // </editor-fold>

#else
	BUFFCON = 0b00000000;               // Unity Gain Buffer Control Register. Configured as Stand-Alone Unit. Bench test enabled
    // <editor-fold defaultstate="collapsed" desc="BUFFCON Register Bit Description">
                                        // bit 7 BNCHEN - GPA0 analog multiplexer configuration control bit, 1=GPA0 is configured to be analog multiplexer output, 0=normal operation
                                        // bit 6 DIGOEN - GPA0 digital multiplexer configuration control bit, 1=configured to be digital multiplexer output, 0=normal operation
                                        // bit 5 Unimplemented Read as 0
                                        // bit 4-0 DSEL<4:0> Multiplexer output control bit
                                        //      00000 = 50% period signal
                                        //      00001 = System Clock
                                        //      00010 = Inductor current Sample signal
                                        //      00011 = OV Comparator Output
                                        //      00100 = UV Comparator Output
                                        //      00101 = OVLO Comparator Output
                                        //      00110 = UVLO Comparator Output
                                        //      00111 = OC Comparator Output
                                        //      01000 = High_on Signal
                                        //      01001 = Low-side on signal before the delay block
                                        //      10000 = Output of PWM comparator 
                                        //      10001 = SWFRQ Signal 
                                        //      10010 = T2_EQ_PR2 Signal 
                                        //      10011 = PWM_OUT Signal   
                                        //      10100 = Clock Select / Switchover Waveform
                                        //      10101 = DEM Comparator Output
                                        //      10110 = DEM Blanking Time
                                        //      10111 = Auto Zero Time Or'ed Signal
                                        // Everything else unimplemented.
                                        // If BUFFCON<BNCHEN> is set, the device is in Bench Test Mode and will put one of these out on GPA0. 
                                        // Be sure to add the Buffer Offset to the measured signal which can be read from memory location 0x2087.
                                        // When BNCHEN and DIGOEN are both set, then DIGOEN takes priority.
    // </editor-fold>
//    TRISGPA = 0b10101111;               // Port A Tristate Register.  A7, A5, A3, A2, A1 set as Input.  LED outputs are on GPA6,GPA4 active high. GPA0 set to input.
    TRISGPA = 0b10101110;               // Port A Tristate Register.  A7, A5, A3, A2, A1 set as Input.  LED outputs are on GPA6,GPA4 active high. GPA0 set to input.
    // <editor-fold defaultstate="collapsed" desc="TRISGPA Register Bit Description">
                                        // bit 7-6 TRISA<7:6> PORTGPA Tri-State Control bit, 1=input (tri-stated), 0=output
                                        // bit 5 TRISA5 GPA5 Tri-State Control bit. This bit is always 1 as GPA5 is an input only
                                        // bit 4-0 TRISA<4:0> PORTGPA Tri-State Control Bit, 1=input (tri-state), 0=output
    // </editor-fold>
	ANSELA  = 0b00001110;               // Port A Analog Select Register.  A1 thru A3 are analog inputs.
    // <editor-fold defaultstate="collapsed" desc="ANSELA Register Bit Description">
                                        // bit 7-4 Unimplemented, Read as 0
                                        // bit 3-0 Digital I/O. Pin is assigned to port or special function.
    // </editor-fold>
    WPUGPA  = 0b00100000;               // Port A Weak Pull-Up Register for digital inputs (A7, A5, A4 are open-drain). A5 (MCLR#) has a weak pullup. A4, A6 have external pullups.
    // <editor-fold defaultstate="collapsed" desc="WPUGPA Register Bit Description">
                                        // bit 7-6 Unimplemented, Read as 0
                                        // bit 5 WPUA5 Weak Pull-Up Register bit, 1=Pull-up enabled, 0=Disabled
                                        // bit 4 Unimplemented, Read as 0
                                        // bit 3-2 WCS<1:0> Weak current source bit, 1=pull-up enabled, 0=disabled
                                        // bit 1-0 WPUA<1:0> Weak Pull-up Register bit, 1=pull-up enabled, 0=disabled
                                        // NOTE: The weak pull-up device is enabled only when the global RAPU# bit is enabled, the pin is in input mode (TRISGPA=1),
                                        //       the individual WPUA bit is enabled (WPUA=1), and the pin is not configured as an analog input.
    // </editor-fold>
#endif

	PORTGPB = 0b00000000;               // Clear port B to start (GPB3 not present as a pin on this part)
    // <editor-fold defaultstate="collapsed" desc="PORTGPB Register Bit Description">
                                        // bit 7-0 GPB<7:0> General Purpose I/O Pin bit, 1=Port pin is >VIH, 0=Port Pin is <VIL
    // </editor-fold>
//    TRISGPB = 0b10101111;               // Port B Tristate Register.  B7, B6, B5, B2, B1, B0 set as Input. Button input is GPB3 active high. GPB6 drives battery disconnect. GPB4 is ICSPDAT.
    TRISGPB = 0b10101011;
    // <editor-fold defaultstate="collapsed" desc="TRISGPB Register Bit Description">
                                        // bit 7-0 TRISB<7:0> PORTGPB Tri-State Control Bit, 1=Pin configured as input (tri-stated), 0=pin configured as output
    // </editor-fold>
//    ANSELB  = 0b00000110;               // Port B Analog Select Register. GPB1 and GPB2 are analog inputs for measuring ADC IOUT and Battery Type Divider.
    ANSELB = 0b00000010;
    // <editor-fold defaultstate="collapsed" desc="ANSELB Register Bit Description">
                                        // bit 7-6 Unimplemented, Read as 0.
                                        // bit 5-4 ANSB<5:4> Analog Select PORTGPB Register Bit, 1=Analog input, 0=Digital I/O or Special Function
                                        // bit 3 Unimplemented, Read as 0.
                                        // bit 2-1 ANSB<2:1> Analog Select PORTGPB Register bit, 1=Analog input, 0=Digital I/O or Special Function
                                        // bit 0 Unimplemented, Read as 0.
    // </editor-fold>
    WPUGPB  = 0b00000000;               // Port B Weak Pull-up Register for digital inputs. 
    // <editor-fold defaultstate="collapsed" desc="WPUGPB Register Bit Description">
                                        // bit 7-3 WPUB<7:3> Weak Pull-Up Register bit
                                        // bit 2-1 WPUB<2:1> Weak Pull-Up Register bit
                                        // bit 0 Unimplemented, Read as 0.
                                        // NOTE: The weak pull-up device is enabled only when the global RAPU# bit is enabled, the pin is in input mode (TRISGPB=1),
                                        //       the individual WPUB bit is enabled (WPUB=1), and the pin is not configured as an analog input.
    // </editor-fold>

/* Set up the PWM Generator with Timers. Frequency and Maximum Duty Cycle. */
    
    T1CON   = 0x00;                     // Timer1 Control Register. Timer OFF, Prescalar at 1, Not used in this design
    // <editor-fold defaultstate="collapsed" desc="T1CON Register Bit Description">
                                        // bit 7-6 Unimplemented Read as 0
                                        // bit 5-4 T1CKPS<1:0> Timer2 Clock Prescale Select bit
                                        // 00 = Prescaler is 1:1
                                        // 01 = Prescaler is 1:2
                                        // 10 = Prescaler is 1:4
                                        // 11 = Prescaler is 1:8
                                        // bit 3-2 Unimplemented, Read as 0
                                        // bit 1 TMR1CS Timer1 Clock Source Select Bits 1=timer1 clock source is 8MHz system clock (Fosc), 0=2MHz instruction clock (Fosc/4)
                                        // bit 0 TMR1ON, Timer1 ON bit, 1=Enabled, 0=Disabled
    // </editor-fold>
    TMR1H   = 0x00;                     // Timer1 Time Base set to 0x00.  Not used in this design
    // <editor-fold defaultstate="collapsed" desc="TMR1H Register Bit Description">
                                        // bit 7-0 Holding Register for the most significant byte of the TMR1 Time Base.
    // </editor-fold>
    TMR1L   = 0x00;                     // Timer1 Time Base set to 0x00.  Not used in this design
    // <editor-fold defaultstate="collapsed" desc="TMR1L Register Bit Description">
                                        // bit 7-0 Holding Register for the least significant byte of the TMR1 Time Base.
    // </editor-fold>
    T1GCON  = 0x00;                     // Timer1 Gate Control Register.  Not used in this design
    // <editor-fold defaultstate="collapsed" desc="T1GCON Register Bit Description">
                                        // bit 7, TMR1GE Timer1 Gate Enable bit, If TMR1ON=0, then this is ignored, if TMR1ON=1, 
                                        //              then 1 = Timer1 incrementing is controlled by the Timer1 gate function, 0=Timer1 increments regardless of Timer1 gate function
                                        // bit 6, T1GPOL Timer1 Gate Polarity bit, 1=Timer1 gate is active-high (Timer1 counts when gate is high), 0=Timer1 gate is active low
                                        // bit 5, T1GTM Timer1 Gate Toggle mode bit, 1=Timer1 Gate toggle mode is enabled, 0=disabled and toggle flip-flop is cleared.
                                        //  Note, Timer1 gate flip-flop toggles on every rising edge
                                        // bit 4, T1GSPM Timer1 Gate Single Pulse mode bit, 1=Timer1 Gate Single-Pulse mode is enabled and is controlling Timer1 gate, 0=disabled
                                        // bit 3, T1GGO/DONE#, Timer 1 GAte Single Pulse Acquisition Status bit, 1=Acquisition is ready and waiting, 0=completed or not started
                                        // bit 2, T1GVAL Timer1 Gate Current State bit, indicates the current state of the Timer1 gate that could be provided to TMR1H:TMR1L
                                        //  Note, Unaffected by Timer1 Gate Enable (TMR1GE)
                                        // bit 1-0 T1GSS<1:0> Timer1 Gate Source Select Bit
                                        // 00 = Timer1 gate pin T1G1
                                        // 01 = Timer0 overflow output
                                        // 10 = Timer1 gate pin T1G2
                                        // 11 = UV Comparator Output
    // </editor-fold>
    CCDCON  = 0x00;                     // Capture Compare Module1 and 2.  Disabled.  Not used in this design.
    // <editor-fold defaultstate="collapsed" desc="CCDCON Register Bit Description">
                                        // bit 7-4 CC@M<3:0> Capture/Compare Register Set 2 Mode Select Bits
                                        //      00xx = Capture/Compare off (resets module)
                                        //      0100 = Capture mode: every falling edge
                                        //      0101 = Capture mode: every rising edge
                                        //      0110 = Capture mode: every 4th rising edge
                                        //      0111 = Capture mode: every 16th rising edge
                                        //      1000 = Compare mode: set output on match (CC2IF bit is set)
                                        //      1001 = Compare mode: clear output on match (CC2IF bit is set)
                                        //      1010 = Compare mode: toggle ouput on match (CC2IF bit is set)
                                        //      1011 = Reserved
                                        //      11xx = Compare mode: generate software interrupt on match (CC2IF bit is set, CMP2 pin is unaffected and configured as an I/O port)
                                        //      1111 = Compare mode: trigger special event (CC2IF bit is set); CCD2 does not reset TMR1 and starts an ADC,
                                        //              If the A/D module is enabled.  CMP2 pin is unaffected and configured as an I/O port
                                        // bit 3-0 CC1M<3:0> Capture/Compare Register Set 1 Mode Select Bits
                                        //      00xx = Capture/Compare off (resets module)
                                        //      0100 = Capture mode: every falling edge
                                        //      0101 = Capture mode: every rising edge
                                        //      0110 = Capture mode: every 4th rising edge
                                        //      0111 = Capture mode: every 16th rising edge
                                        //      1000 = Compare mode: set output on match (CC1IF bit is set)
                                        //      1001 = Compare mode: clear output on match (CC1IF bit is set)
                                        //      1010 = Compare mode: toggle ouput on match (CC1IF bit is set)
                                        //      1011 = Reserved
                                        //      11xx = Compare mode: generate software interrupt on match (CC1IF bit is set, CMP1 pin is unaffected and configured as an I/O port)
                                        //      1111 = Compare mode: trigger special event (CC1IF bit is set); CCD1 does not reset TMR1 and starts an ADC,
                                        //              If the A/D module is enabled.  CMP1 pin is unaffected and configured as an I/O port
                                        // NOTE: When a compare interrupt is set, the TMR1 is not reset.  This is different than standard Microchip microcontroller operation.
    // </editor-fold>
    T2CON   = 0x00;                     // Timer2 Control Register. Timer OFF, Prescalar at 1  
    // <editor-fold defaultstate="collapsed" desc="T2CON Register Bit Description">
                                        // bit 7-3 Unimplemented, Read as 0
                                        // bit 2 TMR2ON Timer2 On bit, 1=On, 0=Off
                                        // bit 1-0 T2CKPS<1:0> Timer2 Clock Prescale Select bit
                                        // 00 = Prescaler is 1
                                        // 01 = Prescaler is 4
                                        // 10 = Prescaler is 8
                                        // 11 = Prescaler is 16
    // </editor-fold>
    TMR2    = 0x00;                     // Timer2 Time Base set to 0x00.  This is the target duty cycle.
    // <editor-fold defaultstate="collapsed" desc="TMR2 Register Bit Description">
                                        // bit 7-0 Holding Register for the 9-bit TMR2 Time Base.  Basically the target duty cycle (D = TMR2/PR2)
    // </editor-fold>
    MODECON = 0b01010000;              // System Configuration Control Register, Disable EA clamps, Enable virtual Ground, Disable LDO on SLEEP, Regulate Voltage, Stand Alone unit 
    // <editor-fold defaultstate="collapsed" desc="MODECON Register Bit Description">
                                        // bit 7, CLMPSEL Error Amplifier Clamp Configuration bit, 1=EA Clamp current set by GPA3 voltage, 0=EA Clamp current set by GPA3 voltage and average output current
                                        // bit 6, VGNDEN Virtual Ground Control bit, 1=Virtual Ground enabled, 0=disabled.
                                        // bit 5, BDDEN VDD LDO control bit, 0=VDD LDO is disabled when SLEEP command issued, 1=LDO remains enabled when SLEEP command issued
                                        // bit 4, CNSG Control Signal configuration bit, 0=device set to control system output voltage, 1=set to control system output current
                                        // bit 3, EACLAMP Error Amplifier Clamp control bit, 1=error amplifier output clamp enabled, 0=error amplifier output clamp disabled
                                        // bit 2-0, MSC<2:0> System configuration control bit
                                        //  000 = Device set as a stand alone unit
                                        //  001 = Device set as multiple output Master (GPB3: clock singal out, GPA1: synchronization signal out)
                                        //  010 = Device set as multiple output Slave (GPB3: clock signal in, GPA1: synchronization signal in)
                                        //  011 = Device set as multi-phase Master (GPB3: clock signal out, GPA1: synchronization signal out, GPB1: demand out)
                                        //  100 = Device set as multi-phase Slave (GPB3: clock signal in, GPA1: synchronization signal in, GPB1: demand in)
                                        //  101 = Device set as multiple output Master (GPA1: synchronization signal out)
                                        //  110 = Device set as multiple output Slave (GPA1: synchronization signal in)
                                        //  111 = Unimplemented
    // </editor-fold>       
    PWMPHL  = 0x00;                     // SLAVE Phase shift to be added. Not used and set to 0.  Slave phase shift = PWMPHL * Tosc * (T2 Prescale Value)
    // <editor-fold defaultstate="collapsed" desc="PWMPHL Register Bit Description">
                                        // bit 7-0 Phase shift added to the SLAVE system clock received on GPA1 in a MASTER/SLAVE or multi-phase implementation. 
    // </editor-fold>
    PWMRL   = 0x19;                     // Sets the maximum PWM duty cycle.  Set to 25.  So (25*296kHz)/8MHz = 93.75%
    // <editor-fold defaultstate="collapsed" desc="PWMRL Register Bit Description">
                                        // bit 7-0 PWM Register Low Byte.  Equation is PWM_Duty_Cycle = PWMRL * T_OSC * TMR2_Prescale_Value
                                        // PWMRL can be written to at any time, but it will not be latched into PWMRH until after a match between PR2 and TMR2 occurs.
    // </editor-fold>
    PR2     = 0x1B;                     // Timer2 Period Register.  Set to 300 kHz frequency: 8MHz/300kHz = 26.7. So Set 27 for 296kHz. 
    // <editor-fold defaultstate="collapsed" desc="PR2 Register Bit Description">
                                        // bit 7-0 Timer2 Module Period Register
                                        // This is the PWM period.  The equation is PWM_Period = [(PR2)+1] * T_OSC * T2_Prescale_Value
    // </editor-fold>
    T2CON   = 0x04;                     // Timer 2 Control Register. Timer ON. Prescalar at 1.
    // <editor-fold defaultstate="collapsed" desc="T2CON Register Bit Description">
                                        // bit 7-3 Unimplemented, Read as 0
                                        // bit 2 TMR2ON Timer2 On bit, 1=On, 0=Off
                                        // bit 1-0 T2CKPS<1:0> Timer2 Clock Prescale Select bit
                                        // 00 = Prescaler is 1
                                        // 01 = Prescaler is 4
                                        // 10 = Prescaler is 8
                                        // 11 = Prescaler is 16
    // </editor-fold>
    
/* Analog power supply specific configurations */
    
    PE1         = 0x30;                 // Analog Peripheral Enable 1 Control Register. Disable diode emulation. Disable FET Drivers. This is a *MUST* or the battery could destroy the sync FET.  
    // <editor-fold defaultstate="collapsed" desc="PE1 Register Bit Description">
                                        // bit 7 (DECON) is Diode Emulation Mode. 1(Enabled), 0(Disabled).  Enabled the synchronous rectification will disable when indutor current reaches zero.
                                        // bit 6 (TOPO) Topology selection control bit, 1=Boost topology is selected, 0=Buck topology
                                        // bit 5 (HIDIS) High Side Driver Control bit, 1=High side driver is disabled, 0=enabled
                                        // bit 4 (LODIS) Low Side Driver Control bit, 1=Low side driver is disabled, 0=enabled
                                        // bit 3 (MEASEN) Relative efficiency measurement control bit, 1=Initiate relative efficiency measurement, 0=Not in progress
                                        // bit 2 (SPAN) Relative efficiency ramp measuremetn contorl bit, 1=A/D Channel 0x08h measures the peak fo the RELEFF signal, 0=measure the valley
                                        // bit 1 (UVTEE) is Output Undervoltage Accelerator Enable bit
                                        // bit 0 (OVTEE) is Output Over Voltage Accelerator Enable bit
    // </editor-fold>
    DEADCON     = 0x8D;                 // Driver Dead time Control Register.  46nS high-side dead time, 48nS low-side dead time 
    // <editor-fold defaultstate="collapsed" desc="DEADCON Bit Descriptions">
                                        // bit 7-4 HDLY<3:0> High-Side Dead Tiem Configuration Bits
                                        // 0000 = 14nsec delay
                                        // 0001 = 18nsec delay
                                        // 0010 = 22nsec delay
                                        // 0011 = 26nsec delay
                                        // 0100 = 30nsec delay
                                        // 0101 = 34nsec delay
                                        // 0110 = 38nsec delay
                                        // 0111 = 42nsec delay
                                        // 1000 = 46nsec delay
                                        // 1001 = 50nsec delay
                                        // 1010 = 54nsec delay
                                        // 1011 = 58nsec delay
                                        // 1100 = 62nsec delay
                                        // 1101 = 66nsec delay
                                        // 1110 = 70nsec delay
                                        // 1111 = 74nsec delay
                                        // bit 3-0 LDLY<3:0> Low-Side Dead Tiem Configuration Bits
                                        // 0000 = -4nsec delay
                                        // 0001 = 0nsec delay
                                        // 0010 = 4nsec delay
                                        // 0011 = 8nsec delay
                                        // 0100 = 12nsec delay
                                        // 0101 = 16nsec delay
                                        // 0110 = 20nsec delay
                                        // 0111 = 24nsec delay
                                        // 1000 = 28nsec delay
                                        // 1001 = 32nsec delay
                                        // 1010 = 36nsec delay
                                        // 1011 = 40nsec delay
                                        // 1100 = 44nsec delay
                                        // 1101 = 48nsec delay
                                        // 1110 = 52nsec delay
                                        // 1111 = 56nsec delay
    // </editor-fold>
    DAGCON      = 0x03;                 // Differential amplifier gain control.  Set to gain of 1/8.  Vrefdac must be less than 1024.  Vrefdac = Vout / (Gain * 0.002)
    // <editor-fold defaultstate="collapsed" desc="DAGCON Bit Descriptions">
                                        // bit 7-3 Unimplemented, read as 0
                                        // bit 2-0 Diff Amp gain value
                                        // 000 = Gain of 1
                                        // 001 = Gain of 1/2
                                        // 010 = Gain of 1/4
                                        // 011 = Gain of 1/8
                                        // 100 = Gain of 2
                                        // 101 = Gain of 4
                                        // 110 = Gain of 8
                                        // 111 = Gain of 1 //%%BRYAN%% check this out with the product line.  Is it really 1 again?  Or 16?
    // </editor-fold>
    CMPZCON     = 0xAA;                 // Compensation Setting Control Register. Zero Set to 12.2kHz. Gain Set to 6dB.
    //<editor-fold defaultstate="collapsed" desc="CMPZCON Register Bit Description">
    
                                        // bit 7-4 CMPZF<3:0> Compensation Zero Frequency Setting Bits
                                        // 0000 = 1500Hz
                                        // 0001 = 1850Hz
                                        // 0010 = 2300Hz
                                        // 0011 = 2840Hz
                                        // 0100 = 3460Hz
                                        // 0101 = 4300Hz
                                        // 0110 = 5300Hz
                                        // 0111 = 6630Hz
                                        // 1000 = 8380Hz
                                        // 1001 = 9950Hz
                                        // 1010 = 12200Hz
                                        // 1011 = 14400Hz
                                        // 1100 = 18700Hz
                                        // 1101 = 23000Hz
                                        // 1110 = 28400Hz
                                        // 1111 = 35300Hz
                                        // bit 3-0 CMPZG<3:0> Compensation Gain Setting Bits
                                        // 0000 = 30.13dB
                                        // 0001 = 27.73dB
                                        // 0010 = 24.66dB
                                        // 0011 = 22.41dB
                                        // 0100 = 20.08dB
                                        // 0101 = 17.78dB
                                        // 0110 = 15.42dB
                                        // 0111 = 13.06dB
                                        // 1000 = 10.75dB
                                        // 1001 = 8.30dB
                                        // 1010 = 6.02dB
                                        // 1011 = 3.52dB
                                        // 1100 = 1.21dB
                                        // 1101 = -1.41dB
                                        // 1110 = -3.74B
                                        // 1111 = -6.02dB
    //</editor-fold>
    CSGSCON     = 0x0F;                 // Current Sense  Gain Control Register.  Additional AC gain = 0dB on top of fixed 30dB gain.
    // <editor-fold defaultstate="collapsed" desc="CSGSCON Register Bit Description">
                                        // bit 7-5 Unimplemented Read as 0
                                        // bit 4-0 CSGS<4:0> Current Sense Gain Setting Bits
                                        // 00000 = -3.0dB
                                        // 00001 = -2.8dB
                                        // 00010 = -2.6dB
                                        // 00011 = -2.4dB
                                        // 00100 = -2.2dB
                                        // 00101 = -2.0dB
                                        // 00110 = -1.8dB
                                        // 00111 = -1.6dB
                                        // 01000 = -1.4dB
                                        // 01001 = -1.2dB
                                        // 01010 = -1.0dB
                                        // 01011 = -0.8dB
                                        // 01100 = -0.6dB
                                        // 01101 = -0.4dB
                                        // 01110 = -0.2dB
                                        // 01111 = 0.0dB
                                        // 10000 = 0.2dB
                                        // 10001 = 0.4dB
                                        // 10010 = 0.6dB
                                        // 10011 = 0.8dB
                                        // 10100 = 1.0dB
                                        // 10101 = 1.2dB
                                        // 10110 = 1.4dB
                                        // 10111 = 1.6dB
                                        // 11000 = 1.8dB
                                        // 11001 = 2.0dB
                                        // 11010 = 2.2dB
                                        // 11011 = 2.4dB
                                        // 11100 = 2.6dB
                                        // 11101 = 2.8dB
                                        // 11110 = 3.0dB
                                        // 11111 = 3.2dB
    // </editor-fold>
    OCCON       = 0xB0;                 // Output Overcurrent Control Register. Enabled, 200ns blanking, 435mV dr op //**BRYAN** Go figure out what to set this to for the evaluation board.
    // <editor-fold defaultstate="collapsed" desc="OCCON Register Bit Description">
                                        // bit 7 OCEN Output Overcurrent DAC Control Bit, 1=enabled, 0=disabled
                                        // bit6-5 OCLEB<1:0> Leading Edge Blanking
                                        // 00 = 110ns blanking
                                        // 01 = 200ns blanking
                                        // 10 = 380ns blanking
                                        // 11 = 740ns blanking
                                        // bit4-0 OOC<4:0> Output Overcurrent Configuration bits
                                        // 00000 = 91mV drop
                                        // 00001 = 112mV drop
                                        // 00010 = 134mV drop
                                        // 00011 = 155mV drop
                                        // 00100 = 177mV drop
                                        // 00101 = 198mV drop
                                        // 00110 = 220mV drop
                                        // 00111 = 241mV drop
                                        // 01000 = 263mV drop
                                        // 01001 = 284mV drop
                                        // 01010 = 306mV drop
                                        // 01011 = 327mV drop
                                        // 01100 = 350mV drop
                                        // 01101 = 370mV drop
                                        // 01110 = 392mV drop
                                        // 01111 = 413mV drop
                                        // 10000 = 435mV drop
                                        // 10001 = 456mV drop
                                        // 10010 = 478mV drop
                                        // 10011 = 500mV drop
                                        // 10100 = 521mV drop
                                        // 10101 = 542mV drop
                                        // 10110 = 563mV drop
                                        // 10111 = 585mV drop
                                        // 11000 = 607mV drop
                                        // 11001 = 628mV drop
                                        // 11010 = 650mV drop
                                        // 11011 = 671mV drop
                                        // 11100 = 693mV drop
                                        // 11101 = 714mV drop
                                        // 11110 = 736mV drop
                                        // 11111 = 757mV drop
    // </editor-fold>
    //%%BRYAN%% This setting needs to be set up to be adjusted by the user configuration cs.uvlo_threshold_off.  The GUI needs to be adjusted for this new register definition here.
    //VINUVLO = cs.uvlo_threshold_off;
    VINUVLO = 0x01;                     // Input Under Voltage Lockout Control Register.  Set to 6V.
    // <editor-fold defaultstate="collapsed" desc="VINUVLO Register Bit Description">
                                        // bit 7-4 Unimplemented Read as 0
                                        // bit 3-0 UVLO<3:0> Under Voltage Lockout Configuration Bits
                                        // 0000 = 4V
                                        // 0001 = 6V
                                        // 0010 = 8V
                                        // 0011 = 10V
                                        // 0100 = 12V
                                        // 0101 = 14V
                                        // 0110 = 16V
                                        // 0111 = 18V
                                        // 1000 = 20V
                                        // 1001 = 22V
                                        // 1010 = 24V
                                        // 1011 = 26V
                                        // 1100 = 28V
                                        // 1101 = 30V
                                        // 1110 = 32V
                                        // 1111 = 34V
    // </editor-fold>
    VINOVLO = 0x0D;                     // Input Over Voltage Lockout Control Register.  Set to 38V.
    // <editor-fold defaultstate="collapsed" desc="VINOVLO Register Bit Description">
                                        // bits 7-4 Unimplemented Read as 0 
                                        // bits 3-0 Overvoltage Lockout Configuration bits
                                        // 0000 = 12V
                                        // 0001 = 14V
                                        // 0010 = 16V
                                        // 0011 = 18V
                                        // 0100 = 20V
                                        // 0101 = 22V
                                        // 0110 = 24V
                                        // 0111 = 26V
                                        // 1000 = 28V
                                        // 1001 = 30V
                                        // 1010 = 32V
                                        // 1011 = 34V
                                        // 1100 = 36V
                                        // 1101 = 38V
                                        // 1110 = 40V
                                        // 1111 = 42V
    // </editor-fold>
    VINCON = 0xAA;                      // Input Under/Over Voltage Control Register. Hardware lockouts enabled for UVLO and OVLO and interrupts enabled on positive going edges of comparators.
    // <editor-fold defaultstate="collapsed" desc="VINCON Register Bit Description">
                                        // bit 7 UVLOEN, UVLO comparator Module Logic Enable bit, 1=UVLO comparator enabled, 0=disabled
                                        // bit 6 UVLOOUT, Under voltage Lock Out Status bit, 1=UVLO event has occurred, 0=has not occurred
                                        // bit 5 UVLOINTP, UVLO Comparator Interrupt On Positive Going Edge Enable bit, 1=UVLOIF will be set upon a positive going edge of UVLO, 0=will not be set
                                        // bit 4 UVLOINTN, UVLO Comparator Interrupt on Negative Going Edge Enable bit, 1=UVLOIF will be set upon a negative going edge of UVLO, 0=will not be set
                                        // bit 3 OVLOEN, OVLO Comparator Module Logic Enable bit, 1=OVLO comparator enabled, 0=disabled
                                        // bit 2 OVLOOUT Overvoltage Lock Out Status bit, 1=OVLO event has occurred, 0=has not occurred
                                        // bit 1 OVLOINTP OVLO Comparator Interrupt on Positive Going Edge Enable bit, 1=OVLOIF will be set upon a positive going edge of the OVLO, 0=will not be set
                                        // bit 0 OVLOINTN OVLO Comparator Interrupt on Negative Going Edge Enable bit, 1=OVLOIF will be set upon a negative going edge of the OVLO, 0=will not be set
    // </editor-fold>
    VOUTL = 0x00;                       // Reference Voltage Configuration LSB.  Start at 0.
    // <editor-fold defaultstate="collapsed" desc="VOUTL Register Bit Description">
                                        // bit 7-0 VOUT<7:0> Reference Voltage Set Point LSB Configuration Bits. 2mV increment
    // </editor-fold>
    VOUTH = 0x00;                       // Reference Voltage Configuration MSB.  Start at 0.
    // <editor-fold defaultstate="collapsed" desc="VOUTH Register Bit Description">
                                        // bits 7-2 Unimplemented Read as 0
                                        // bits 1-0 VOUT<9:8> Reference voltage set point MSB configuration bits
    // </editor-fold>
    VOTUVLO = 0x0F;                     // Output Under Voltage Detect Level Control Register.  Set to 500mV difference, but not used since PE1.UVTEE is not set.
    // <editor-fold defaultstate="collapsed" desc="VOTUVLO Register Bit Description">
                                        // bits 7-4 Unimplemented Read as 0
                                        // bits 3-0 OUV<3:0> Output Under Voltage Detect Level Configuration Bits
                                        // 0000 = Vref - 50mV
                                        // 0001 = Vref - 80mV
                                        // 0010 = Vref - 110mV
                                        // 0011 = Vref - 140mV
                                        // 0100 = Vref - 170mV
                                        // 0101 = Vref - 200mV
                                        // 0110 = Vref - 230mV
                                        // 0111 = Vref - 260mV
                                        // 1000 = Vref - 290mV
                                        // 1001 = Vref - 320mV
                                        // 1010 = Vref - 350mV
                                        // 1011 = Vref - 380mV
                                        // 1100 = Vref - 410mV
                                        // 1101 = Vref - 440mV
                                        // 1110 = Vref - 470mV
                                        // 1111 = Vref - 500mV
    // </editor-fold>
    VOTOVLO = 0x01;                     // Output Over Voltage Detect Level Control Register.  Set to 80mV difference, butn ot used since PE1.OVTEE is not set.
    // <editor-fold defaultstate="collapsed" desc="VOTOVLO Register Bit Description">
                                        // bits 7-4 Unimplemented Read as 0
                                        // bits 3-0 OUV<3:0> Output Under Voltage Detect Level Configuration Bits
                                        // 0000 = Vref + 50mV
                                        // 0001 = Vref + 80mV
                                        // 0010 = Vref + 110mV
                                        // 0011 = Vref + 140mV
                                        // 0100 = Vref + 170mV
                                        // 0101 = Vref + 200mV
                                        // 0110 = Vref + 230mV
                                        // 0111 = Vref + 260mV
                                        // 1000 = Vref + 290mV
                                        // 1001 = Vref + 320mV
                                        // 1010 = Vref + 350mV
                                        // 1011 = Vref + 380mV
                                        // 1100 = Vref + 410mV
                                        // 1101 = Vref + 440mV
                                        // 1110 = Vref + 470mV
                                        // 1111 = Vref + 500mV
    // </editor-fold>
    //RAMPCON = 0x19;                     // Slope Compensation Ramp.  Slope comp enabled  RMP<4:0> = 0x19
    RAMPCONbits.RMPEN = 1;
    RAMPCONbits.RMP = 0x09;
    // <editor-fold defaultstate="collapsed" desc="RAMPCON Register Bit Description">
                                        // bit 7 = RMPEN Compensation Ramp Disable bit.  1 = Disabled, 0 = Enabled
                                        // bits 6-5 = Unimplemented, read as 0
                                        // bits 4-0 = RMP<4:0> Compensation Ramp Configuration bits. RMP<4:0> = (dV/dt * 200/Vin); Where dV/dt is in V/us
    // </editor-fold> 
    vh          = 0;                    // DAC High Byte 
    vl          = 0;                    // DAC Low Byte 
}

void inc_iout(short increment)      // Topology Specific function - Increase Current (Combined)
{
	int i;

  for(i=0;i<increment;i++)			// Loop through the increment routine (if fast_ramp requires)
  {
    vl++;                           // Increment fine register
    if (vl > (IOUT_LSB_ROLL-1)) 		// If we are over the top of the fine register
	{
		if (vh < cs.ovcfcon_max) 
        {	// and the coarse register is not maxed out
            vh ++;					// increment coarse register
			vl = 0;					// and reset fine register to zero
         }
        else 
        {
            vh = cs.ovcfcon_max;	// Set to max
            vl = IOUT_LSB_ROLL;		// We went one increment beyond IOUT_LSB_ROLL, so reset to max.
         }
    }
  }
    VOUTL = vl;                     //Load the DAC with the new values
    VOUTH = vh;
}

short dec_iout( short decrement)	// Topology Specific function - Decrease Current (Combined)
{
	int i;

    if ((vh == 0) && (vl == 0))     // Are we already at the bottom (off)?
        return(0);
	for (i = 0; i<decrement; i++)
	{
		if(vl > 0)					// Must check for zero because 'vl' can roll to FF if not.
			vl--;					// Reduce fine value if we are not already at the bottom of the range

		if (vl == 0) 				// If fine == 0
		{
			if (vh > cs.ovcfcon_min) // and corse > the minimum
			{
				vh--;				 // Then decrease course and reset fine to top of range
				vl = IOUT_LSB_ROLL;	 //
			}
			else
			{
				vh = cs.ovcfcon_min; // Else, we are at the bottom (vl = vf = 0)
				return(0);			 // Return a fault code that we can't decrement any more
			}
		}
	}
    VOUTL = vl;         // Update the setpoint registers
    VOUTH = vh;         //
	return(1);			// Success
}

void zero_iout(void)
{
	/* Output current set to zero */
	vl = 0;
	vh = cs.ovcfcon_min;
	VOUTL = 0;
	VOUTH = 0;
}

void check_button(void)
{
}

void hardware_custom_a(void)
{
}

void disable_charger(void)
{
	/* PWM off */
	PE1bits.LODIS = 1;
    PE1bits.HIDIS = 1;
    
    /* Engage Battery Disconnect Switch */
    PORTBbits.GPB6 = 0;

	/* Adjust UVLO threashold */
	//%%BRYAN%% This needs to be addressed with the new hardware registers to match the GUI.  There's HW comparators WITH hysteresis now.
    //VINUVLO = cs.uvlo_threshold_off;
    VINUVLO = 0x01;

    /* Set output current to minimum */
    zero_iout();
}

void enable_charger(void) //%%BRYAN%% need to add the output overcurrent and overvoltage interrupt enables
{
    /* Set output current to minimum */
    zero_iout();

	/* Adjust UVLO threshold */
	//%%BRYAN%% this needs to be addressed with the new hardware registers to match the GUI.  There's HW comparators WITH hysteresis now.
//    VINUVLO = cs.uvlo_threshold_off;
    VINUVLO = 0x01;
    
	/* UVLO interrupt */
	PIE2bits.UVIE = 1;
    
    /* OVLO interrtup */
    PIE2bits.OVIE = 1;
    
    /* Release Battery Disconnect Switch */
    	PORTBbits.GPB6 = 1;
        __delay_ms(1);
        
    /* Limit Duty Cycle to 1 oscillator period for heavy load conditions*/
        PWMRL = 1;
	/* PWM on */
        VOUT = 0;
        PE1bits.HIDIS = 0;
        PE1bits.LODIS = 0;
        __delay_ms(100);
    /* Restore Duty Cycle to maximum */    
        PWMRL = 0x19; 
}

#ifdef ENABLE_FIXED_CONFIG
#include "MultiChemCharger_Values.h"
#endif

void init_cal_registers(void) //%%BRYAN%% Verify and make use of the read_flash function instead of this repetition
{
    unsigned short addr;
    unsigned short data;
    
    /* Calibration Word 1 */
    addr = 0x20 <<8;                    // Location of the TTA<3:0> = CALWD1<11:8> = TTACAL bits
    addr |= 0x80;                       // and the FCAL<6:0> = CALWD1<6:0> = OSCCAL bits 
    data = read_flash(addr);
    OSCCAL = 0x04; //%%BRYAN%% The cal value is off and giving 10MHz.  This gives 8MHz on the money.
    //OSCCAL = (unsigned short)data;      // Load OSCCAL value from memory (Oscillator tune bits)
    TTACAL = (unsigned short)data>>8;   // Load Device Calibration from memory (Over Temperature Shutdown Threshold Point)
    
    /* Calibration Word 2 */
    addr = 0x20 <<8;                    // Location of the BGT<3:0> = CALWD2<11:8> = BGTCAL bits
    addr |= 0x81;                       // and the BGR<3:0> = CALWD2<3:0> = BGRCAL bits
    data = read_flash(addr);
    BGRCAL = (unsigned short)data;      // Internal band-gap reference calibration
    BGTCAL = (unsigned short)data>>8;   // Internal band gap over temperature calibration
    
    /* Calibration Word 3 */
    addr = 0x20 <<8;                    // Location of the AVDD<3:0> = CALWD3<11:8> = AVDDCAL bits
    addr |= 0x82;                       // Location of the VOUR<4:0> = CALWD3<4:0> = VOURCAL bits
    data = read_flash(addr);
    VOURCAL = (unsigned short)data;     // Calibrate the output overvoltage and undervotlage reference
    AVDDCAL = (unsigned short)data>>8;  // Internal 4.096V bias voltage calibration

    
    /* Calibration Word 4 */
    addr = 0x20 <<8;                    // Location of the DOV<4:0> = CALWD4<12:8> = DOVCAL bits
    addr |= 0x83;                       // Location of the VEAO<4:0> = CALWD4<4:0> = VEAOCAL bits
    data = read_flash(addr);
    VEAOCAL = (unsigned short)data;     // Offset calibration for the error amplifier
    DOVCAL = (unsigned short)data>>8;   // Offset calibration for the output voltage remote sense differential amplifier   
    
   /* Calibration Word 5 */
    addr = 0x20 <<8;                    // Location of the VREF<4:0> = CALWD5<12:8> = VREFCAL bits
    addr |= 0x84;                      
    data = read_flash(addr);
    VREFCAL = (unsigned short)data>>8;  // Calibrates the reference to the DAC that sets the output voltage reference. 
    
    /* Calibration Word 6 */
    addr = 0x20 <<8;                    // Location of the RAMP<4:0> = CALWD6<12:8> = RAMPCAL bits
    addr |= 0x85;                       // Location of the CSR<4:0> = CALWD6<4:0> = CSRCAL bits
    data = read_flash(addr);
    CSRCAL = (unsigned short)data;      // Calibrates the gain of the current sense amplifier
    RAMPCAL = (unsigned short)data>>8;  // Calibrates the span of the slope compensation ramp

    
    /* Calibration Word 7 */
    addr = 0x20 <<8;                    // Location of the UVCO<3:0> = CALWD7<7:4> = OVUVCAL<7:4>
    addr |= 0x86;                       // Location of the OVCO<3:0> = CALWD7<3:0> = OVUVCAL<3:0>
    data = read_flash(addr);
    OVUVCAL = (unsigned short)data;     // OVCO bits contain the output overvotlage comparator offset voltage calibration.  
                                        // UVCO bits are the output undervoltage comparator offset voltage calibration.
    
    /* Calibration Word 8 */
    addr = 0x20 <<8;                    // Location of the DEMOV<4:0> = CALWD8<4:0> = DEMCAL bits
    addr |= 0x87;                      
    data = read_flash(addr);
    DEMCAL = (unsigned short)data;      // Contain the diode emulation mode comparator offset voltage
    
    /* Calibration Word 9 */
    addr = 0x20 <<8;                    // Location of the HCSOV<6:0> = CALWD9<4:0> = HCSOVCAL bits
    addr |= 0x88;                      
    data = read_flash(addr);
//%%BRYAN%% The HCSOVCAL value is wrong from the factory.  This is temporary.  But bench tested to be 0x4A.
    HCSOVCAL = (unsigned short)data;    // Calibration values for the offset voltage on the high-side current sense amplifier
    //HCSOVCAL = 0x40;


    /* The Differential Amplifier loads a different value into VRFSCAL depending on the setting of DAGCON.  DAGCON is the differential amplifier gain setting 
     * and can range from 1/8 to 8.  The VRFSCAL value sets the full scale range of the reference to the DAC that sets the output voltage reference.  The 
     * array below loads the flash addresses for the 7 different settings from Calibration Words 5 and 14 through 16 and whether the required value is in 
     * the high or low byte of that address.  */
    
    const unsigned int GainTranslator[2][8] = {{0x84,0x8E,0x8D,0x8D,0x8E,0x8F,0x8F,0x84},{0,0,1,0,1,0,1,0}};
    addr = 0x20 <<8;
    addr |= GainTranslator[0][DAGCON];
    data = read_flash(addr);
    if (GainTranslator[1][DAGCON]==0)
    {
        VRFSCAL = (unsigned short)data;
    }        
    else
    {
        VRFSCAL = (unsigned short)data>>8;
    }
    
    /* Calibration Words 11 - 13 are not used yet in this firmware*/
    //%%BRYAN%% Experiment with a structure for calibration words (e.g. relocated the cs.tanai and cs.tanam elements) that can be used
    // with ifndef / endif conditions to enable the firmware to correct ADC measurements for maximum accuracy.
}

/* This ENABLE_GUI_CONFIG ifdef is meant to save a little code and prevent flash writes when this device function */
/* was locked down. While it adds the ability to include MultiChemCharger_Values.h it locks out the ability for   */
/* the GUI to change configuration settings. Be aware of this.  If you are testing compiling with the header file */
/* and still want to be able to use the GUI then be sure to set the ENABLE_GUI_CONFIG and ENABLE_FIXED_CONFIG in  */
/* the header file.   */

#ifdef ENABLE_GUI_CONFIG   

/* This function writes 8 bytes (4 flash words) at a time to flash. */
/* The incoming address must be 8-byte aligned.                     */

unsigned short write_flash(unsigned short addr, unsigned short counter) 
{
    unsigned char b = 0;
	if (addr < CAL_BASE_ADDR)           // Invalid address.  Should return a failure result
	{
        return 0x00; 
    }
    if (counter == 8)				// Write data if 8 bytes (4-words) have been received
    {
        while (b < 8) 
        {
            PMADRH = addr >> 8;
            PMADRL = addr;
            PMDATL = flash_write_buf[(unsigned char)(b & 0x07)];
            b++;
            PMDATH = flash_write_buf[(unsigned char)(b & 0x07)];
            b++;
            PMCON1bits.CALSEL = 0;
            PMCON1bits.WREN = 1;
            PMCON2 = 0x55;
            PMCON2 = 0xAA;
            PMCON1bits.WR = 1;
            NOP();
            NOP();
            PMCON1bits.WREN = 0;
            addr++;
        }
        return 0x04;
    }
    return 0x00;
}
#endif

unsigned short read_flash(unsigned short addr)
{
    unsigned short a;

    PMADRH = addr >> 8;
    PMADRL = addr;
    PMCON1bits.CALSEL = 0;
    PMCON1bits.RD = 1;
    asm("nop");
    asm("nop");
    a = (unsigned short)(PMDATH << 8);
    a |= PMDATL;
    return a;
}

unsigned char check_for_uvlo(void)
{
    if ((cd.adc_vin < cs.uvlo_adc_threshold_off) || (PIR2bits.UVLOIF == 1))
	{
		/* Attempt a reset */
		PIR2bits.UVLOIF = 0;
		return 1;
	}
    return 0;
}


#ifdef ENABLE_BENCH_TEST_OUTPUT_PIN
void update_buffcon(void) // for parts with a copy of an analog value on a pin. 
{
    //The Bench Test Output Pin is set here.  Update the analog mux source here.
    //BUFFCON = cd.buffcon_reg;       // Unity Gain Buffer Control Register. Configured as Stand-Alone Unit and Buffer puts out Digital or Analog signal.
    // <editor-fold defaultstate="collapsed" desc="BUFFCON Register Bit Description">
                                        // bit 7 BNCHEN - GPA0 analog multiplexer configuration control bit, 1=GPA0 is configured to be analog multiplexer output, 0=normal operation
                                        // bit 6 DIGOEN - GPA0 digital multiplexer configuration control bit, 1=configured to be digital multiplexer output, 0=normal operation
                                        // bit 5 Unimplemented Read as 0
                                        // bit 4-0 DSEL<4:0> Multiplexer output control bit
                                        //      00000 = 50% period signal
                                        //      00001 = System Clock
                                        //      00010 = Inductor current Sample signal
                                        //      00011 = OV Comparator Output
                                        //      00100 = UV Comparator Output
                                        //      00101 = OVLO Comparator Output
                                        //      00110 = UVLO Comparator Output
                                        //      00111 = OC Comparator Output
                                        //      01000 = High_on Signal
                                        //      01001 = Low-side on signal before the delay block
                                        //      10000 = Output of PWM comparator 
                                        //      10001 = SWFRQ Signal 
                                        //      10010 = T2_EQ_PR2 Signal 
                                        //      10011 = PWM_OUT Signal   
                                        //      10100 = Clock Select / Switchover Waveform
                                        //      10101 = DEM Comparator Output
                                        //      10110 = DEM Blanking Time
                                        //      10111 = Auto Zero Time Or'ed Signal
                                        // Everything else unimplemented.
                                        // If BUFFCON<BNCHEN> is set, the device is in Bench Test Mode and will put one of these out on GPA0. 
                                        // Be sure to add the Buffer Offset to the measured signal which can be read from memory location 0x2087.
                                        // When BNCHEN and DIGOEN are both set, then DIGOEN takes priority.
    // </editor-fold> 
    
    cd.user_interface_mode = 3;		// Tell the GUI we are in bench-test mode

}
#endif

#ifdef ENABLE_STATUS_LEDS
/************************************************************************************/
/* This board has two LEDs.                                                         */
/* LED1 is a fault light on GPA4                                                    */
/* LED2 is a status light on GPA6                                                   */
/************************************************************************************/
void update_status_leds(void)
{
	// LED #2 - Charger Active
	if(cd.charger_state != CHARGER_OFF)		// If the charger is active,
	{
        if (LED_TMR-- == 0) 
        {
            if (bLED_On) 
            {
                bLED_On = 0;
                LED2_PIN = 1;
                LED_TMR = LED_OffTime;
            } 
            else 
            {
                bLED_On = 1;
                LED2_PIN = 0;
                LED_TMR = LED_OnTime;
            }
        }
    }
	else
	{   
    	LED2_PIN = 0;						// Charger Active LED #2 off
        LED_TMR = LED_OffTime;
    }
    	// LED #1 - Fault LED
        if((cd.status.word & 0x7FFF) != 0)       // These shutdown causes are not defined yet
		LED1_PIN = 0;						// Fault LED #1 off

	else									// All other shutdown_causes are true hard-faults
		LED1_PIN = 1;						// Fault LED #1 on

}
#endif // end Enable Status LEDS

void connect_battery(void)
{
    /* Connect Battery Switch */
    //Vbat_EN = 1;
}

void disconnect_battery(void)
{
    /* PWM off: We don't want the charger running when we disconnect the battery or bad things will happen */
	//ATSTCONbits.DRVDIS = 1;
    
    /* Disconnect Battery Switch */
    //Vbat_EN = 0;
}
 #endif	// end of MCP19123 Configuration
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="MCP19125_ADM00745_CHARGER">
#ifdef MCP19125_ADM00745_CHARGER
char vh, vl;
extern struct charger_settings_t cs;	// Get access to Charger_Settings structure
extern struct charger_data_t cd;		// Get access to Charger_Data structure

void interrupt isr()
{
    if (ADIF)
    {
        //TEST_PIN = 0;
        ADIF = 0;
    }
     
    if (UVLOIF)
    //if (VINCONbits.UVLOOUT == 1 ) // UVLO flag set?
	{
        // Disable the interrupt 
        //PIE2bits.UVLOIE = 0;

        // PWM off 
        PE1bits.SDRVEN = 0;
        PE1bits.PDRVEN = 0;
        
		// log the reason 
		cd.status.vinuvlo = 1;
        
        // clear the interrupt
        UVLOIF = 0;
    }
    
    if (OVLOIF)
    //if (VINCONbits.OVLOOUT == 1 ) // OVLO flag set?
    {
        // Disable the interrupt
        //PIE2bits.OVLOIE = 0;

        // PWM off
        PE1bits.SDRVEN = 0;
        PE1bits.PDRVEN = 0;
        
		// log the reason 
	    cd.status.voutovlo = 1;
        OVLOIF = 0;
    }
}

void init_hardware()	/* MCP19125 Hardware specific configuration. */
{
    OSCTUNE = 0x00;                     // Oscillator Tuning Register. Set oscillator to Calibrated Center Frequency
    // <editor-fold defaultstate="collapsed" desc="OSCTUNE Register bit Description">
                                        //bit 7-5 Unimplemented
                                        //bit 4-0 TUN<4:0> Frequency Tuning Bits
                                        //b01111 = Maximum Frequency.
                                        //b01110 thru b00001 = monotonic ramp in between maximum and center.
                                        //b00000 = Center frequency.  Oscillator Module is running at the calibrated frequency.
                                        //b11111 thru b10001 = monotonic ramp in between center and minimum.
                                        //b10000 = Minimum Frequency
    // </editor-fold>
    OPTION_REG = 0x80; // Prescaler (bits 2:0) set to 000 for 1:2 TMR0 Rate, ~256usec per timer overflow. 
    // <editor-fold defaultstate="collapsed" desc="OPTION_REG Register Description">
                                        // bit 7 RAPU# Port GPx Pull_up Enable bit, 1=Disabled, 0=Enabled
                                        // bit 6 INTEDG Interrupt Edge Select bit, 0=Interrupt on rising edge of INT pin, 1=falling edge
                                        // bit 5 T0CS TMR0 Clock Source Select bit, 1=Transition on T0CKI pin, 0=Internal instruction cycle clock
                                        // bit 4 T0SE TMR0 Source Edge Select bit, 1=Increment on high to low transition on T0CKI pin, 0=low to high
                                        // bit 3 PSA Prescaler Assignment bit, 1=Prescalar is assigned to WDT, 0=Prescaler is assigned to Timer0 module
                                        // bit 2-0 PS<2:0> Prescalar Rate Select Bits:
                                        //  000 - 1:2 TMR0 Rate 1:1 WDT Rate
                                        //  001 - 1:4 TMR0 Rate 1:2 WDT Rate
                                        //  010 - 1:8 TMR0 Rate 1:4 WDT Rate
                                        //  011 - 1:16 TMR0 Rate 1:8 WDT Rate
                                        //  100 - 1:32 TMR0 Rate 1:16 WDT Rate
                                        //  101 - 1:64 TMR0 Rate 1:32 WDT Rate
                                        //  110 - 1:128 TMR0 Rate 1:64 WDT Rate
                                        //  111 - 1:256 TMR0 Rate 1:128 WDT Rate
    // </editor-fold>   
    VINCON = 0x00;
    // <editor-fold defaultstate="collapsed" desc="VINCON Register Bit Description">
                                        // bit 7 UVLOEN, UVLO comparator Module Logic Enable bit, 1=UVLO comparator enabled, 0=disabled
                                        // bit 6 UVLOOUT, Under voltage Lock Out Status bit, 1=UVLO event has occurred, 0=has not occurred
                                        // bit 5 UVLOINTP, UVLO Comparator Interrupt On Positive Going Edge Enable bit, 1=UVLOIF will be set upon a positive going edge of UVLO, 0=will not be set
                                        // bit 4 UVLOINTN, UVLO Comparator Interrupt on Negative Going Edge Enable bit, 1=UVLOIF will be set upon a negative going edge of UVLO, 0=will not be set
                                        // bit 3 OVLOEN, OVLO Comparator Module Logic Enable bit, 1=OVLO comparator enabled, 0=disabled
                                        // bit 2 OVLOOUT Overvoltage Lock Out Status bit, 1=OVLO event has occurred, 0=has not occurred
                                        // bit 1 OVLOINTP OVLO Comparator Interrupt on Positive Going Edge Enable bit, 1=OVLOIF will be set upon a positive going edge of the OVLO, 0=will not be set
                                        // bit 0 OVLOINTN OVLO Comparator Interrupt on Negative Going Edge Enable bit, 1=OVLOIF will be set upon a negative going edge of the OVLO, 0=will not be set
    // </editor-fold>
    PIR1    = 0x00;                     // Peripheral Interrupt Flag Register 1. All Interrupts Cleared.
    // <editor-fold defaultstate="collapsed" desc="PIR1 Register Bit Description">
                                        // bit 7-6 Unimplemented - Read as 0
                                        // bit 5 BCLIF MSSP Bus Collision Interrupt Flag bit, 1=Interrupt is pending, 0=Interrupt is not pending
                                        // bit 4 SSPIF Synchronous Serial Port (MSSP) Interrupt Flag bit, 1=Interrupt is pending, 0=Interrupt not pending.
                                        // bit 3 CC2IF Capture2/Compare2 Interrupt Flag bit, 1=Capture or Compare has occurred, 0=not occurred
                                        // bit 2 CC1IF Capture1/Compare1 Interrupt Flag bit, 1=Capture or Compare has occurred, 0=not occurred
                                        // bit 1 TMR2IF Timer2 to PR2 Match Interrupt Flag, 1=Timer2 to PR2 match occurred (must be cleared in SW), 0=Match did not occur
                                        // bit 0 TMR1IF Timer1 Interrupt Flag.  1=Timer1 rolled over (must be cleared in SW). 0=Timer 1 has not rolled over.
    // </editor-fold>
    PIE1    = 0x00;                     // Peripheral Interrupt Enable Register 1. All Interrupts Disabled.
    // <editor-fold defaultstate="collapsed" desc="PIE1 Register Bit Description">
                                        // bit 7-6 Unimplemented, Read as '0'
                                        // bit 5 BCLIE MSSP Bus Collision Interrupt Enable bit, 1=Enables the MSSP bus Collision Interrupt, 0=Disables
                                        // bit 4 SSPIE synchronous Serial Port (MSSP) Interrupt Enable bit, 1=Enables MSSP Bus Collision Interrupt, 0=Disables
                                        // bit 3 CC2IE Capture2/Compare2 Interupt Enable bit, 1=Enables the Capture2/Compare2 interrupt, 0=disables
                                        // bit 2 CC1IE Capture1/Compare1 Interrupt Enable bit, 1=Enables the capture1/compare1 interrupt, 0=disables
                                        // bit 1 TMR2IE Timer2 Interrupt Enable, 1=Enables the Timer2 Interrupt, 0=Disables
                                        // bit 0 TMR1IE Timer1 Interrupt Enable, 1=Enables the Timer1 Interrupt, 0=Disables
    // </editor-fold>
    PIR2    = 0x00;                     // Peripheral Interrupt Flag Register 2.  All interrupts Cleared.
    // <editor-fold defaultstate="collapsed" desc="PIR2 Register Bit Description">
                                        // bit 7 CDSIF - DESAT Detect Comparator Module Interrupt Flag, 1=Interrupt Pending, 0=Not Pending
                                        // bit 6 ADIF - ADC Interrupt Flag, 1=ADC Complete, 0=Not Complete or not started
                                        // bit 5 Unimplmeneted - Read as 0
                                        // bit 4 OTIF Overtermperature interupt flag bit, 1=overtermperature error has occurred, 0=has not occurred
                                        // bit 3 OVIF Overvoltage interrupt flag bit
                                        //      When OVLOINTP bit is set 1= A Vout Not overvoltage to overvoltage edge has been detected, 0=has not been detected
                                        //      When OVLOINTN bit is set 1= A Vout overvoltage to Not overvoltage edge has been detected, 0=has not been detected
                                        // bit 2 DRUVIF Gate Drive Undervoltage Lockout Interrupt Flag, 1=Gate Drive UVLO has occurred, 0=Not occurred
                                        // bit 1 OVLOIF Vin overvoltage lock interrupt flag bit
                                        //      When OVLOINTP bit is set 1= A Vin NOT overvoltage to VIN overvoltage edge has been detected, 0=has not been detected
                                        //      When OVLOINTN bit is set 1= A Vin overvoltage to Vin NOT overvoltage edge has been detected, 0=has not been detected
                                        // bit 0 UVLOIF Vin Undervoltage Lock Out Interrupt flag bit
                                        //      When UVLOINTP bit is set 1= A Vin NOT undervoltage to Vin undervoltage edge has been detected, 0=has not been detected
                                        //      When UVLOINTN bit is set 1= A Vin undervoltage to VIN NOT undervoltage edge has been detected, 0=has not been detected
    // </editor-fold>
    PIE2 = 0x43;                        // Peripheral Interrupt Enable Register 2. ADC, OVLO, UVLO interrupts enabled
    // <editor-fold defaultstate="collapsed" desc="PIE2 Register Bit Description">
                                        // bit 7 CDSIE Desaturation Detection Interrupt enable bit, 1=Enables, 0=Disables
                                        // bit 6 ADIE ADC Interrupt Enable bit, 1=Enables, 0=Disables
                                        // bit 5 Unimplemented, Read as 0
                                        // bit 4 OTIE Over temperature Interrupt enable bit, 1=Enables, 0=Disables
                                        // bit 3 OVIE V_OUT Overvoltage Interrupt Enable bit, 1=Enables, 0=Disables
                                        // bit 2 DRUVIE Gate Drive Undervoltage Lockout Interrupt Enable bit, 1=Enables, 0=Disables
                                        // bit 1 OVLOIE V_IN Overvoltage Lockout Interrupt Enable bit, 1=Enables, 0=Disables
                                        // bit 0 UVLOIE V_IN Undervotlage Lockout Interrupt Enable bit, 1=Enables UVLO, 0=Disables
    // </editor-fold>
    INTCON = 0xC0;                      //Global + Peripheral interrupts.  Enable all Unmasked Interrupts and Unmasked Peripheral Interrupts. 
    // <editor-fold defaultstate="collapsed" desc="INTCON Register Bit Description">
                                        // bit 7 GIE Global Interrupt Enable bit, 1=Enables all unmasked interrupts, 0=Disables
                                        // bit 6 PEIE Peripheral Interrupt Enable bit, 1=Enables all unmasked peripheral interrupts, 0=Disables
                                        // bit 5 T0IE TMR0 Overflow Interrupt Enable bit, 1=Enables the TMR0 Interrupt, 0=Disables
                                        // bit 4 INTE The INT External Interrupt Enable bit, 1=Enables the INT external interrupt. 0=Disables
                                        // bit 3 IOCE Interrupt on Change Enable bit, 1=Enables the interrupt-on-change interrupt, 0=Disables
                                        // bit 2 T0IF TMR0 Overflow Interrupt Flag big, 1=TMR0 register has overflowed (must be cleared in software), 0=TMR0 did not overflow
                                        // bit 1 INTF External Interrupt Flag bit. 1=External Interrupt Occurred (must be cleared in software), 0=External Interrupt did not occur
                                        // bit 0 IOCF Interrupt on Change Interrupt Flag bit, 1=When at least one of the IOC pins changed state, 0=None have changed state
    // </editor-fold>   
    IOCA = 0x00;                        /* Interrupt on Change PortGPA Register.  Disabled on all Port A pins.  GPA5 is disabled if used as MCLR#.
                                         Setting a bit also requires bit 3 of INTCON be set to enable interrupt on change.*/
    // <editor-fold defaultstate="collapsed" desc="IOCA Register Bit Description">
                                        // bit 7-6 Interrupt on Change PortGPA Register bits, 1=enabled, 0=disabled
                                        // bit 5 Interupt on Change PortGPA Register bits, 1=enabled, 0=disabled
                                        // bit 4 Unimplemented Read as 0
                                        // bit 3-0 Interrupt on Change PortGPA Register bits, 1=enabled, 0=disabled
    // </editor-fold>
    IOCB = 0x00;                        // Interrupt on Change PortGPB Register.  Disabled on all Port B pins.  Setting a bit also requires bit 3 of INTCON be set to enable interrupt on change.
    // <editor-fold defaultstate="collapsed" desc="IOCB Register Bit Description">
                                        // bit 7-4 IOCB<7:4> Interrupt on Change PortGPA Register bits, 1=enabled, 0=disabled
                                        // bit 3-2 Unimplemented, Read as 0
                                        // bit 1-0 IOCB<1:0> Interrupt on Change PortGPB register bits, 1=enabled, 0=disabled
    // </editor-fold>
    ADCON0  = ADC_MUX_VIN | 0x01;       // ADC Control Register 0.  Enables the ADC and sets the default input channel to VIN (001011)
    // <editor-fold defaultstate="collapsed" desc="ADCON0 Register Bit Description">
                                        // bit 6:2 CHS<5:0> Analog Channel Selected bits
                                        // 00000 = V_in/n analog voltage measurement (V_IN/15.5328)
                                        // 00001 = V_REF + VZC (DAC reference voltage + VZC pedestal setting current regulation level)
                                        // 00010 = OV_REF (reference for overvotlage comparator)
                                        // 00011 = V_BGR (band gap reference)
                                        // 00100 = V_S (voltage proportional to V_OUT))
                                        // 00101 = EA_SC (error amplifier after slope compensation output)
                                        // 00110 = A2 (secondary current sense amplifier output at R_FB-INT connection)
                                        // 00111 = Pedestal (Pedestal Voltage)
                                        // 01000 = Reserved
                                        // 01001 = Reserved
                                        // 01010 = IP_ADJ (IP after Pedestal and Offset Adjust (at PWM Comparator))
                                        // 01011 = IP_OFF_REF (IP Offset Reference)
                                        // 01100 = V_DR/n (V_DR/n analog driver voltage measurement = 0.229V/V * V_DR)
                                        // 01101 = TEMP_SNS (analog voltage representing internal temperature)
                                        // 01110 = DLL_VCON (Delay Locked Loop Voltage Reference - control voltage for dead time)
                                        // 01111 = SLPCMP_REF (slope compensation reference)
                                        // 10000 = EAOR (OR'd output node from the two error amplifiers EA1 & EA2)
                                        // 10001 = Unimplemented
                                        // 10010 = Unimplemented
                                        // 10011 = Unimplemented
                                        // 10100 = Unimplemented
                                        // 10101 = Unimplemented
                                        // 10110 = Unimplemented
                                        // 10111 = Unimplemented
                                        // 11000 = GPA0/AN0 (i.e. ADDR1)
                                        // 11001 = GPA1/AN1 (i.e. ADDR0)
                                        // 11010 = GPA2/AN2 (i.e. Temperature Sensor Input)
                                        // 11011 = GPA3/AN3 (i.e. BIN)
                                        // 11100 = GPB4/AN4
                                        // 11101 = GPB5/AN6
                                        // 11110 = GPB6/AN6
                                        // 11111 = GPB7/AN7
                                        // bit 1, GO/Done#, 1=ADC in Progress, 0=complete or not-in-progress
                                        // bit 0, ADON, 1=Enable ADC, 0=Disable ADC
                                        // If ABECON<DIGOEN> or ABECON<ANAOEN> is set, the device is in TEST_OUT Mode and will put one of 
                                        // these out on GPA0. Analog signals present on this pin are controlled by the ADCON0 register.  
                                        // Digital signals are controlled by the ABECON register
    // </editor-fold>
    ADCON1  = 0x20;                     // ADC Control Register 1.  Enable F_OSC / 32 which sets 4usec per conversion. AVDD is ADC reference.
    // <editor-fold defaultstate="collapsed" desc="ADCON1 Register Bit Description">
                                        // bit 7 Unimplemented, Read as 0
                                        // bit 6-4 ADCS<2:0> ADC Clock Select Bits
                                        // 000 = Reserved
                                        // 001 = F_OSC/8
                                        // 010 = F_OSC/32
                                        // x11 = F_RC (clock derived from internal oscillator with a divisor of 16)
                                        // 100 = Reserved
                                        // 101 = F_OSC/16
                                        // 110 = F_OSC/64
                                        // bit 3-1 Unimplemented, Read as 0
                                        // bit 0 VCFG A/D Voltage Reference Bit
                                        //      0 = AVDD
                                        //      1 = VDD
    // </editor-fold>
	PORTGPA = 0b00000000;               // Clear port A to start. GPA4 not implemented on this part.
    // <editor-fold defaultstate="collapsed" desc="PORTGPA Register Bit Description">
                                        // bit 7 GPA7 General Purpose I/O Pin, 1=Port pin is > VIH, 0=Port Pin is < VIL
                                        // bit 6 GPA6 General Purpose I/O Pin, 1=Port pin is > VIH, 0=Port Pin is < VIL
                                        // bit 5 GPA5/MCLR#/TEST_EN5 General Purpose Open Drain I/O pin
                                        // bit 4 Unimplemented
                                        // bit 3-0 GPA<3:0> General Purpose I/O Pin, 1=Port Pin is > VIH, 0=Port Pin is < VIL
    // </editor-fold>

#ifdef ENABLE_BENCH_TEST_OUTPUT_PIN //%%BRYAN%% this is going to need to be rewritten I think for the analog and digital modes.
	ABECON = 0b10001000;               // Analog/Digital Test Signal Control. Digital Mux Enabled, OSC selected, Gate UVLO 5.4V, EA's Enabled.
    // <editor-fold defaultstate="collapsed" desc="ABECON Register Bit Description">
                                        // bit 7 DIGOEN - DIG Test MUX to GPA0 Connection Control bit, 1=connected to GPA0, 0=not connected
                                        // bit 6-4 DSEL<2:0> Digital Multiplexer output control bits
                                        //      000 = QRS (output of DESAT comparator)
                                        //      001 = PWM_L (PWM output after monostable)
                                        //      010 = PWM (oscillator output from the micro)
                                        //      011 = TMR2EQ (when TMR2 equals PR2)
                                        //      100 = OV (overvotlage comparator output)
                                        //      101 = SWFRQ (switching frequency output)
                                        //      110 = SDRV_ON_ONESHOT (200ns one-shot signal to reset WDM logic)
                                        //      111 = Unimplemented
                                        // bit 3 DRUVSEL Selectes gate drive undervoltage lockout level, 1=UVLO set to 5.4V, 0=Set to 2.7V
                                        // bit 2 EA2DIS Voltage Error Amplifier Disable bit, 
                                        //              1=Disables the voltage EA (Enables Output OV Comparator Protection)
                                        //              0=Enables the voltage error amplifier
                                        // bit 1 EA1DIS Current Error Amplifier Disable bit, 1=Disables Current EA, 0=Enables EA
                                        // bit 0 ANAOEN Analog MUX Output Control Bit, 1=Analog Mux output connected to GPA0, 0=Not Connected
                                        // If ANAOEN is set, the device is in Bench Test Mode and will mux the Analog channel set in ADCON0 to GPA0. 
                                        // When BNCHEN and DIGOEN are both set, then DIGOEN takes priority.
    // </editor-fold> 
    TRISGPA = 0b10100110;               // Port A Tristate Register.  A7, A5, A2, A1 set as Input.  ABECON output is on GPA0.
    // <editor-fold defaultstate="collapsed" desc="TRISGPA Register Bit Description">
                                        // bit 7-6 TRISA<7:6> PORTGPA Tri-State Control bit, 1=input (tri-stated), 0=output
                                        // bit 5 TRISA5 GPA5 Tri-State Control bit. This bit is always 1 as GPA5 is an input only
                                        // bit 4 Unimplemented, Read as 0
                                        // bit 3-0 TRISA<3:0> PORTGPA Tri-State Control Bit, 1=input (tri-state), 0=output
    // </editor-fold>
	ANSELA  = 0b00000111;               // Port A Analog Select Register.  A1 thru A2 are Analog inputs.  Test Signal requires GPA0 be configured for analog.
    // <editor-fold defaultstate="collapsed" desc="ANSELA Register Bit Description">
                                        // bit 7-4 Unimplemented, Read as 0
                                        // bit 3-0 ANSA<3:0> Analog Select GPA Register Bits, 1=Analog Input, 0=Digital I/O
    // </editor-fold>
    WPUGPA  = 0b00000000;               // Port A Weak Pull-Up Register for digital inputs. A5, A7 have external pullups.
    // <editor-fold defaultstate="collapsed" desc="WPUGPA Register Bit Description">
                                        // bit 7-6 Unimplemented, Read as 0
                                        // bit 5 WPUA5 Weak Pull-Up Register bit, 1=Pull-up enabled, 0=Disabled
                                        // bit 4 Unimplemented, Read as 0
                                        // bit 3-0 WPUA<3:0> Weak Pull-up Register bit, 1=pull-up enabled, 0=disabled
                                        // NOTE: The weak pull-up device is enabled only when the global RAPU# bit is enabled, the pin is in input mode (TRISGPA=1),
                                        //       the individual WPUA bit is enabled (WPUA=1), and the pin is not configured as an analog input.
                                        // NOTE: GPA5 weak pull-up is also enabled when the pin is configured as MCLR# in the CONFIG register.
    // </editor-fold>
    cd.user_interface_mode = 3;		// Tell the GUI we are in bench-test mode
#else
	ABECON = 0b00011000;               // Analog/Digital Test Signal Control. Digital Mux Enabled, PWM_L selected, Gate UVLO 5.4V, EA's Enabled.
    // <editor-fold defaultstate="collapsed" desc="ABECON Register Bit Description">
                                        // bit 7 DIGOEN - DIG Test MUX to GPA0 Connection Control bit, 1=connected to GPA0, 0=not connected
                                        // bit 6-4 DSEL<2:0> Digital Multiplexer output control bits
                                        //      000 = QRS (output of DESAT comparator)
                                        //      001 = PWM_L (PWM output after monostable)
                                        //      010 = PWM (oscillator output from the micro)
                                        //      011 = TMR2EQ (when TMR2 equals PR2)
                                        //      100 = OV (overvotlage comparator output)
                                        //      101 = SWFRQ (switching frequency output)
                                        //      110 = SDRV_ON_ONESHOT (200ns one-shot signal to reset WDM logic)
                                        //      111 = Unimplemented
                                        // bit 3 DRUVSEL Selectes gate drive undervoltage lockout level, 1=UVLO set to 5.4V, 0=Set to 2.7V
                                        // bit 2 EA2DIS Voltage Error Amplifier Disable bit, 
                                        //              1=Disables the voltage EA (Enables Output OV Comparator Protection)
                                        //              0=Enables the voltage error amplifier
                                        // bit 1 EA1DIS Current Error Amplifier Disable bit, 1=Disables Current EA, 0=Enables EA
                                        // bit 0 ANAOEN Analog MUX Output Control Bit, 1=Analog Mux output connected to GPA0, 0=Not Connected
                                        // If ANAOEN is set, the device is in Bench Test Mode and will mux the Analog channel set in ADCON0 to GPA0. 
                                        // When BNCHEN and DIGOEN are both set, then DIGOEN takes priority.
    // </editor-fold> 
    TRISGPA = 0b10100110;               // Port A Tristate Register.  A7, A5, A2, A1 set as Input.  
    // <editor-fold defaultstate="collapsed" desc="TRISGPA Register Bit Description">
                                        // bit 7-6 TRISA<7:6> PORTGPA Tri-State Control bit, 1=input (tri-stated), 0=output
                                        // bit 5 TRISA5 GPA5 Tri-State Control bit. This bit is always 1 as GPA5 is an input only
                                        // bit 4 Unimplemented, Read as 0
                                        // bit 3-0 TRISA<3:0> PORTGPA Tri-State Control Bit, 1=input (tri-state), 0=output
    // </editor-fold>
	ANSELA  = 0b00000110;               // Port A Analog Select Register.  A1 thru A2 are analog inputs.
    // <editor-fold defaultstate="collapsed" desc="ANSELA Register Bit Description">
                                        // bit 7-4 Unimplemented, Read as 0
                                        // bit 3-0 ANSA<3:0> Analog Select GPA Register Bits, 1=Analog Input, 0=Digital I/O
    // </editor-fold>
    WPUGPA  = 0b00000000;               // Port A Weak Pull-Up Register for digital inputs. A5, A7 have external pullups.
    // <editor-fold defaultstate="collapsed" desc="WPUGPA Register Bit Description">
                                        // bit 7-6 Unimplemented, Read as 0
                                        // bit 5 WPUA5 Weak Pull-Up Register bit, 1=Pull-up enabled, 0=Disabled
                                        // bit 4 Unimplemented, Read as 0
                                        // bit 3-0 WPUA<3:0> Weak Pull-up Register bit, 1=pull-up enabled, 0=disabled
                                        // NOTE: The weak pull-up device is enabled only when the global RAPU# bit is enabled, the pin is in input mode (TRISGPA=1),
                                        //       the individual WPUA bit is enabled (WPUA=1), and the pin is not configured as an analog input.
                                        // NOTE: GPA5 weak pull-up is also enabled when the pin is configured as MCLR# in the CONFIG register.
    // </editor-fold>
#endif
	PORTGPB = 0b00000000;               // Clear port B to start (GPB2 and GPB3 not present as a pin on this part), 
    // <editor-fold defaultstate="collapsed" desc="PORTGPB Register Bit Description">
                                        // bit 7-4 GPB<7:4> General Purpose I/O Pin bit, 1=Port pin is >VIH, 0=Port Pin is <VIL
                                        // bit 3-2 Unimplemented, read as 0
                                        // bit 1-0 GPB<1:0> General Purpose I/O Pin bit, 1=Port pin is >VIH, 0=Port Pin is <VIL
    // </editor-fold>
    TRISGPB = 0b01110001;               // Port B Tristate Register.  B7, B6, B5, B2, B1, B0 set as Input. Button input is GPB6 active high. GPB1 is LED.
    // <editor-fold defaultstate="collapsed" desc="TRISGPB Register Bit Description">
                                        // bit 7-4 TRISB<7:4> PORTGPB Tri-State Control Bit, 1=Pin configured as input (tri-stated), 0=pin configured as output
                                        // bit 3-2 Unimplemented, Read as 0
                                        // bit 1-0 TRISB<1:0> PORTGPB Tri-State Control Bit, 1=Pin configured as input (tri-stated), 0=pin configured as output
    // </editor-fold>
    ANSELB  = 0b00000000;               // Port B Analog Select Register. 
    // <editor-fold defaultstate="collapsed" desc="ANSELB Register Bit Description">
                                        // bit 7 Unimplemented, Read as 0.
                                        // bit 6-4 ANSB<6:4> Analog Select PORTGPB Register Bit, 1=Analog input, 0=Digital I/O or Special Function
                                        // bit 3-2 Unimplemented, Read as 0.
                                        // bit 1 ANSB<1> Analog Select PORTGPB Register bit, 1=Analog input, 0=Digital I/O or Special Function
                                        // bit 0 Unimplemented, Read as 0.
    // </editor-fold>
    WPUGPB  = 0b00000000;               // Port B Weak Pull-up Register for digital inputs. 
    // <editor-fold defaultstate="collapsed" desc="WPUGPB Register Bit Description">
                                        // bit 7-4 WPUB<7:4> Weak Pull-Up Register bit, 1=Enabled, 0=Disabled
                                        // bit 3-2 Unimplemented, Read as 0
                                        // bit 1 WPUB<1> Weak Pull-Up Register bit, 1=Enabled, 0=Disabled
                                        // bit 0 Unimplemented, Read as 0.
                                        // NOTE: The weak pull-up device is enabled only when the global RAPU# bit is enabled, the pin is in input mode (TRISGPB=1),
                                        //       the individual WPUB bit is enabled (WPUB=1), and the pin is not configured as an analog input.
    // </editor-fold>

/* Set up the PWM Generator with Timers. Frequency and Maximum Duty Cycle. */
    
    T1CON   = 0x00;                     // Timer1 Control Register. Timer OFF, Prescalar at 1, Not used in this design
    // <editor-fold defaultstate="collapsed" desc="T1CON Register Bit Description">
                                        // bit 7-6 Unimplemented Read as 0
                                        // bit 5-4 T1CKPS<1:0> Timer2 Clock Prescale Select bit
                                        // 00 = Prescaler is 1:1
                                        // 01 = Prescaler is 1:2
                                        // 10 = Prescaler is 1:4
                                        // 11 = Prescaler is 1:8
                                        // bit 3-2 Unimplemented, Read as 0
                                        // bit 1 TMR1CS Timer1 Clock Source Select Bits 1=timer1 clock source is 8MHz system clock (Fosc), 0=2MHz instruction clock (Fosc/4)
                                        // bit 0 TMR1ON, Timer1 ON bit, 1=Enabled, 0=Disabled
    // </editor-fold>
    TMR1H   = 0x00;                     // Timer1 Time Base set to 0x00.  Not used in this design
    // <editor-fold defaultstate="collapsed" desc="TMR1H Register Bit Description">
                                        // bit 7-0 Holding Register for the most significant byte of the TMR1 Time Base.
    // </editor-fold>
    TMR1L   = 0x00;                     // Timer1 Time Base set to 0x00.  Not used in this design
    // <editor-fold defaultstate="collapsed" desc="TMR1L Register Bit Description">
                                        // bit 7-0 Holding Register for the least significant byte of the TMR1 Time Base.
    // </editor-fold>
    CCDCON  = 0x00;                     // Capture Compare Module1 and 2.  Disabled.  Not used in this design.
    // <editor-fold defaultstate="collapsed" desc="CCDCON Register Bit Description">
                                        // bit 7-4 CC@M<3:0> Capture/Compare Register Set 2 Mode Select Bits
                                        //      00xx = Capture/Compare off (resets module)
                                        //      0100 = Capture mode: every falling edge
                                        //      0101 = Capture mode: every rising edge
                                        //      0110 = Capture mode: every 4th rising edge
                                        //      0111 = Capture mode: every 16th rising edge
                                        //      1000 = Compare mode: set output on match (CC2IF bit is set)
                                        //      1001 = Compare mode: clear output on match (CC2IF bit is set)
                                        //      1010 = Compare mode: toggle ouput on match (CC2IF bit is set)
                                        //      1011 = Reserved
                                        //      11xx = Compare mode: generate software interrupt on match (CC2IF bit is set, CMP2 pin is unaffected and configured as an I/O port)
                                        //      1111 = Compare mode: trigger special event (CC2IF bit is set); CCD2 does not reset TMR1 and starts an ADC,
                                        //              If the A/D module is enabled.  CMP2 pin is unaffected and configured as an I/O port
                                        // bit 3-0 CC1M<3:0> Capture/Compare Register Set 1 Mode Select Bits
                                        //      00xx = Capture/Compare off (resets module)
                                        //      0100 = Capture mode: every falling edge
                                        //      0101 = Capture mode: every rising edge
                                        //      0110 = Capture mode: every 4th rising edge
                                        //      0111 = Capture mode: every 16th rising edge
                                        //      1000 = Compare mode: set output on match (CC1IF bit is set)
                                        //      1001 = Compare mode: clear output on match (CC1IF bit is set)
                                        //      1010 = Compare mode: toggle ouput on match (CC1IF bit is set)
                                        //      1011 = Reserved
                                        //      11xx = Compare mode: generate software interrupt on match (CC1IF bit is set, CMP1 pin is unaffected and configured as an I/O port)
                                        //      1111 = Compare mode: trigger special event (CC1IF bit is set); CCD1 does not reset TMR1 and starts an ADC,
                                        //              If the A/D module is enabled.  CMP1 pin is unaffected and configured as an I/O port
                                        // NOTE: When a compare interrupt is set, the TMR1 is not reset.  This is different than standard Microchip microcontroller operation.
    // </editor-fold>
    T2CON   = 0x00;                     // Timer2 Control Register. Timer OFF, Prescalar at 1  
    // <editor-fold defaultstate="collapsed" desc="T2CON Register Bit Description">
                                        // bit 7-3 Unimplemented, Read as 0
                                        // bit 2 TMR2ON Timer2 On bit, 1=On, 0=Off
                                        // bit 1-0 T2CKPS<1:0> Timer2 Clock Prescale Select bit
                                        // 00 = Prescaler is 1
                                        // 01 = Prescaler is 4
                                        // 10 = Prescaler is 8
                                        // 11 = Prescaler is 16
    // </editor-fold>
    TMR2    = 0x00;                     // Timer2 Time Base set to 0x00.  This is the target duty cycle.
    // <editor-fold defaultstate="collapsed" desc="TMR2 Register Bit Description">
                                        // bit 7-0 Holding Register for the 9-bit TMR2 Time Base.  Basically the target duty cycle (D = TMR2/PR2)
    // </editor-fold>
    MODECON = 0b00000000;              // System Configuration Control Register, Set as standalone unit.
    // <editor-fold defaultstate="collapsed" desc="MODECON Register Bit Description">
                                        // bit 7-6, MSC<1:0> System configuration control bit
                                        //  00 = Device set as a stand alone unit (V_REF2 disabled, switching frequency internally generated)
                                        //  01 = Device set as MASTER (V_REF2 to GPB1, CLKOUT sync to GPA1)
                                        //  10 = Device set as SLAVE MODE (CLKIN switching frequency sync signal to GPA1)
                                        //  11 = Device set as SEMI-MASTER MODE
                                        // bit 5-4 Unimplemented, Read as 0
                                        // bit 3 MSC2 Semi-Master Mode Options bit
                                        //  0 = GPB1 is V_REF2 Output, GPA1 is GPIO
                                        //  1 = GPB1 is GPO, GPA1 is CLKOUT
                                        // bit 2-0 Unimplemented, Read as 0
    // </editor-fold>       
    PWMPHL  = 0x00;                     // SLAVE Phase shift to be added. Not used and set to 0.  Slave phase shift = PWMPHL * Tosc * (T2 Prescale Value)
    // <editor-fold defaultstate="collapsed" desc="PWMPHL Register Bit Description">
                                        // bit 7-0 Phase shift added to the SLAVE system clock received on GPA1 in a MASTER/SLAVE or multi-phase implementation. 
    // </editor-fold>
    PWMRL   = 0x13;                     // Sets the maximum PWM duty cycle.  Set to 19.  So (19*296kHz)/8MHz = 70%
    // <editor-fold defaultstate="collapsed" desc="PWMRL Register Bit Description">
                                        // bit 7-0 PWM Register Low Byte.  Equation is PWM_Duty_Cycle = PWMRL * T_OSC * TMR2_Prescale_Value
                                        // PWMRL can be written to at any time, but it will not be latched into PWMRH until after a match between PR2 and TMR2 occurs.
    // </editor-fold>
    PWMRH   = 0x00;                     // Preload Value latched from PWMRL. Leave this at 0.
    PR2     = 0x1A;                     // Timer2 Period Register.  Set to 300 kHz frequency: 8MHz/300kHz - 1 = PR2 = 25.7. So Set 26 for 296kHz. 
    // <editor-fold defaultstate="collapsed" desc="PR2 Register Bit Description">
                                        // bit 7-0 Timer2 Module Period Register
                                        // This is the PWM period.  The equation is PWM_Period = [(PR2)+1] * T_OSC * T2_Prescale_Value
    // </editor-fold>
    T2CON   = 0x04;                     // Timer 2 Control Register. Timer ON. Prescalar at 1. 
    // <editor-fold defaultstate="collapsed" desc="T2CON Register Bit Description">
                                        // bit 7-3 Unimplemented, Read as 0
                                        // bit 2 TMR2ON Timer2 On bit, 1=On, 0=Off
                                        // bit 1-0 T2CKPS<1:0> Timer2 Clock Prescale Select bit
                                        // 00 = Prescaler is 1
                                        // 01 = Prescaler is 4
                                        // 10 = Prescaler is 8
                                        // 11 = Prescaler is 16
    // </editor-fold>
    
/* Analog power supply specific configurations */
    
    PE1         = 0x00;                 // Analog Peripheral Enable 1 Control Register. Disable FET Drivers. This is a *MUST* or the battery could destroy the sync FET.  
    // <editor-fold defaultstate="collapsed" desc="PE1 Register Bit Description">
                                        // bit 7 PDRVEN, 1=PDRV gate drive is enabled, 0=disabled
                                        // bit 6 SDRVEN, 1=SDRV gate drive is enabled, 0=disabled
                                        // bit 5 PDRVBY PDRV Dead Time Bypass bit, 1=Bypassed, 0=Not Bypassed
                                        // bit 4 SDRVBY SDRV Dead Time Bypass bit, 1=Bypassed, 0=Not Bypassed
                                        // bit 3 Unimplemented, Read as 0
                                        // bit 2 ISPUEN I_SP Weak Pull-Up Enable Bit, 1=Enabled, 0=Disabled
                                        // bit 1 PWMSTR_PEN PDRV PWM Steering Bit, 1=Enables open-loop PWM control to PDRV, 0=Disables
                                        // bit 0 PWMSTR_SEN SDRV PWM Steering Bit, 1=Enables open-loop PWM control to SDRV, 0=Disables
    // </editor-fold>
    DEADCON     = 0x77;                 // Driver Dead time Control Register.  46nS high-side dead time, 48nS low-side dead time 
    // <editor-fold defaultstate="collapsed" desc="DEADCON Bit Descriptions">
                                        // bit 7-4 HDLY<3:0> High-Side Dead Tiem Configuration Bits
                                        // 0000 = 14nsec delay
                                        // 0001 = 18nsec delay
                                        // 0010 = 22nsec delay
                                        // 0011 = 26nsec delay
                                        // 0100 = 30nsec delay
                                        // 0101 = 34nsec delay
                                        // 0110 = 38nsec delay
                                        // 0111 = 42nsec delay
                                        // 1000 = 46nsec delay
                                        // 1001 = 50nsec delay
                                        // 1010 = 54nsec delay
                                        // 1011 = 58nsec delay
                                        // 1100 = 62nsec delay
                                        // 1101 = 66nsec delay
                                        // 1110 = 70nsec delay
                                        // 1111 = 74nsec delay
                                        // bit 3-0 LDLY<3:0> Low-Side Dead Tiem Configuration Bits
                                        // 0000 = -4nsec delay
                                        // 0001 = 0nsec delay
                                        // 0010 = 4nsec delay
                                        // 0011 = 8nsec delay
                                        // 0100 = 12nsec delay
                                        // 0101 = 16nsec delay
                                        // 0110 = 20nsec delay
                                        // 0111 = 24nsec delay
                                        // 1000 = 28nsec delay
                                        // 1001 = 32nsec delay
                                        // 1010 = 36nsec delay
                                        // 1011 = 40nsec delay
                                        // 1100 = 44nsec delay
                                        // 1101 = 48nsec delay
                                        // 1110 = 52nsec delay
                                        // 1111 = 56nsec delay
    // </editor-fold>
    //%%BRYAN%% This setting needs to be set up to be adjusted by the user configuration cs.uvlo_threshold_off.  The GUI needs to be adjusted for this new register definition here.
    //VINUVLO = cs.uvlo_threshold_off;
    VINUVLO     = 0x12;                 // Input Undervoltage Lockout Register.  Set to 18.  UVLO(V) = 3.57 * 1.028578^18 = 5.93V
    // <editor-fold defaultstate="collapsed" desc="VINUVLO Register Bit Description">
                                        // bits 7-6 Unimplemented, Read as 0
                                        // bits 5-0 UVLO<5:0> Undervoltage Lockout Configuration Bits, UVLO(V = 3.57 * (1.028578^N), N is the register value
    // </editor-fold>
    VINOVLO     = 0x36;                 // Input Overvoltage Lockout Register. N=54 for OVLO(V)= 34.6V =7.5212 * 1.028645^60
    // <editor-fold defaultstate="collapsed" desc="VINOVLO Register Bit Description">
                                        // bit 7-6 Unimplemented, Read as 0
                                        // bit 5-0 OVLO<5:0> Overvoltage Lockout Configuration 'N'.  OVLO(V) = 7.5212 * (1.028645^N)
    // </editor-fold>
    VINCON      = 0xAA;                 // UVLO and OVLO Comparator Control Register.  Logic Enabled.  Interrupts enabled.
    // <editor-fold defaultstate="collapsed" desc="VINCON Register Bit Description">
                                        // bit 7 UVLOEN - UVLO Comparator Module Logic Enable bit, 1=Enabled, 0=Disabled
                                        // bit 6 UVLOOUT - UVLO Status Output, 1=UVLO event has occurred, 0=No UVLO event has occurred
                                        // bit 5 UVLOINTP - UVLO Comparator Interrupt on Positive Going Edge Enable
                                        //          1 = The UVLOIF interrupt flag will be set upong a positive going edge of the UVLO
                                        //          0 = No UVLOIF interrupt flag will be set upon a positive going edge of the UVLO
                                        // bit 4 UVLOINTN - UVLO Comparator Interrupt on Negative Going Edge Enable bit
                                        //          1 = The UVLOIF interrupt flag will be set upon a negative going edge of the UVLO
                                        //          0 = No UVLOIF interrupt flag will be set upon a negative going edge of the UVLO
                                        // bit 3 OVLOEN - OVLO Comparator Module Logic Enable bit, 1=Enabled, 0=Disabled
                                        // bit 2 OVLOOUT - Overvoltage Lockout Status Output bit, 1=OVLO event has occurred, 0=Not occurred
                                        // bit 1 OVLOINTP - OVLO Comparator Interrupt on Positive Going Edge Enable bit
                                        //          1 = The OVLOIF interrupt flag will be set upon a positive going edge of the OVLO
                                        //          0 = No OVLOIF interrupt flag will be set upon a positive going edge of the OVLO
                                        // bit 0 OVLOINTN - OVLO comparator INterrupt on Negative Going Edge Enable bit
                                        //          1 = The OVLOIF interrupt flag will be set upon a negative going edge of the OVLO
                                        //          0 = No OVLOIF interrupt flag will be set upon a negative going edge of the OVLO
    // </editor-fold>
    DEADCON     = 0x77;                 // Driver dead time control register. 128nsec delay on PDRV and SDRV.
    // <editor-fold defaultstate="collapsed" desc="Dead Time Register Bit Description">
                                        // bit 7-4 PDRVDT <3:0> PDRV Dead Time Configuration bits (t_TD-1)
                                        // 0000 = 16nsec delay
                                        // 0001 = 32nsec delay
                                        // 0010 = 48nsec delay
                                        // 0011 = 64nsec delay
                                        // 0100 = 80nsec delay
                                        // 0101 = 96nsec delay
                                        // 0110 = 112nsec delay
                                        // 0111 = 128nsec delay
                                        // 1000 = 144nsec delay
                                        // 1001 = 160nsec delay
                                        // 1010 = 176nsec delay
                                        // 1011 = 192nsec delay
                                        // 1100 = 208nsec delay
                                        // 1101 = 224nsec delay
                                        // 1110 = 240nsec delay
                                        // 1111 = 256nsec delay
                                        // bit 3-0 SDRVDT <3:0> SDRV Dead Time Configuration bits (t_TD-1)
                                        // 0000 = 16nsec delay
                                        // 0001 = 32nsec delay
                                        // 0010 = 48nsec delay
                                        // 0011 = 64nsec delay
                                        // 0100 = 80nsec delay
                                        // 0101 = 96nsec delay
                                        // 0110 = 112nsec delay
                                        // 0111 = 128nsec delay
                                        // 1000 = 144nsec delay
                                        // 1001 = 160nsec delay
                                        // 1010 = 176nsec delay
                                        // 1011 = 192nsec delay
                                        // 1100 = 208nsec delay
                                        // 1101 = 224nsec delay
                                        // 1110 = 240nsec delay
                                        // 1111 = 256nsec delay
    // </editor-fold>
    SLPCRCON    = 0x29;                 // Slope Compensation Ramp Control Register. Not Bypassed. N=41 SLPS(mv/us) = 8.8 = 4.2785 * 1.0765^41
    // <editor-fold defaultstate="collapsed" desc="SLPCRCON Register Bit Description">
                                        // bit 7 Unimplemented, Read as 0
                                        // bit 6 SLPBY - Slope Compensation Bypass Control bit, 1=bypassed, 0=not bypassed
                                        // bit 5-0 SLPS<5:0> - Slope Compensation Rate Control bits 'N'. SLPS (mV/usec) = 4.2785 * 1.0765^N
    // </editor-fold>
    ICOACON     = 0x00;                 // Input Current Offset Adjust Control Register.  0mV Offset
    // <editor-fold defaultstate="collapsed" desc="ICOACON Register Bit Description">
                                        // bit 7-4 Unimplemented, Read as 0
                                        // bit 3-0 ICOAC<3:0> Input Current Offset Adjustment Configuration bits
                                        //          0000 = 0mV
                                        //          0001 = 50mV
                                        //          0010 = 100mV
                                        //          0011 = 150mV
                                        //          0100 = 200mV
                                        //          0101 = 250mV
                                        //          0110 = 300mV
                                        //          0111 = 350mV
                                        //          1000 = 400mV
                                        //          1001 = 450mV
                                        //          1010 = 500mV
                                        //          1011 = 550mV
                                        //          1100 = 600mV
                                        //          1101 = 650mV
                                        //          1110 = 700mV
                                        //          1111 = 750mV
    // </editor-fold>
    ICLEBCON    = 0x03;                 // Input Current Leading Edge Blanking Control Register.  200nsec blanking
    // <editor-fold defaultstate="collapsed" desc="ICLEBCON Register Bit Description">
                                        // bit 7-2 Unimplemented, read as 0
                                        // bit 1-0 ICLEBC<1:0> Input current Leading Edge Blanking Configuration bits.
                                        //          00 = 0nsec
                                        //          01 = 50nsec
                                        //          10 = 100nsec
                                        //          11 = 200nsec
    // </editor-fold>
    DESATCON    = 0x00;                 // Desaturation Comparator Control Register. Everything disabled.
    // <editor-fold defaultstate="collapsed" desc="DESATCON Register Bit Description">
                                        // bit 7 CDSMUX - DESAT Comparator Module Multiplexer Channel Selection bit, 
                                        //      1=BandGap Selected - 1.23V (BG), 0=DESAT_p Selected
                                        // bit 6 CDSWDE - DESAT Comparator Watch Dog Enable, 1=WD signal enables PWM Reset, 0=WD does not allow PWM reset
                                        // bit 5 reserved
                                        // bit 4 CDSPOL - DESAT Comparator Polarity Select bit, 1=output is inverted, 0=output is not inverted
                                        // bit 3 CDSOE - DESAT Comparator output enable bit, 1=DESAT Comparator output PWM is enabled, 0=disabled
                                        // bit 2 CDSOUT - DESAT Comparator Output Status bit
                                        //      if CDSPOL = 1 (inverted polarity). 1 = CDSVP < CDSVN (DESAT detected), 0 = CDSVP > CDSVN (Not Detected)
                                        //      if CDSPOL = 0 (not inverted). 1 = CDSVP > CDSVN (DESAT not detected), 0 = CDSVP < CDSVIN (Detected)
                                        // bit 1 CDSINTP - CDSIF Comparator Interrupt on Positive Going Edge Enable
                                        //      1 = The CDSIF interrupt flag will be set upon a positive going edge
                                        //      0 = No CDSIF interrupt flag will be set upon a positive going edge
                                        // bit 0 CDSINTN - CDSIF Comparator Interrupt on Negative Going Edge Enable bit
                                        //      1 = The CDSIF interrupt flag will be set upon a negative going edge
                                        //      0 = No CDSIF interrupt flag will be set upon a negative going edge
    // </editor-fold>
    VREFCON     = 0x00;                 // Current/Voltage Regulation Set Point Control Register.  Set to 0 for start.
    // <editor-fold defaultstate="collapsed" desc="VREFCON Register Bit Description">
                                        // bit 7-0 VREF<7:0> Voltage-Controlling Current Regulation Set Point bits
                                        // V_REF(V) = V_BG * N/255 where N = the decimal value between 0-255.
    // </editor-fold>
    VREF2CON    = 0x00;                 // V_REF2 Voltage Reference. Used to be sent off chip.
    // <editor-fold defaultstate="collapsed" desc="VREF2CON Register Bit Description">
                                        // bit 7-0 VREF2<7:0> Voltage Controlling Current Regualtion Set Point bits
                                        // VREF2(V) = V_BG * N/255
                                        // Used to control a second ref DAC to be sent off chip.  The MSC<0:1> = 01 in the MODECON register to 
                                        // connect V_REF2 to GPB1.  In Stand-Alone mode, V_REF2 is not accessible.
    // </editor-fold>
    OVCON       = 0x00;                 // Output overvoltage comparator control register.  Comparator disabled.
    // <editor-fold defaultstate="collapsed" desc="OVCON Register Bit Description">
                                        // bit 7-4 Unimplemented, Read as 0
                                        // bit 3 OVEN - OV Comparator output enable bit, 1=enabled, 0=disabled
                                        // bit 2 OVOUT - Output Overvotlage Status Output bit, 1=OV has occurred, 0=has not occurred
                                        // bit 1 OVINTP - OV Comparator Interrupt on Positive Going Edge Enable bit
                                        //              1 = The OVIF interrupt flag will be set upon a positive going edge of the OV
                                        //              0 = No OVIF interrupt flag will be set upon a positive going edge of the OV
                                        // bit 0 OVINTN - OV Comparator Interrupt on Negative Going Edge Enable bit
                                        //              1 = The OVIG interrupt flag will be set upon a negative going edge of the OV
                                        //              0 = No OVIG interrupt flag will be set upon a negative going edge of the OV
                                        // NOTE: The OVIF interrupt flag bit is set when an interrupt condition occurs, regardless of the 
                                        // state of its corresponding enable bit or the GIE in the INTCON register.  Also, if enabled
                                        // during and Output over voltage event the SDRV automatically goes HIGH!!!! PDRV goes Low.
                                        // Exercise caution using this comparator instead of setting up a second loop with EA2.
    // </editor-fold>
    OVREFCON    = cs.OVREFCON_CV;       // Voltage Regulation and Output Overvoltage Detect Level Register. With EA1 enabled, this is the CV regulation value
    // <editor-fold defaultstate="collapsed" desc="OVREFCON Register Bit Description">
                                        // bit 7-0 OOV<7:0> Output Overvoltage Detect Level Configuration bits
                                        // V_OV-REF(V) = 2 * V_BG * N/255, N is between 0-255.
    // </editor-fold>
    // OVREFCON = cs.OVREFCON_OV; %%BRYAN%% When EA1 is disabled and OOC comparator is enabled this sets the output over voltage threshold.
    // OVREFCON = cs.OVREFCON_CV; %%BRYAN%% this is a change in the DAC for when the converter is in CV mode.
    /* Set these below for turning on the divider and setting for 3 or 4 cells stacks. There's no GUI support now for it so it's set to 3 cell.*/
    Switch1 = 1;
    Switch2 = 1;
    Switch3 = 0;

}

void inc_iout(short increment)      // Topology Specific function - Increase Current (Combined)
{
	int i;
    
#ifdef SOFT_START_ENABLED           // What's going on here is that the voltage control loop is being used to clamp the 
    if (cd.adc_iout < (cs.ibat_pc >> 2))    //current control loop to avoid the huge inrush otherwise present from the
    {                                       //error amplifier being railed at start-up. The voltage is incremented until
        vl++;                               //it catches the battery voltage and a current becomes present indicating the
        OVREFCON = vl;                      //current error amplifier takes over.  At which point the OVREFCON is set to
    }                                       //it's maximum value and the reference is controlled for ramping the current.
    else                                    //The threshold for determining current flow is set to about 25% of pre-condition
    {                                       //current arbitrarily.  For Super-caps it's 2.5% of charge current.
        OVREFCON = cs.OVREFCON_CV;
        for(i=0;i<increment;i++)			// Loop through the increment routine (if fast_ramp requires)
        {
          if (vh >= cs.VREFCON_MAX) 		// If we are over the top of the register
          {
              vh = cs.VREFCON_MAX;
          }
          else 
          {
                  vh++;
          }
          VREFCON = vh;                     //Load the DAC with the new values
        }
    }
#endif
    
#ifndef SOFT_START_ENABLED
    for(i=0;i<increment;i++)			// Loop through the increment routine (if fast_ramp requires)
    {
        if (vh >= cs.VREFCON_MAX) 		// If we are over the top of the register
        {
            vh = cs.VREFCON_MAX;
        }
        else 
        {
            vh++;
        }
        VREFCON = vh;                     //Load the DAC with the new values
    }
#endif
}

short dec_iout(short decrement)	// Topology Specific function - Decrease Current (Combined)
{
	int i;

    if (vh == 0)     // Are we already at the bottom (off)?
        return(0);
	for (i = 0; i<decrement; i++)
	{
		if(vh > 0)					// Must check for zero because 'vh' can roll to FF if not.
		{
            vh--;					
        }
	}
    VREFCON = vh;         // Set DAC
	return(1);			// Success
}

void zero_iout(void)
{
	/* Output current set to zero */
	vh = 0;
	VREFCON = 0;
#ifdef SOFT_START_ENABLED //This is part of the new soft-start routing.  See void inc_iout(void) for more comments.
    OVREFCON = 0; 
    VREFCON = 0x05;       //%%BRYAN%% This needs to be a declared variable somewhere instead since there are going to be
    //vh = 0x15;            //%%BRYAN%% more versions of this board with different sense resistors.
    vh = 0x05;
    vl = 0;
#endif
}

void disable_charger(void)
{
	/* PWM off */
    PE1bits.SDRVEN = 0;
    PE1bits.PDRVEN = 0;

    T2CON = 0;
    
    /* Engage Battery Disconnect Switch if there is one... there isn't :( */

    /* Set output current to minimum */
    zero_iout();
    
    /* Disable Divider to prevent it discharing battery */
    //Switch1 = 0;
}

void hardware_custom_a(void)
{
    //Placeholder for any custom code.
    if (cd.adc_iout > 1142 && PE1bits.SDRVEN == 0 && PE1bits.PDRVEN ==1) //This is very close to the A2D reading for 500mA.  Which is where the PCB is guaranteed to be in CCM.
    {
        PE1bits.SDRVEN = 1;
    }
    else if (cd.adc_iout < 1142) //was 1542 for stock board, but I changed it to 1142 for the higher current board.
    {   
        PE1bits.SDRVEN = 0;
    }
}

void connect_battery(void)
{
    /* PWM off */
	PE1bits.SDRVEN = 0;
    PE1bits.PDRVEN = 0;

    T2CON = 0;
    
    /* Release Battery Disconnect Switch if there is one... there isn't :( */
    
    /* Enable Divider so the A2D can see the battery */
    // Switch1 = 1;
    // __delay_ms(1);
}

void disconnect_battery(void)
{
    /* PWM off: We don't want the charger running when we disconnect the battery or bad things will happen */
	PE1bits.SDRVEN = 0;
    PE1bits.PDRVEN = 0;
    
    T2CON = 0;
    
    /* Release Battery Disconnect Switch if there is one... there isn't :( */
    
    /* Release Divider so the A2D can see the battery */
    // Switch1 = 0;
    // __delay_ms(1);
}

void enable_charger(void) //%%BRYAN%% need to add the output overcurrent and overvoltage interrupt enables
{
    /* Set output current to minimum */
    zero_iout();

	/* UVLO Check */
    if (VINCONbits.UVLOOUT)
    {
		// log the reason 
		cd.status.vinuvlo = 1;
    }
    /* OVLO Check */
    else if (VINCONbits.OVLOOUT)
    {
        cd.status.voutovlo = 1;
    }
    /* Proceed */
    else
    {
        /* Release Battery Disconnect Switch if there is one... there isn't :( */
        
        /* Enable Divider so the A2D can see the battery */
        //Switch1 = 1;
        // __delay_ms(1);

        /* Limit Duty Cycle to 1 oscillator period for heavy load conditions*/
            PWMRL = 0x01;
        /* PWM on */
            T2CON = 0x04;
            VREFCON = 0;
            TMR2 = 0;
            PE1bits.PDRVEN = 1;
            //PE1bits.SDRVEN = 1;   //Starting into a pre-bias (i.e. battery) with a sync drive and no diode emulation is a bad idea.  
                                    //The converter will act bi-directionally with a very large duty cycle (from the soft-start)
                                    //and the result will be extreme current reversal charging the input capacitor.
        /* Restore Duty Cycle to maximum */ 
            PWMRL = 0x13;    // Limit duty cycle to 70% = (19 * 296kHz)/8MHz
    }
}

void check_button(void)
{
    extern unsigned short button_cnt;
    extern bit start;
    extern bit shutdown;
    
    button_cnt++;

    switch (button_state)
    {        
        case NOT_PRESSED:
            if (!BUTTON_PRESS_n)
            {
                button_state = DEBOUNCING;           
            }
            button_cnt = 0;
            break;
        case DEBOUNCING:
            if (BUTTON_PRESS_n)
            {
                button_state = NOT_PRESSED;
                button_cnt = 0;
            }
            else if (button_cnt > 100)
            {
                button_state = PRESSED;
                button_cnt = 0;
            }
            break;
        case PRESSED:
            if (BUTTON_PRESS_n)
            {
                button_state = NOT_PRESSED;
                button_cnt = 0;
            }
            else if (button_cnt > 1000)
            {
                button_state = COOL_DOWN;
                button_cnt = 0;
                if (cd.charger_state == CHARGER_OFF)
                {
                    start = 1;
                    cd.status.word = 0x0000; // Clear the shutdown cause
                }
                else 
                {
                    shutdown = 1;
                }
            }
            break;
        case COOL_DOWN:
            if (!BUTTON_PRESS_n)
            {
                button_cnt = 0;
            }
            else if (button_cnt > 1000)
            {
                button_state = NOT_PRESSED;
            }
            break;                 
    }        
}

#ifdef ENABLE_FIXED_CONFIG
#include "MultiChemCharger_Values.h"
#endif

void init_cal_registers(void) //%%BRYAN%% Verify and make use of the read_flash function instead of this repetition
{
    unsigned short addr;
    unsigned short data;
    
    /* Calibration Word 1 */
    addr = 0x20 <<8;                    // Location of the CGM<3:0> = CALWD1<7:4> = CGMCAL bits
    addr |= 0x80;                       // and the VGM<3:0> = CALWD1<3:0> = VGMCAL bits 
    data = read_flash(addr);
    CGMVGMCAL = ((unsigned short)data & 0xFF);   // Load CGMVGMCAL value from memory (Transconductance Amp Gains)
    
    /* Calibration Word 2 */
    addr = 0x20 <<8;                    // Location of the DST<4:0> = CALWD2<4:0> = DSTCAL bits
    addr |= 0x81;                       
    data = read_flash(addr);
    DSTCAL = ((unsigned short)data & 0x1F); // Input Offset Voltage for Desaturation Current Measurement Comparator
    
    /* Calibration Word 3 */
    addr = 0x20 <<8;                    // Location of the BGR<4:0> = CALWD3<4:0> = BGRCAL bits
    addr |= 0x82;                        
    data = read_flash(addr);
    BGRCAL = ((unsigned short)data & 0x1F); // Band Gap Reference Calibration

    
    /* Calibration Word 4 */
    addr = 0x20 <<8;                    // Location of the TTA<3:0> = CALWD4<3:0> = TTACAL bits
    addr |= 0x83;                       
    data = read_flash(addr);
    TTACAL = ((unsigned short)data & 0x0F); // Over Temperature Threshold Calibration
    
    /* Calibration Word 5 */
    // 2084h holds the ADC reading from the internal temperature sensor when the silicon temperature is at 28C. 
    // This 10 bit reading TANA<9:0> can be used to calculate the silicon die temp.  The temp coefficient is typically
    // 14.0mV/C +/- 0.8mV/C from -20C to 125C.  If the GUI want's it, then it "reads" FLASH at an opcode address of 0x40.
    
    /* Calibration Word 6 */
    addr = 0x20 <<8;                    // Location of the OSCCAL<7:0> = OSCCAL
    addr |= 0x85;                       
    data = read_flash(addr);
    OSCCAL = ((unsigned short)data & 0x7F); // Calibrates the internal oscillator
   
    /* Calibration Word 7 */
    addr = 0x20 <<8;                    // Location of the DCS<6:0> = CALWD7<6:0> = DSCCAL
    addr |= 0x86;                       
    data = read_flash(addr);
    DCSCAL = ((unsigned short)data & 0x7F); // Calibrates the offset for the diff amp (A2) when using I_SOUT  

    
    /* Calibration Word 8 */
    addr = 0x20 <<8;                    // Location of the VEAOFFCAL<6:0> = CALWD8<7:0> = VEAOFFCAL bits
    addr |= 0x87;                      
    data = read_flash(addr);
    VEAOFFCAL = ((unsigned short)data & 0x7F); // Calibrates the offset for the voltage regulation error amp
    
    /* Calibration Word 9 */
    addr = 0x20 <<8;                    // Location of the OVRSPCAL<4:0> = CALWD9<12:8> = OVRSPCAL bits
    addr |= 0x88;                       // and A2GCAL<3:0> = CALWD9<3:0> = A2GCAL
    data = read_flash(addr);
    OVRSPCAL = (((unsigned short)data>>8) & 0x1F);    // Calibration values for the OV_REF DAC span trim
    A2GCAL = ((unsigned short)data & 0x0F); // Calibration values for the Current Sense Amplifier Gain
    
    /* Calibration Word 10 */
    addr = 0x20 <<8;                    // Location of the VR2SPCAL<9:0> = CALWD10<12:8> = VRSSPCAL
    addr |= 0x89;                       // and VRSPCAL<4:0> = CALWD10<4:0> = VR2SPCAL
    data = read_flash(addr);
    VR2SPCAL = (((unsigned short)data>>8) & 0x1F);     // Calibration bits for V_REF2 DAC span trim
    VRSPCAL = ((unsigned short)data & 0x1F); // Calibration bits for V_REF2 DAC span trim

    /* Calibration Word 11 */
    addr = 0x20 <<8;                    // Location of the AVDDCAL<3:0> = CALWD10<11:9> = AVDDCAL
    addr |= 0x8A;                       // and BUFF<7:0> = CALWD10<7:0> = Test Mux Buffer offset
    data = read_flash(addr);
    AVDDCAL = (((unsigned short)data>>8) & 0x0F); // Calibraion bits for the 4V LDO Avdd
    // BUFF is also contained in this CALWD and is an 8 bit, 2's complement word that represents the buffer's offset voltage in units
    // of mV.  This value can be used to correct for buffer offset of the analog test signal measurements.
    
    /* Calibration Word 12 */
    addr = 0x20 <<8;                    // Location of the CEAOFFCAL<6:0> = CALWD10<6:0> = CEAOFFCAL
    addr |= 0x8B;                       
    data = read_flash(addr);
    CEAOFFCAL = ((unsigned short)data & 0x7F); // Calibration for the offset voltage of the current regulation error amp
    
    //%%BRYAN%% Experiment with a structure for calibration words (e.g. relocated the cs.tanai and cs.tanam elements) that can be used
    // with ifndef / endif conditions to enable the firmware to correct ADC measurements for maximum accuracy.
}

/* This ENABLE_GUI_CONFIG ifdef is meant to save a little code and prevent flash writes when this device function */
/* was locked down. While it adds the ability to include MultiChemCharger_Values.h it locks out the ability for   */
/* the GUI to change configuration settings. Be aware of this.  If you are testing compiling with the header file */
/* and still want to be able to use the GUI then be sure to set the ENABLE_GUI_CONFIG and ENABLE_FIXED_CONFIG in  */
/* the header file.   */

#ifdef ENABLE_GUI_CONFIG   

/* This function writes 8 bytes (4 flash words) at a time to flash. */
/* The incoming address must be 8-byte aligned.                     */

unsigned short write_flash(unsigned short addr, unsigned short counter) 
{
    unsigned char b = 0;
	if (addr < CAL_BASE_ADDR)           // Invalid address.  Should return a failure result
	{
        return 0x00; 
    }
    if (counter == 8)				// Write data if 8 bytes (4-words) have been received
    {
        while (b < 8) 
        {
            PMADRH = addr >> 8;
            PMADRL = addr;
            PMDATL = flash_write_buf[(unsigned char)(b & 0x07)];
            b++;
            PMDATH = flash_write_buf[(unsigned char)(b & 0x07)];
            b++;
            PMCON1bits.CALSEL = 0;
            PMCON1bits.WREN = 1;
            PMCON2 = 0x55;
            PMCON2 = 0xAA;
            PMCON1bits.WR = 1;
            NOP();
            NOP();
            PMCON1bits.WREN = 0;
            addr++;
        }
        return 0x04;
    }
    return 0x00;
}
#endif

unsigned short read_flash(unsigned short addr)
{
    unsigned short a;

    PMADRH = addr >> 8;
    PMADRL = addr;
    PMCON1bits.CALSEL = 0;
    PMCON1bits.RD = 1;
    asm("nop");
    asm("nop");
    a = (unsigned short)(PMDATH << 8);
    a |= PMDATL;
    return a;
}

unsigned char check_for_uvlo(void)
{
    if ((cd.adc_vin < cs.uvlo_adc_threshold_off) || (VINCONbits.UVLOOUT)) //UVLO and OVLO edges are being handled by ISR
	{
		return 1;
	}
    return 0;
}

#ifdef ENABLE_STATUS_LEDS
/************************************************************************************/
/* This board has one LEDs.                                                         */
/* LED1 is a status light                                                           */
/************************************************************************************/
void update_status_leds(void)
{
    // LED #1 - Fault LED
    if (LED_TMR-- == 0) 
    {
        if (bLED_On) 
        {
            bLED_On = 0;
            LED1_PIN = 1;
            LED_TMR = LED_OffTime;
        } 
        else 
        {
            bLED_On = 1;
            LED1_PIN = 0;
            LED_TMR = LED_OnTime;
        }
    }

}
#endif // end Enable Status LEDS

 #endif	// end of MCP19125 Configuration
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="PIC16F1713 MCP1631 SEPIC">
#ifdef PIC16F1716_MCP1631_SEPIC
/* The following hardware routines are specific to the reference board #102-00232 also known
 * as the MCP1631-MCC2 reference design.  This particular firmware source requries the PIC be
 * replaced with a PIC16F1716. This is a newer processor at a lower cost-point.  It includes
 * additional resources internally including multiple PWM channels which allow for a simple
 * duty-cycle adjustment to create the Vref for the MCP1631 (vs. bit-banging the output).
 */

#ifdef ENABLE_FIXED_CONFIG
#include "MultiChemCharger_Values.h"
#endif

// Configuration bits //

// CONFIG1
#pragma config IESO = ON		// Internal/External Switchover Mode->Internal/External Switchover Mode is enabled
#pragma config BOREN = ON		// Brown-out Reset Enable->Brown-out Reset enabled
#pragma config PWRTE = OFF		// Power-up Timer Enable->PWRT disabled
#pragma config FOSC = INTOSC    // Oscillator Selection Bits->INTOSC oscillator: I/O function on CLKIN pin
#pragma config FCMEN = ON		// Fail-Safe Clock Monitor Enable->Fail-Safe Clock Monitor is enabled
#pragma config MCLRE = ON		// MCLR Pin Function Select->MCLR/VPP pin function is MCLR
#pragma config CP = OFF			// Flash Program Memory Code Protection->Program memory code protection is disabled
#pragma config WDTE = OFF		// Watchdog Timer Enable->WDT disabled
#pragma config CLKOUTEN = OFF   // Clock Out Enable->CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin

// CONFIG2
#pragma config WRT = OFF		// Flash Memory Self-Write Protection->Write protection off
#pragma config ZCDDIS = ON		// Zero-cross detect disable->Zero-cross detect circuit is disabled at POR and can be enabled with ZCDSEN bit.
#pragma config LPBOR = OFF		// Low-Power Brown Out Reset->Low-Power BOR is disabled
#pragma config PPS1WAY = ON		// Peripheral Pin Select one-way control->The PPSLOCK bit cannot be cleared once it is set by software
#pragma config LVP = OFF		// Low-Voltage Programming Enable->High-voltage on MCLR/VPP must be used for programming
#pragma config STVREN = ON		// Stack Overflow/Underflow Reset Enable->Stack Overflow or Underflow will cause a Reset
#pragma config PLLEN = ON		// Phase Lock Loop enable->4x PLL is always enabled
#pragma config BORV = LO		// Brown-out Reset Voltage Selection->Brown-out Reset Voltage (Vbor), low trip point selected.


// Global Variables for this hardware implementation //
unsigned short Vref_PWM_DC;	// Current value of the output PWM

// External variables required by these routines
extern struct charger_settings_t cs;	// Get access to Charger_Settings structure
extern struct charger_data_t cd;		// Get access to Charger_Data structure

// Defines specific to this hardware //
#define PWM_MAX_DC 0x37A	// Maximum allowed duty cycle of the Vref generation PWM
#define PWM_MIN_DC 0x02		// Minimum allowed duty cycle of the Vref generation PWM


// Initialize the Hardware //
void init_hardware()	// Hardware specific configuration
{
// Configure Oscillator //
    OSCCON = 0x70;	// SPLLEN disabled; SCS FOSC; IRCF 8MHz_HF;
    OSCTUNE = 0x00;	// TUN 0x0;

// Configure I/O Registers //
	LATA =	 0b00010000;	// Start with MCP1631 on for calibration
	TRISA =  0b11101111;    /* 0=output, 1=input (tri-state) */
    ANSELA = 0b00001111;	// 0B = select bits 0, 1 & 3 as analog bits,  0=Digital, 1=Analog
    WPUA = 0x00;

    LATB = 0x00;
	TRISB = 0xC0;    /* 0=output, 1=input (tri-state) */
    ANSELB = 0x00;
    WPUB = 0x00;

    LATC =  0b00000100;	// Set PWM output (PWM) high for MCP1631 safety
	TRISC = 0b00011000;    /* RC3, RC4 are inputs.  0=output, 1=input (tri-state) */
    ANSELC = 0x00;
    WPUC = 0x00;

    TRISE = 0x08;	// MCLR pin set to output
    WPUE = 0x00;

    GIE = 0;						// Disable interrupts
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x00;	// Unlock PPS

    RC1PPSbits.RC1PPS = 0x0E;		// RC1->PWM3:PWM3OUT	Vref generation PWM
    RC2PPSbits.RC2PPS = 0x0F;		// RC2->PWM4:PWM4OUT	500Khz Fsw generation PWM
	SSPCLKPPSbits.SSPCLKPPS = 0x13;	// RC3->MSSP:SCL		I2C Clock Pin Selection
    RC3PPSbits.RC3PPS = 0x10;		// RC3->MSSP:SCL
    SSPDATPPSbits.SSPDATPPS = 0x14;	// RC4->MSSP:SDA		I2C Data Pin Selection
    RC4PPSbits.RC4PPS = 0x11;		// RC4->MSSP:SDA
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x01;	// Lock PPS



// Configure output MOSFET drivers (safe powerup, so do this first thing)
//	  This reference design doesn't have an output FET for protection due to the SEPIC topology.

// Configure Timer-0 for 512uS/Tick
	OPTION_REG = 0x02;	/* TMR0 1:8 prescale, 256 us per timer overflow */
    OPTION_REGbits.nWPUEN = 0x01;	// Weak pullups Disabled

// Configure Timer-2 for Vref Generation on RC1
    T2CON = 0x01;		// TMR2ON off; T2CKPS 1:4; T2OUTPS 1:1;
    PR2 = 0xFF;			// PR2 FF;	// Set the period for 7.8125Khz (8-bits of resolution)
    TMR2 = 0x00;		// Clear the timer
    PIR1bits.TMR2IF = 0;	// Clearing IF flag.
    T2CONbits.TMR2ON = 1;	// Start the Timer by writing to TMRxON bit

// Configure PWM3 for Vreg Generation on RC1 (Uses Timer-2)
    PWM3CON = 0x80;		// PWM3EN enabled; PWM3POL active_hi;
    PWM3DCH = 0x0;		// PWM3DCH 25;
    PWM3DCL = 0x2;		// PWM3DCL 16;
    CCPTMRSbits.P3TSEL = 0x00;	// Select timer 2 as source


// Configure Timer-4 for MCP1631 switching frequency and max duty-cycle control on RC2
    T4CON = 0x00;	// T4CKPS 1:1; T4OUTPS 1:1; TMR4ON off;
    PR4 = 0x0F;		// Set the period for 500Khz
    TMR4 = 0x00;	// Clear the timer
    PIR2bits.TMR4IF = 0;	// Clearing IF flag.
    T4CONbits.TMR4ON = 1;	// Start TMR4

// Configure PWM4 for Switching Frequency Generation on RC2 (Uses Timer-4)
	PWM4CON = 0x00;	// PWM4POL active_low; PWM4EN disabled;
    PWM4DCH = 0x05;	// PWM4DCH
    PWM4DCL = 0x30;	// PWM4DCL
    CCPTMRSbits.P4TSEL = 0x01;	// Select timer 4 as the source

// Configure Interrupts //
	PIE1 = 0;	// No hardware or I/O interrupts
    PIE2 = 0;
	INTCON = 0xc0; /* Global + Peripheral interrupts */

// Configure ADC //
    ADCON0 = ADC_MUX_VIN | 0x01; // FVR Reference, Default input channel, module enabled
    ADCON1 = 0b11100000;		 // Right Justified data, ADC 4 us/conv, VDD and VSS connected
	ADCON2 = 0x00;				 // No autoconversion
	
// Enable input undervoltage lockout threshold //
	// Not on this topology.  We use the ADC to do this.

} // End of init_hardware()



void inc_iout( short increment)  	// Topology Specific function - Increase Current (Combined)
{
	int i;

  for(i=0;i<increment;i++)				// Loop through the increment routine
	{
		if (Vref_PWM_DC <= PWM_MAX_DC)	// If DC is not maxed out
			Vref_PWM_DC++;				// Increment PWM Duty Cycle register
		else
			Vref_PWM_DC = 2;			// Reset to minimum if we are out of range (shouldn't happen)
	}

    // Writing to 8 MSBs of PWM duty cycle in PWMDCH register
    PWM3DCH = ((Vref_PWM_DC & 0x03FC)>>2);

    // Writing to 2 LSBs of PWM duty cycle in PWMDCL register
    PWM3DCL  |= ((Vref_PWM_DC & 0x0003)<<6);
}


short dec_iout( short decrement)	// Topology Specific function - Decrease Current (Combined)
{
	int i;

	if (Vref_PWM_DC == PWM_MIN_DC)	// Are we already at the bottom (off)?
		return(0);					// Return a fault code that we can't decrement any more

	for(i=0;0<decrement;i++)
	{
		if(Vref_PWM_DC > PWM_MIN_DC)  // Must check for zero because 'Vref_PWM_DC' can roll to FF if not.
			Vref_PWM_DC--;			  // Reduce value if we are not already at the bottom of the range
		else
		{
			Vref_PWM_DC = PWM_MIN_DC; // Else, we are at the bottom (vl = vf = 0)
			return(0);				  // Return a fault code that we can't go lower
		}
	}
    // Writing to 8 MSBs of PWM duty cycle in PWMDCH register
    PWM3DCH = ((Vref_PWM_DC & 0x03FC)>>2);

    // Writing to 2 LSBs of PWM duty cycle in PWMDCL register
    PWM3DCL = ((Vref_PWM_DC)<<6);
	return(1);
}


void zero_iout(void)
{
	Vref_PWM_DC = PWM_MIN_DC;	/* Output current set to zero */

	// Writing to 8 MSBs of PWM duty cycle in PWMDCH register
    PWM3DCH = ((Vref_PWM_DC & 0x03FC)>>2);

    // Writing to 2 LSBs of PWM duty cycle in PWMDCL register
    PWM3DCL = ((Vref_PWM_DC)<<6);
}


void disable_charger(void)
{
	MCP1631_ENABLE = 0;	// Clear MCP1631 enable pin
	NOP();
	NOP();
    PWM4CON = 0x00;	// Turn off PWM output
	zero_iout();	// Drop reference to minimum
}


void enable_charger(void)
{
	zero_iout();	// Drop reference to minimum
    PWM4CON = 0x80;	// Turn on PWM output
	MCP1631_ENABLE = 1;	// Set MCP1631 enable pin
}


/***************************** PIC16F1713/6 FLASH ROUTINES ************************/

void init_cal_registers(void)	// There are no internal parameters in the PIC16F171x
{								// that require calibration at boot up.
}


/* This function writes 32 bytes (16 flash words) at a time to flash.
 * The incoming address must be aligned to a row of memory.  Writing to a single
 * memory location within a row is not supported.
 */

/* This ifdef is meant to save a little code and prevent flash writes when this device function was locked down.  */
/* While it adds the ability to include MultiChemCharger_Values.h it locks out the ability for the GUI to change  */
/* configuration settings. Be aware of this.  If you are testing compiling with the header file and still want to */ 
/* be able to use the GUI then be sure to set the ENABLE_GUI_CONFIG and ENABLE_FIXED_CONFIG in the header file.   */
#ifdef ENABLE_GUI_CONFIG   
unsigned short write_flash(unsigned short addr, unsigned short counter)
{
    unsigned char b = 0;
	if (addr < CAL_BASE_ADDR)           // Invalid address.  Should return a failure result
	{
        return 0x00; 
    }
    if (counter == 64)				// Write data if 64 bytes (32-words) have been received
    {
        if (addr == 0xe80)
            NOP();
        if (addr == 0xea0)
            NOP();
        if (addr == 0xeb0)
            NOP();
        if (addr == 0xec0)
            NOP();

        INTCONbits.GIE = 0;		// Disable Interrupts

        /* Start by erasing 32 words from memory, one row. (Figure 10-4) */
        if (addr == 0xe80 | addr == 0xea0 | addr == 0xec0)	// Valid starting addresses of complete rows of memory
        {
            PMADRH = addr >> 8;
            PMADRL = addr;
            PMCON1bits.CFGS = 0;	// Not configuration space, we are erasing program memory
            PMCON1bits.FREE = 1;	// Specify an erase operation
            PMCON1bits.WREN = 1;	// Enable write

            PMCON2 = 0x55;			// Unlock Sequence
            PMCON2 = 0xAA;
            PMCON1bits.WR = 1;
            NOP();
            NOP();

            PMCON1bits.WREN = 0;
        }

        // Now program a row of 16/32 words
        // Per Figure 10-6 of the datasheet //
        PMCON1bits.CFGS = 0;	// Not configuration space, we are programming the program memory registers
        PMADRH = addr >> 8;		// Load the address to program
        PMADRL = addr;
        PMCON1bits.FREE = 0;	// Specify a write operation
        PMCON1bits.LWLO = 1;	// Load write latches only
        PMCON1bits.WREN = 1;	// Enable write

        while (b < counter)				// Load the program memory latches with data one word at a time
        {
            PMDATL = flash_write_buf[b & 0x07];
            b++;
            PMDATH = flash_write_buf[b & 0x07];
            b++;

            if (addr == 0xee0)
                NOP();

            if (b < counter)
            {
                PMCON2 = 0x55;		// Unlock Sequence
                PMCON2 = 0xAA;
                PMCON1bits.WR = 1;
                NOP();
                NOP();

                addr++;				// Increment the address
                PMADRH = addr >> 8;	// Load the address to program
                PMADRL = addr;
            }
        }

        PMCON1bits.LWLO = 0;	// Load write latches
        PMCON2 = 0x55;			// Unlock Sequence
        PMCON2 = 0xAA;
        PMCON1bits.WR = 1;
        NOP();
        NOP();
        PMCON1bits.WREN = 0;
        INTCONbits.GIE = 1;		// Re-enable Interrupts
            if (addr > 0xee0)
                NOP();
        return 0x20; 
    }
    return 0x00;
}
#endif

unsigned short read_flash(unsigned short addr)
{
    unsigned short a;

    PMADRH = addr >> 8;
    PMADRL = addr;
    PMCON1bits.CFGS = 0;
    PMCON1bits.RD = 1;
    asm("nop");
    asm("nop");
    a = PMDATH << 8;
	a |= PMDATL;
    return a;
}

unsigned char check_for_uvlo(void)
{
    if (cd.adc_vin < cs.uvlo_adc_threshold_off)
	{
		return 1;
	}
    return 0;
}

void check_button(void)
{
}

void hardware_custom_a(void)
{
}

void connect_battery(void)
{
    /* Connect Battery Switch */
     
}

#ifdef ENABLE_STATUS_LEDS
void update_status_leds(void)
{
	// LED #1 - Charger Active
	if(cd.charger_state != CHARGER_OFF)		// If the charger is active,
		LED1_PIN = 1;						// Charger Active LED #1 on
	else
		LED1_PIN = 0;						// Charger Active LED #1 off

	// LED #2 - Charger State (CC/CV)
	if(cd.charger_state == CHARGER_LIION_CC ||	// If the charge state = CC
		cd.charger_state == CHARGER_NIMH_RAPID ||
		cd.charger_state == CHARGER_VRLA_CC ||
		cd.charger_state == CHARGER_VRLAF_RAPID ||
		cd.charger_state == CHARGER_LIFEPO4_CC)
		LED2_PIN = 1;						// Charger CC/CV State LED #2 on
	else
		LED2_PIN = 0;						// Charger CC/CV State LED #2 off

	// LED #3 - Charge Complete
	if(cd.status.complete == 1)
		LED3_PIN = 1;						// Charge Complete LED #3 on
	else
		LED3_PIN = 0;						// Charge Complete LED #3 off

	// LED #4 - Fault LED
        if((cd.status.word & 0x7FFF) != 0)       // These shutdown causes are not defined yet
		LED4_PIN = 0;						// Fault LED #4 off

	else									// All other shutdown_causes are true hard-faults
		LED4_PIN = 1;						// Fault LED #4 on

}
#endif // end Enable Status LEDS

#endif		// PIC16F1713_MCP1631_SEPIC
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="PIC16F883 Topology">
#ifdef PIC16F883_MCP1631_SEPIC
/* The following hardware routines are specific to the reference board #102-00232 also known
 * as the MCP1631-MCC2 reference design.  This particular firmware source uses the standard PIC CPU on the reference.
 * Note that the Vref generation is performed using a manual bit-banging output via a Timer-1 interrupt.
 */

#ifdef ENABLE_FIXED_CONFIG
#include "MultiChemCharger_Values.h"
#endif

// Configuration bits //

// CONFIG1
#pragma config IESO = ON	// Internal/External Switchover Mode->Internal/External Switchover Mode is enabled
#pragma config BOREN = ON	// Brown-out Reset Enable->Brown-out Reset enabled
#pragma config PWRTE = OFF	// Power-up Timer Enable->PWRT disabled
#pragma config FOSC = INTRC_NOCLKOUT	// Oscillator Selection Bits->INTOSC oscillator: No Clock out (I/O function on CLKIN pin)
#pragma config FCMEN = ON	// Fail-Safe Clock Monitor Enable->Fail-Safe Clock Monitor is enabled
#pragma config MCLRE = ON	// MCLR Pin Function Select->MCLR/VPP pin function is MCLR
#pragma config CP = OFF		// Flash Program Memory Code Protection->Program memory code protection is disabled
#pragma config WDTE = OFF	// Watchdog Timer Enable->WDT disabled
#pragma config LVP = OFF    // Low-Voltage Programming Enable->High-voltage on MCLR/VPP must be used for programming

// CONFIG2
#pragma config WRT = OFF    // Flash Memory Self-Write Protection->Write protection off
#pragma config BOR4V = BOR40V  // Low-Power Brown Out Reset-> 4v protection enabled


// Global Variables for this hardware implementation //
unsigned short Vref_PWM_DC;	// Current value of the output PWM


/* Software PWM for bit-banged reference generation (via interrupt) */
int IRef;
unsigned int IRefOn;
unsigned int IRefOnHiBits;
unsigned int IRefOffHiBits;
unsigned int IRefOff;
int IRefMax;
int IRefMin;


// External variables required by these routines
extern struct charger_settings_t cs;	// Get access to the Charger_Settings structure
extern struct charger_data_t cd;		// Get access to Charger_Data structure

void interrupt isr()
{
    if (PIR1bits.TMR1IF)			// Timer-1 Flagged?
	{
		PIR1bits.TMR1IF = 0;	// Clear flag
		if (!VREF_PWM_PIN)		// If pin is low, then make it high
		{
			VREF_PWM_PIN = 1;
			TMR1H = IRefOnHiBits;	// Change timer to on-time value (high byte)
			TMR1L = IRefOn;			// (low byte)
		}
		else
		{
			VREF_PWM_PIN = 0;
			TMR1H = IRefOffHiBits;	// Change timer to off-time value (high byte)
			TMR1L = IRefOff;		// (low byte)
		}
	}
	PIR1 = 0;	//!! CLear all INT flags
}

// Initialize the Hardware //
void init_hardware()	// Hardware specific configuration.  !!!! In process
{
// Configure Oscillator //
    OSCCON = 0x70;	// SPLLEN disabled; SCS FOSC; IRCF 16MHz_HF;
    OSCTUNE = 0x00;	// 

// Configure I/O Registers //

	ANSEL = 0x0B;    /* B = select bits 0, 1 & 3 as analog inputs,  0=Digital, 1=Analog */
	ANSELH = 0x00;   /* 0 = select bits 8-13 as digital bits, 0=Digital, 1=Analog */

	PORTA = 0x00;
	TRISA = 0xEF;    /* 0=output, 1=input (tri-state) */

    PORTB = 0x00;
	TRISB = 0xC0;    /* 0=output, 1=input (tri-state) */

    PORTC = 0x00;
	TRISC = 0x18;    /* RC3, RC4 are inputs.  0=output, 1=input (tri-state) */

    TRISE = 0x08;	// MCLR pin set to output

    GIE = 0;				// Disable interrupts

	while (!OSCCONbits.HTS);	// Wait for the oscillator to become stable : stable = 1, unstable = 0

// Configure output MOSFET drivers (safe powerup, so do this first thing)
//	  This reference design doesn't have an output FET for protection due to the SEPIC topology.

// Configure Timer-0 for 512uS/Tick
	OPTION_REG = 0x00;	/* TMR0 1:2 prescale, 256 us per timer overflow */

// Configure Timer-2 MCP1631 switching frequency and max duty-cycle control on RC2
    T2CON = 0x00;		 // TMR2ON off; T2OUTPS 1:1; T2CKPS 1:1
    PR2 = 0x03;			 // PR2 = 4 counts at 2Mhz;	// Set the period
    TMR2 = 0x00;		 // Clear the timer
    PIR1bits.TMR2IF = 0; // Clearing IF flag.

// Configure PWM 1 for Switching Frequency Generation on RC2 (Uses Timer-2 above)
	CCP1CON = 0x0C;  /* PWM Mode, single channel, Bits <5:4> are 2 ls bits of duty cycle, 2 us / 125 ns = 16, 25% of 16 = 4 = 00000001 00b */ \
	CCPR1L = 0x01;   /* 8 ms bits of 10 bit duty cycle, 2 us / 125 ns = 16, 25% of 16 =  4 = 00000001 00b */
    T2CONbits.TMR2ON = 1;	// Start the Timer by writing to TMRxON bit

// Configure Interrupts //
//	INTCONbits.T0IE = 0;	// Timer 0 Overflow Interrupt Enable (512uS/Tick timer) We sample this manually in main loop.
	PIE1bits.TMR1IE = 1;	// Timer 1 Interrupt enable (Vref bit-bang interrupt)
	INTCONbits.PEIE = 1;	// Enable unmasked peripheral interrupts
	INTCONbits.GIE = 1;		// Enable Global Interrupts


// Configure ADC //
    ADCON0 = ADC_MUX_VIN | 0x01; /* Default input channel */
    ADCON1 = 0x20; /* ADC 4 us/conv */

 // Enable input undervoltage lockout threshold //
	//!! We dont' have undervoltage configured via a comparator yet

} // End of init_hardware()

void inc_iout(short increment)  	// Topology Specific function - Increase Current (Combined)
{
	int i;
	int timer1_max_count = 0 - cs.ovcfcon_max;	// Set the maximum allowed value of the timer duty cycle

	for(i=0;i<increment;i++)			// Loop through the increment routine (if fast_ramp requires)
	{
		if (Vref_PWM_DC <= cs.ovcfcon_max) // If DC is not maxed out
			Vref_PWM_DC++;				   // Increment PWM Duty Cycle register
		else
			Vref_PWM_DC = 2;			   // Reset to minimum if we are out of range (shouldn't happen)
	}

	update_pwm();	// Update the hardware PWM
}

short dec_iout(short decrement)  	// Topology Specific function - Decrease Current (Combined)
{
    int i;
    if (Vref_PWM_DC == cs.ovcfcon_min) // Are we already at the bottom (off)?
    {
        return(0);
    }
	for (i = 0; i<decrement; i++)
	{
        if(Vref_PWM_DC > cs.ovcfcon_min)  // Must check for zero because 'Vref_PWM_DC' can roll to FF if not.
            Vref_PWM_DC--;				  // Reduce fine value if we are not already at the bottom of the range
        else
        {
            Vref_PWM_DC = cs.ovcfcon_min; // Else, we are at the bottom (vl = vf = 0)
            return(0);
        }
    }
	update_pwm();	// Update the hardware PWM
    return(1);
}

void zero_iout(void)
{
	Vref_PWM_DC = cs.ovcfcon_min;	/* Output current set to zero */
	update_pwm();	// Update the hardware PWM
}

void disable_charger(void)
{
	MCP1631_ENABLE = 0;	// Clear MCP1631 enable pin
	T1CON = 0b00010000;	// Turn off PWM output
	zero_iout();		// Drop reference to minimum
	update_pwm();		// Update the pwm output to minimum
}

void update_pwm(void)
{
	T1CON = 0b00010000;	/* Temporarily pause the PWM timer */

	IRefOn =  0 - Vref_PWM_DC;	// Timer starts at this pre-load value and counts up to overflow, generating an interrupt
	IRefOnHiBits = IRefOn >> 8;  /* used to reload timer faster in interrupt routine */

	IRefOff = Vref_PWM_DC - cs.ovcfcon_max;  /* IRef must always be less than ovcfcon_max */
	IRefOffHiBits = IRefOff >> 8; /* used to reload timer faster in interrupt routine */

	T1CON = 0b00010001;	/* Re-enable PWM timer, set prescale */
}

void enable_charger(void)
{
	zero_iout();		// Drop reference value to minimum
	T1CON = 0b00010000;	/* Turn off the PWM timer */
	MCP1631_ENABLE = 1;	// Set MCP1631 enable pin
}

/***************************** EEPROM ROUTINES ************************/

void init_cal_registers(void)	// No calibration registers on the PIC16F883
{
}

/* This ifdef is meant to save a little code and prevent flash writes when this device function was locked down.  */
/* While it adds the ability to include MultiChemCharger_Values.h it locks out the ability for the GUI to change  */
/* configuration settings. Be aware of this.  If you are testing compiling with the header file and still want to */ 
/* be able to use the GUI then be sure to set the ENABLE_GUI_CONFIG and ENABLE_FIXED_CONFIG in the header file.   */
#ifdef ENABLE_GUI_CONFIG 

/* This function writes 8 bytes (4 flash words) at a time to flash.
 * The incoming address must be 8-byte aligned.
 */

unsigned short write_flash(unsigned short addr, unsigned short counter) 
{
    unsigned char b = 0;
	if (addr < CAL_BASE_ADDR)           // Invalid address.  Should return a failure result
	{
        return 0x00; 
    }
    if (counter == 8)				// Write data if 8 bytes (4-words) have been received
    {
        while (b < 8) 
        {
            EECON1bits.EEPGD = 1;
            EEADRH = addr >> 8;
            EEADR = addr;
            EEDAT = flash_write_buf[(unsigned char)(b & 0x07)];
            b++;
            EEDATH = flash_write_buf[(unsigned char)(b & 0x07)];
            b++;
            EECON1bits.WREN = 1;
            EECON2 = 0x55;
            EECON2 = 0xAA;
            EECON1bits.WR = 1;
            NOP();
            NOP();
            EECON1bits.WREN = 0;
            addr++;
        }
        return 0x04;
    }
    return 0x00;
}
#endif

unsigned short read_flash(unsigned short addr)
{
     unsigned short a;

    EEADRH = addr >> 8;		// Load the address pointer high byte
    EEADR = addr;			// Load the address pointer low byte
	EECON1bits.EEPGD = 1;	// Point to program memory
	EECON1bits.RD = 1;		// Do a read
    asm("nop");
    asm("nop");
	a = (unsigned short)(EEDATH << 8);
	a |= EEDAT;
    return a;
}

unsigned char check_for_uvlo(void)
{
    if (cd.adc_vin < cs.uvlo_adc_threshold_off)
	{
		return 1;
	}
    return 0;
}

void check_button(void)
{
}

void hardware_custom_a(void)
{
}

void connect_battery(void)
{
    /* Connect Battery Switch */
     
}

#ifdef ENABLE_STATUS_LEDS
void update_status_leds(void)
{
	// LED #1 - Charger Active
	if (LED_TMR-- == 0) 
    {
    if(cd.charger_state != CHARGER_OFF)		// If the charger is active,
		LED1_PIN = 1;						// Charger Active LED #1 on
	else
		LED1_PIN = 0;						// Charger Active LED #1 off

	// LED #2 - Charger State (CC/CV)
	if(cd.charger_state == CHARGER_LIION_CC ||	// If the charge state = CC
		cd.charger_state == CHARGER_NIMH_RAPID ||
		cd.charger_state == CHARGER_VRLA_CC ||
		cd.charger_state == CHARGER_VRLAF_RAPID ||
		cd.charger_state == CHARGER_LIFEPO4_CC)
	{
        if (bLED_On)
        {
            LED2_PIN = 1;						// Charger CC/CV State LED #2 on
            bLED_On = 0;
            LED_TMR = LED_OffTime;
        }	
        else
        {
            bLED_On = 1;
            LED2_PIN = 0;
            LED_TMR = LED_OnTime;
        }
    }
    else
		LED2_PIN = 0;						// Charger CC/CV State LED #2 off

	// LED #3 - Charge Complete
	if(cd.status.complete == 1)
		LED3_PIN = 1;						// Charge Complete LED #3 on
	else
		LED3_PIN = 0;						// Charge Complete LED #3 off

	// LED #4 - Fault LED
        if((cd.status.word & 0x7FFF) != 0)                         // These shutdown causes are not defined yet
		LED4_PIN = 0;						// Fault LED #4 off

	else									// All other shutdown_causes are true hard-faults
		LED4_PIN = 1;						// Fault LED #4 on
    }
}
#endif // end Enable Status LEDS

#endif // PIC16F883 Topology
// </editor-fold>

