
#ifndef MULTICHEMBATTERYCHARGER_HARDWARE_H
#define	MULTICHEMBATTERYCHARGER_HARDWARE_H

#ifdef	__cplusplus
extern "C" {
#endif
   
// Function Prototypes //
    
void init_cal_registers(void);
unsigned short read_flash(unsigned short addr);
unsigned short write_flash(unsigned short addr, unsigned short counter);
void update_pwm(void);
unsigned char check_for_uvlo(void);
void init_hardware(void);			// Configure the hardware specific parameters, registers, and I/O
void inc_iout(short increment); // Hardware specific command to increase output current (gross increment)
short dec_iout(short decrement); // Decrease output current (gross increment)
void zero_iout(void);		// Set output current to zero (mimimum setting, might not be zero current)
void disable_charger(void);	// Disable output
void enable_charger(void);	// Enable output
void connect_battery(void); // This is an op-code initiated command to connect the battery to the circuit for calibration
void disconnect_battery(void); // This is an op-code initiated command to disconnect the battery from the circuit
void hardware_custom_a(void); // Generic function call for the main loop to custom hardware specific routines. Users choice.
void check_button(void);    // Routine for checking hardware specific manual button presses
#ifdef ENABLE_STATUS_LEDS
void update_status_leds(void);		// Function prototype for status LED update routine
#endif
#ifdef ENABLE_BENCH_TEST_OUTPUT_PIN
void update_buffcon(void);
#endif

enum button_state_t
{
    NOT_PRESSED = 0,
    DEBOUNCING = 1,
    PRESSED = 2,
    COOL_DOWN = 3,
} button_state;

/* Select Hardware Platform                                                             */
/* The configuration settings drop-down in MPLAB-X is used to set the reference design  */
/* platform that is utilized.  Please select the appropriate design there.  The result  */
/* will be that the following lines of code will be defined appropriately for you       */
/* You might also need to select a specific development board if there are more than    */
/* one board using the same CPU            */  

// <editor-fold defaultstate="collapsed" desc="MCP19111_CHARGER_TOPOLOGY">
#ifdef MCP19111_CHARGER_TOPOLOGY					// MCP19111 CHARGER DEMOBOARD

#define _XTAL_FREQ 8000000		// 8Mhz FOSC
#define T0COUNTSPERSEC (1953)	// 512uS per interupt
#define SERIAL_INT_FLAG SSPIF

/* Setup the device configuration bits of the CONFIG word at 0x2007*/
/* The #DBGEN (bit 13) is managed automatically by device development tools.  For normal operation it should be a 1*/
    
#pragma config MCLRE = ON       // #MCLR Pin Function Selet (bit 5) 1(ON)=MCLR# function and weak pull-up, 0(OFF)=alternate function
#pragma config CP = OFF         // Code Protection (bit 6) 1(OFF)=Disabled, 0(ON)=Enabled
#pragma config PWRTE = OFF      // Power-Up Timer Enable (bit 4) 1(OFF)=Disabled, 0(ON)=Enabled, Note:Not Controlled by User
#pragma config WDTE = OFF       // Watchdog Timer Enable (bit 3) 1(ON)=Enabled, 0(OFF)=Disabled
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable (bit 11:10) 11(OFF)=No Protection, 00(FULL)=Entire Memory, 10(BOOT)=0x000-0x3FF, 01(HALF)=0x000-0x7FF
//#pragma config BOREN = OFF      // Brown-out Reset Enable Bit (bit 8) 1(ON)=Enabled during operation and disabled in Sleep, 0 (OFF) Disabled


/* Hardware Specific Defines */
#define PCB_ID 1					// Reference design info passed to GUI 
#define BUTTON_RESET PORTAbits.GPA6	// Button #1 (Reset)
#define BUTTON_START PORTAbits.GPA4	// Button #2 (Start)
#define LED1_PIN PORTBbits.RB2		// LED #1 Pin	(RB2)
#define LED2_PIN PORTAbits.RA0		// LED #2 Pin	(RA0)
#define LED3_PIN PORTAbits.RA2		// LED #3 Pin	(RA2) TP6 These will just be TTL at the test point unless you deadbug a LED and resistor onto R32
#define LED4_PIN PORTAbits.RA1		// LED #4 Pin	(RA1) TP8 These will just be TTL at the test point unless you deadbug a LED and resistor onto R38


/* ADC-Channel Memory Offset Locations */
#define ADC_MUX_VIN  (0x00 << 2)    // VIN_ANA (analog voltage proportional to Vin)
#define ADC_MUX_VBAT (0x14 << 2)    // GPB1
#define ADC_MUX_IOUT (0x05 << 2)    // INT_VREG (internal version of the V_REG load voltage) //**BRYAN** This will be the output of the differential Vsense input? Explain
#define ADC_MUX_VREF (0x01 << 2)    // VREGREF (reference voltage for V_REG output)  //**BRYAN** Explain this one in comments to.
// Here below select where you want your temperature sampled from.  
//#define ADC_MUX_TEMP (0x0A << 2)    // TMP_ANA (analog voltage proportional to temperature)
#define ADC_MUX_TEMP (0x13 << 2)	// External battery temp sensing

/* The LSB (ovfcon) for output current where the course
 * control would need to be incremented. */
#define IOUT_LSB_ROLL (18)

/* Temp Cal*/
#define TEMP_CAL_ADDR (0x0000) // There is no TEMP_CAL register so just put in a bogus value so the complier doesn't complain.

#endif
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="MCP19118 Four Switch Buck-Boost Charger Demoboard">
#ifdef MCP19118_FOUR_SWITCH_BUCK_BOOST_CHARGER		// MCP19118_FOUR_SWITCH_BUCK_BOOST_CHARGER CHARGER DEMOBOARD

#define _XTAL_FREQ 8000000		// 8Mhz FOSC
#define T0COUNTSPERSEC (1953)	// 512uS per interupt
#define SERIAL_INT_FLAG SSPIF

/* Setup the device configuration bits of the CONFIG word at 0x2007*/
/* The #DBGEN (bit 13) is managed automatically by device development tools.  For normal operation it should be a 1*/
    
#pragma config MCLRE    = ON    // #MCLR Pin Function Selet (bit 5) 1(ON)=MCLR# function and weak pull-up, 0(OFF)=alternate function
#pragma config CP       = OFF   // Code Protection (bit 6) 1(OFF)=Disabled, 0(ON)=Enabled
#pragma config PWRTE    = OFF   // Power-Up Timer Enable (bit 4) 1(OFF)=Disabled, 0(ON)=Enabled, Note:Not Controlled by User
#pragma config WDTE     = OFF   // Watchdog Timer Enable (bit 3) 1(ON)=Enabled, 0(OFF)=Disabled
#pragma config WRT      = OFF   // Flash Program Memory Self Write Enable (bit 11:10) 11(OFF)=No Protection, 00(FULL)=Entire Memory, 10(BOOT)=0x000-0x3FF, 01(HALF)=0x000-0x7FF


/* Hardware Specific Defines */
#define PCB_ID          4					// Reference design info passed to GUI 
#define BUTTON_RESET    PORTBbits.GPB2      // Button #1 (Start/Stop)
#define BUTTON_START    PORTBbits.GPB2      // Button #1 (Start/Stop)
#define LED1_PIN        PORTAbits.RA0       // LED #1 Pin (GPA0)
#define TEST_PIN1       PORTAbits.RA2       // Test pin (GPA2)
#define BUTTON_PRESS_n  PORTBbits.GPB2      // Button #1 (Start/Stop)  Active Low
#define BUTTON_START    PORTBbits.GPB2      // Button #1 (Start/Stop)  Active Low

/* ADC-Channel Memory Offset Locations */ //%%BRYAN%% Correct for the new board
#define ADC_MUX_VIN  (0x00 << 2)    // VIN_ANA (analog voltage proportional to Vin)
#define ADC_MUX_VBAT (0x14 << 2)    // INT_VREG (internal version of the V_REG load voltage) Basically +/- VSEN
#define ADC_MUX_IOUT (0x13 << 2)    // GPA3 (Analog Input)
#define ADC_MUX_VREF (0x01 << 2)    // VREGREF (reference voltage for V_REG output)  //**BRYAN** Explain this one in comments to.
#define ADC_MUX_TEMP (0x0A << 2)    // TMP_ANA (analog voltage proportional to temperature)

/* The LSB (ovfcon) for output current where the course
 * control would need to be incremented. */
#define IOUT_LSB_ROLL (18)

/* Temp Cal*/
#define TEMP_CAL_ADDR (0x0000) // There is no TEMP_CAL register so just put in a bogus value so the complier doesn't complain.

#endif
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="MCP19119_4A_Power_Tool_Charger">
#ifdef MCP19119_4A_Power_Tool_Charger					// MCP19111 CHARGER DEMOBOARD

#define _XTAL_FREQ 8000000		// 8Mhz FOSC
#define T0COUNTSPERSEC (1953)	// 512uS per interupt
#define SERIAL_INT_FLAG SSPIF

/* Setup the device configuration bits of the CONFIG word at 0x2007*/
/* The #DBGEN (bit 13) is managed automatically by device development tools.  For normal operation it should be a 1*/
    
#pragma config MCLRE = ON       // #MCLR Pin Function Selet (bit 5) 1(ON)=MCLR# function and weak pull-up, 0(OFF)=alternate function
#pragma config CP = OFF         // Code Protection (bit 6) 1(OFF)=Disabled, 0(ON)=Enabled
#pragma config PWRTE = OFF      // Power-Up Timer Enable (bit 4) 1(OFF)=Disabled, 0(ON)=Enabled, Note:Not Controlled by User
#pragma config WDTE = OFF       // Watchdog Timer Enable (bit 3) 1(ON)=Enabled, 0(OFF)=Disabled
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable (bit 11:10) 11(OFF)=No Protection, 00(FULL)=Entire Memory, 10(BOOT)=0x000-0x3FF, 01(HALF)=0x000-0x7FF


/* Hardware Specific Defines */
#define PCB_ID 7					// Reference design info passed to GUI 
#define BUTTON_RESET PORTAbits.GPA6	// Button #1 (Reset)
#define BUTTON_START PORTAbits.GPA4	// Button #2 (Start)
#define LED1_PIN PORTBbits.RB2		// LED #1 Pin	(RB2)
#define LED2_PIN PORTAbits.RA0		// LED #2 Pin	(RA0)
#define LED3_PIN PORTAbits.RA2		// LED #3 Pin	(RA2) TP6 These will just be TTL at the test point unless you deadbug a LED and resistor onto R32
#define LED4_PIN PORTAbits.RA1		// LED #4 Pin	(RA1) TP8 These will just be TTL at the test point unless you deadbug a LED and resistor onto R38
#define VBAT_EN  PORTBbits.GPB7      // Battery Enable/Isoltation FET.  Active High to connect battery to topology and ADC voltage divider


/* ADC-Channel Memory Offset Locations */
#define ADC_MUX_VIN  (0x00 << 2)    // VIN_ANA (analog voltage proportional to Vin. Internal voltage divider = 1/13th Vin)
#define ADC_MUX_VBAT (0x15 << 2)    // GPB2, Vbat_ADC
#define ADC_MUX_IOUT (0x14 << 2)    // GPB1, +IBAT
#define ADC_MUX_VREF (0x01 << 2)    // VREGREF (reference voltage for V_REG output)  //**BRYAN** Explain this one in comments to.
#define ADC_MUX_TEMP (0x0A << 2)    // TMP_ANA (analog voltage proportional to temperature)

/* The LSB (ovfcon) for output current where the course
 * control would need to be incremented. */
#define IOUT_LSB_ROLL (18)

/* Temp Cal*/
#define TEMP_CAL_ADDR (0x0000) // There is no TEMP_CAL register so just put in a bogus value so the complier doesn't complain.

#endif
//</editor-fold>

// <editor-fold defaultstate="collapsed" desc="MCP19123 Four Switch Buck Boost Charger Demoboard">
#ifdef MCP19123_BUCK_BOOST_CHARGER		// MCP19123_FOUR_SWITCH_BUCK_BOOST_CHARGER CHARGER DEMOBOARD

#define _XTAL_FREQ 8000000		// 8Mhz FOSC
#define T0COUNTSPERSEC (1953)	// 512uS per interupt
#define SERIAL_INT_FLAG SSPIF

/* Setup the device configuration bits of the CONFIG word at 0x2007*/
/* The #DBGEN (bit 13) is managed automatically by device development tools.  For normal operation it should be a 1*/
    
#pragma config MCLRE = ON       // #MCLR Pin Function Selet (bit 5) 1(ON)=MCLR# function and weak pull-up, 0(OFF)=alternate function
#pragma config CP = OFF         // Code Protection (bit 6) 1(OFF)=Disabled, 0(ON)=Enabled
#pragma config PWRTE = OFF      // Power-Up Timer Enable (bit 4) 1(OFF)=Disabled, 0(ON)=Enabled, Note:Not Controlled by User
#pragma config WDTE = OFF       // Watchdog Timer Enable (bit 3) 1(ON)=Enabled, 0(OFF)=Disabled
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable (bit 11:10) 11(OFF)=No Protection, 00(FULL)=Entire Memory, 10(BOOT)=0x000-0x3FF, 01(HALF)=0x000-0x7FF
#pragma config BOREN = OFF      // Brown-out Reset Enable Bit (bit 8) 1(ON)=Enabled during operation and disabled in Sleep, 0 (OFF) Disabled


/* Hardware Specific Defines */
#define PCB_ID 5					// Reference design info passed to GUI 
#define BUTTON_RESET PORTBbits.GPB3	// Button #1 (Start/Stop)
#define BUTTON_START PORTBbits.GPB3	// Button #1 (Start/Stop)
#define LED1_PIN PORTAbits.RA4      // LED #1 Pin (GPA4) Fault lights
#define LED2_PIN PORTAbits.RA6      // LED #2 Pin (GPA6) Status
//#define TEST_PIN1 PORTAbits.RA0     // Test pin (GPA0)
#define TEST_PIN2 PORTBbits.GPB2    // Test Pin (GPB2)

/* ADC-Channel Memory Offset Locations */ 
#define ADC_MUX_VIN  (0x09 << 2)    // VIN_ANA (analog voltage proportional to Vin) Ratio is Vin/16
//#define ADC_MUX_VBAT (0x0A << 2)    // Output voltage measured after differential amplifier (basically +/- VSEN)
#define ADC_MUX_VBAT (0x03 << 2)    // Battery Voltage measured by buffered op-amp on GPA3
//#define ADC_MUX_IOUT (0x0D << 2)    // Average current after Sample & Hold and gain trim
//#define ADC_MUX_IOUT (0x0F << 2)    // Average output current with +6dB gain added
#define ADC_MUX_IOUT (0x04 << 2)     // Shunt current measured from op-amp
//#define ADC_MUX_VREF (0x0B << 2)    // V_REF-REG (reference voltage for V_REG output)  
#define ADC_MUX_VREF (0x11 << 2)    // Band gap voltage reference  
#define ADC_MUX_TEMP (0x01 << 2)    // Measuring the output of a MCP9701A on GPA1
//#define ADC_MUX_TEMP (0x10 << 2)    // Measuring internal temperature of MCP19123
#define ADC_MUX_BATT_TEMP (0x02 << 2) // Battery Temperature measurement from pack lead (GPA2)
#define ADC_MUX_BATT_TYPE (0x05 << 2) // Battery Chemistry resistor strap (GPB2)

/* The LSB (ovfcon) for output current where the course
 * control would need to be incremented. */
#define IOUT_LSB_ROLL (255)

/* Temp Cal*/
#define TEMP_CAL_ADDR (0x0000) // There is no TEMP_CAL register so just put in a bogus value so the complier doesn't complain.

#endif
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="MCP19125 Flyback Charger Demoboard ADM00745">
#ifdef MCP19125_ADM00745_CHARGER		// MCP19125_Flyback_CHARGER CHARGER DEMOBOARD

#define _XTAL_FREQ 8000000		// 8Mhz FOSC
#define T0COUNTSPERSEC (1953)	// 512uS per interupt
#define SERIAL_INT_FLAG SSPIF
#define PWM_MAX 0x13

/* Setup the device configuration bits of the CONFIG word at 0x2007*/
/* The #DBGEN (bit 13) is managed automatically by device development tools.  For normal operation it should be a 1*/
    
#pragma config MCLRE = ON       // #MCLR Pin Function Selet (bit 5) 1(ON)=MCLR# function and weak pull-up, 0(OFF)=alternate function
#pragma config CP = OFF         // Code Protection (bit 6) 1(OFF)=Disabled, 0(ON)=Enabled
#pragma config PWRTE = OFF      // Power-Up Timer Enable (bit 4) 1(OFF)=Disabled, 0(ON)=Enabled, Note:Not Controlled by User
#pragma config WDTE = OFF       // Watchdog Timer Enable (bit 3) 1(ON)=Enabled, 0(OFF)=Disabled
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable (bit 11:10) 11(OFF)=No Protection, 00(FULL)=Entire Memory, 10(BOOT)=0x000-0x3FF, 01(HALF)=0x000-0x7FF
#pragma config BOREN = OFF      // Brown-out Reset Enable Bit (bit 8) 1(ON)=Enabled during operation and disabled in Sleep, 0 (OFF) Disabled

/* Hardware Specific Defines */
#define PCB_ID 6					// Reference design info passed to GUI 
#define BUTTON_PRESS_n PORTBbits.GPB6	// Button #1 (Start/Stop)  Active Low
#define BUTTON_START PORTBbits.GPB6	// Button #1 (Start/Stop)  Active Low
#define LED1_PIN PORTBbits.RB1      // LED #1 Pin (GPA4) Fault lights
#define TEST_PIN PORTAbits.RA0      // GPA0 Test pin
#define Switch1 PORTAbits.RA3       // Divider On/Off
#define Switch2 PORTBbits.RB7       // =1; Vs = Vout * R31 / (R27 + R31) = Vout * 0.13; for 3 and 4 Cells
									// =0; Vs = Vout * (R31 + R41) / (R27 + R31 + R41) = Vout * 0.273; for 1 and 2 Cells 
#define Switch3 PORTAbits.RA6       // =1; Vs = Vout


/* ADC-Channel Memory Offset Locations */ 
#define ADC_MUX_VIN  (0x00 << 2)    // VIN_ANA (analog voltage proportional to Vin) Ratio is Vin/15.5328
#define ADC_MUX_VBAT (0x04 << 2)    // Battery Voltage proportional to Vout on VS pin
#define ADC_MUX_IOUT (0x06 << 2)    // Secondary Current Sense Amplifier Output A2
#define ADC_MUX_VREF (0x01 << 2)    // Vref + Vzc (DAC level plus pedestal current level setting)
#define ADC_MUX_BGR  (0x03 << 2)    // Band Gap Reference
#define ADC_MUX_TEMP (0x0D << 2)    // Measuring internal temperature of MCP19125

/* Flash location for critical value to GUI such as Temp Cal reading at 30C for instance */
#define TEMP_CAL_ADDR (0x2084)

#endif
//</editor-fold>

// <editor-fold defaultstate="collapsed" desc="PIC16F1716 MCP1631 SEPIC">
#ifdef PIC16F1716_MCP1631_SEPIC						// MCP1631RD-MCC2 DEMOBOARD with PIC16F1713/6 installed

#define _XTAL_FREQ 8000000		// 8Mhz FOSC
#define T0COUNTSPERSEC (1953)	// 512uS per interupt
#define SERIAL_INT_FLAG SSP1IF

#pragma config MCLRE = ON
#pragma config CP = OFF
#pragma config PWRTE = OFF
#pragma config WDTE = OFF
#pragma config WRT = OFF


/* Hardware Specific Defines */
#define PCB_ID 2					// Reference design info passed to GUI
#define BUTTON_START PORTAbits.RA5	// Start/Stop button on demoboard
#define BUTTON_RESET PORTAbits.RA7	// 'CHEM' button on demoboard
#define LED_LIION PORTBbits.RB0		// Li-Ion LED	(RB0)
#define LED_NIMH PORTBbits.RB1		// LED #4 Pin	(RB1)
#define LED1_PIN PORTBbits.RB2		// LED #1 Pin	(RB2)
#define LED2_PIN PORTBbits.RB3		// LED #2 Pin	(RB3)
#define LED3_PIN PORTBbits.RB4		// LED #3 Pin	(RB4)
#define LED4_PIN PORTBbits.RB5		// LED #4 Pin	(RB5)
#define MCP1631_ENABLE PORTAbits.RA4	// Enable pin for MCP1631 (/SHDN)
#define DEBUG_PIN_RC5 PORTCbits.RC5	// Status Pin

/* ADC-Channel Memory Offset Locations */
#define ADC_MUX_TEMP (0x00 << 2)	// RA0, AN0
#define ADC_MUX_VBAT (0x01 << 2)	// RA1, AN1
#define ADC_MUX_IOUT (0x02 << 2)	// RA2, AN2
#define ADC_MUX_VIN  (0x03 << 2)	// RA3, AN3
#define ADC_MUX_VREF 0				// Not available to ADC

/* Temp Cal*/
#define TEMP_CAL_ADDR (0x0000) // There is no TEMP_CAL register so just put in a bogus value so the complier doesn't complain.


/* The LSB (ovfcon) for output current where the course
 * control would need to be incremented. */
#define IOUT_LSB_ROLL (18)

#endif //PIC16F1716_MCP1631_SEPIC
//</editor-fold>

// <editor-fold defaultstate="collapsed" desc="PIC16F883 MCP1631 SEPIC Demobaord">
#ifdef PIC16F883_MCP1631_SEPIC						// MCP1631RD-MCC2 DEMOBOARD

#define PCB_ID 3					// Reference design info passed to GUI
#define _XTAL_FREQ 8000000		// 8Mhz FOSC
#define T0COUNTSPERSEC (1953)	// 512uS per interupt
#define SERIAL_INT_FLAG SSPIF

#pragma config MCLRE = ON
#pragma config CP = OFF
#pragma config PWRTE = OFF
#pragma config WDTE = OFF
#pragma config WRT = OFF


/* Hardware Specific Defines */
#define BUTTON_RESET PORTAbits.RA5	// Button #1 (Reset)
#define BUTTON_START 1				// PORTAbits.GPA4	// Button #2 (Start)  //!! Start button disabled for now:
#define LED_LIION PORTBbits.RB1		// Li-Ion LED	(RB0)
#define LED_NIMH PORTBbits.RB2		// LED #4 Pin	(RB1)
#define LED1_PIN PORTBbits.RB2		// LED #1 Pin	(RB2)
#define LED2_PIN PORTBbits.RB3		// LED #2 Pin	(RB3)
#define LED3_PIN PORTBbits.RB4		// LED #3 Pin	(RB4)
#define LED4_PIN PORTBbits.RB5		// LED #4 Pin	(RB5)
#define VREF_PWM_PIN PORTCbits.RC1	// Vref bit-banged PWM outptut
#define MCP1631_ENABLE PORTAbits.RA4 // Enable pin for MCP1631 (/SHDN)

/* ADC-Channel Memory Offset Locations */
#define ADC_MUX_VIN  (0x03 << 2)	// RA3, AN3
#define ADC_MUX_VBAT (0x01 << 2)	// RA1, AN1
#define ADC_MUX_IOUT (0x02 << 2)	// RA2, AN2
#define ADC_MUX_VREF 0				// Not available to ADC
#define ADC_MUX_TEMP (0x00 << 2)	// RA0, AN0

#define REF_PWM_MAX_COUNT 100

/* Temp Cal*/
#define TEMP_CAL_ADDR (0x0000) // There is no TEMP_CAL register so just put in a bogus value so the complier doesn't complain.

/* The LSB (ovfcon) for output current where the course
 * control would need to be incremented. */
#define IOUT_LSB_ROLL (18)

#endif
//</editor-fold>

#ifdef	__cplusplus
}
#endif

#endif	/* MULTICHEMBATTERYCHARGER_HARDWARE_H */

