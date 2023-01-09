
/********************************************************************
 *
 *					Multi-Chemistry Battery Charger
 *
 ********************************************************************
 *
 * FileName:        MultiBatteryCharger.c
 * Dependencies:    See INCLUDES section below
 * Processor:       Miscellaneous
 * Compiler:        XC8 v2.05
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement:
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the "Company") for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company's customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 */

// ENVIRONMENT SETUP //

/*
 * Be sure to disable programming of the memory region between 0xE60 to 0xEBF
 * to prevent the calibration and configuration registers from being overwritten
 * every time you change code.  Click on project properties and navigate here:
 *		 ICD3 or ICD4 or PKOB Nano or PICkit 'Memories to Program' tab, 
 *         'Preserve Memory Regions' checkbox and values
 * Also reserve a memory block in the linker for the fixed code at the following location:
 *	Project Properties: XC8 Global Options:  XC8 Linker: Option categories drop-down select Memory Model.
 *  Enter the following: default,-0e60-0eff  (no spaces)
 * If building for a PIC16F17xx series part, then the offset changes to 0x3E60 to 0x3EBF
 *  and in the linker: default,-3e60-3eff (no spaces).
*/

#include <xc.h>
#include <pic.h>
#include <stdint.h>
#include "MultiChemCharger_Main.h"
#include "MultiChemCharger_Hardware.h"
#include "MultiChemCharger_Serial.h"

/*
                         Main application
 */

/********************** System Variables ****************************/
struct charger_data_t cd;		// Initialize the Charger Live Data structure called 'cd'
struct charger_settings_t cs;	// Initialize the Charger Settings structure called 'cs'

//unsigned short _adc;            // 12-bit accumulator and result from the ADC averaging routine.
uint16_t _adc;
volatile bit adc_done;          // Flag that all ADC channels have been sampled (in sequence) and are ready to be used
volatile bit half_period;       // Flag signaling which half of the timer period the loop is 
unsigned short _adc_vbat[4];             // Accumulator used to measure dV/dt of battery
unsigned short _adc_temp[4];             // Accumulator used to measure dt/dt of battery
unsigned short _adc_iout[4];             // Accumulator used to measure dI/dt of battery
unsigned short bak_vbat_cv;     // Backup of the cs.vbat_cv contents 

unsigned char t0cnt;            // Counter of T0 (512uS ticks) used by the ADC to count the number of samples taken in an average
bit chrg_off_batt_connected;	// Flag to set the charger off, but connect the battery for calibration
bit shutdown;                   // Flag to request the charger state machine to shut down (from GUI)
bit start;                      // Flag to request the charger state machine to start up (from GUI)
unsigned short button_cnt;      // Counter to count the number of 512usec cycles elapsed for button debounce and latch
unsigned short second_counter;	// Counter to count off the number of seconds passed during charging

#if defined(ENABLE_NIMH) || defined(ENABLE_VRLA_CC) || defined(ENABLE_VRLA_FAST)
bit VRLAdvdt;           
bit VRLA_eof_dvdt;
bit VRLAFdvdt;
bit VRLAF_eof_dvdt;
bit VRLAFdidt;
bit VRLAF_eof_didt;
#endif

void main()
{
    load_saved_variables();     // Load the values stored either in EEPROM or FLASH
    init_hardware();			// Configure the hardware specific parameters, registers, and I/O
    init_variables();			// This initializes the configuration and many variables, some dependancies on load_saved_variables
    init_cal_registers();		// Calibrate various internal registers (Oscillator, offset voltages, etc.)
    init_serial_io();			// Initialize the serial I/O system if enabled
    // Start the ADC and preload valid data //

    while (1)	// Main While Loop:
    {
        if (SERIAL_INT_FLAG == 1)       // SMBUS system interrupt ready?
        {	
            service_SMBUS();            // Yes, an event is pending in the SMBUS system
        }

        if (T0IF)                       // 256uS timer timed-out?
        {
            T0IF = 0;                   // Reset the timer
            half_period = !half_period;
            if (half_period)            // 512usec have passed since last true 
            {
                t0cnt++;                    // Increment the timer counter variable (512uS timebase)
                check_button();             // Look for button presses to start/stop the charger
                hardware_custom_a();        // Placeholder for hardware specific custom code
                adc_capture();              // ADC capture state machine
                // Decrement the second counter
                if (second_counter != 0)    
                {
                    second_counter --;
                }  
                // Update charger activity 
                if (adc_done == 1)				// Is the ADC done sampling?
                {
                    charger_protection();       // Check for any shutdown causes
                    charger_state_machine();	// Yes, so update the charger state machine
                    update_user_interface();	// Go update the user interface
                    adc_done = 0;               // Clear the ADC complete bit
                }
            }
            else                            // 512usec have passed since the last time this was true, 256usec offset the rest of the loop
            {
                ADCON0 |= 0x02;				// Begin ADC conversion     
            }       		         
        } 
    } 
    return; 
}

void load_saved_variables(void)
{    
    uint16_t i;
    uint16_t *ptr ;
    ptr = (uint16_t *)&cs;
    for(i=0; i<34; i++)
    {
        ptr[i] = read_flash(SETTINGS_ADDR + i);
    }
    ptr[33] &= 0x00ff;
    ptr[33] |= (read_flash(SETTINGS_ADDR + 34) << 8 & 0xff00);

    cs.endian = 0x1234;
    cs.fwrev = FWREV;
    cs.vbat_pc_25C = cs.vbat_pc;
    cs.vbat_fv_25C = cs.vbat_fv;
    cs.vbat_cv_25C = cs.vbat_cv;
    bak_vbat_cv    = cs.vbat_cv;        //Here we're storing a copy of the vbat_cv. Some parts of the state machine modify vbat_cv.
    if (cs.dtdt_slope & 0x2000)         // It was negative, must sign extend 
    {   
        cs.dtdt_slope = (unsigned short)(cs.dtdt_slope) | 0xc000;
    }
    if (cs.neg_dvdt & 0x2000)         // It was negative, must sign extend 
    {   
        cs.neg_dvdt = (unsigned short)(cs.neg_dvdt) | 0xc000;
    }
    if (cs.neg_didt_cv & 0x2000)         // It was negative, must sign extend 
    {   
        cs.neg_didt_cv = (unsigned short)(cs.neg_didt_cv) | 0xc000;
    }

    /* Determine if settings are ok for charging. */
    /* All configuration validity challenges need to be added below.  If a challenge fails then set cd.status.configerror = 1. */
    // USER add your checks here.
    cd.status.configerror = 0;
}

void update_user_interface(void)	// Call the appropriate user interface routine based on the
{									// selected configuration.
		#ifdef ENABLE_STATUS_LEDS
			update_status_leds();		// Update the status LEDs
		#endif
}

void init_variables(void) {
	/* Initialize everything to known good values */
    _adc = 0;
    adc_done = 0;
    t0cnt = 0;
    second_counter = 0;
    button_cnt = 0;
    adc_reading = ADC_VIN;
    button_state = NOT_PRESSED;

    cd.adc_vin = 0;
    cd.adc_vbat = 0;
    cd.adc_iout = 0;
    cd.adc_vref = 0;
    cd.adc_temp = 0;
    cd.charge_timer = 0;
    cd.charger_state = CHARGER_OFF;
    cd.adc_reading = ADC_VIN;
    cd.restore_timer = 0;
    cd.rapid_charge_timer = 0;
    cd.status.word = 0;
    //cd.eoc_status.word = 0;
	cd.ref_design_PCB = PCB_ID;	// Tell GUI what type of PCB is active

    cd.dvdt_count = DxDT_BLANK_INTERVAL;
	cd.dtdt_count = DxDT_BLANK_INTERVAL;
    cd.Pdvdt_count = DxDT_BLANK_INTERVAL;
    cd.didt_count = DxDT_BLANK_INTERVAL;

	/* Sealed Lead Acid parameters */
#ifdef ENABLE_VRLA_CC
	VRLAdvdt = 0;
    VRLA_eof_dvdt = 0;
#endif     
#ifdef ENABLE_VRLA_FAST    
    VRLAFdvdt = 0;
    VRLAF_eof_dvdt = 0;
    VRLAF_eof_didt = 0;
    VRLAFdidt = 0;
#endif    

	/* Startup flags */
	chrg_off_batt_connected = 0;
	start = 0;
	shutdown = 0;
    sfr_access = 0;
}

void adc_capture()
{
	// Sample each input in sequence 16 times then divide result by four.
	// This effectively adds two bits to the 10-bit ADC giving you a 12-bit result.
	// Each sample happens every 256 T0 ticks or [256 x (1/(8Mhz/(4*4))]=512uS
	// The four inputs below require 4ch x 512uS x 16samples = 32.768mS to complete.  This
	// rate defines how often the charge_state() routine executes via the adc_done flag.
    // NOTE: This routine does not start the ADC sampling.  It just sets it up.  The start command 
    //       happens about 256usec later in the main loop.
    
    _adc += (unsigned short)((ADRESH << 8) | ADRESL);             // Get ADC result into _adc (This is right-justified data)

	if ((t0cnt & 0x0F) == 0)                    // If the timer routine has executed 16 times (512*16 = 8.192mS). (mask only looks at lower nibble so resetting isn't required)
	{
		t0cnt = 0;                            // Clear counter 
		_adc >>= 2;                             // Divide result by four for a 12bit result.
		switch (adc_reading)                    // Store the result in the appropriate register
		{
            case ADC_VIN:                       
                cd.adc_vin = _adc;				// Store _adc value in the appropriate register
                adc_reading = ADC_VBAT;			// Set case statement to the next ADC input
                ADCON0 = ADC_MUX_VBAT | 0x01;	// Change ADC input to the -next- ADC channel
                break;  

            case ADC_VBAT:                      // Same as above
                cd.adc_vbat = _adc;
                adc_reading = ADC_IOUT;
                ADCON0 = ADC_MUX_IOUT | 0x01;
                break;

            case ADC_IOUT:                      // Same as above
                cd.adc_iout = _adc;
                adc_reading = ADC_TEMP;
                ADCON0 = ADC_MUX_TEMP | 0x01;
                break;

            case ADC_TEMP:                      // Same as above
                cd.adc_temp = _adc;
                adc_reading = ADC_VIN;
                ADCON0 = ADC_MUX_VIN | 0x01;
                adc_done = 1;                   // Last of the series of ADC channels to sample,
                break;							// so flag other routines that the data is new and valid
   
            default:
                adc_reading = ADC_VIN;			// In case there isn't a proper value already in adc_reading, give a default value. Usually first iteration.
                break;
		}                                       // end switch
        _adc = 0;                               // Clear the register for the next 16 ADC samples
	}                                           // end ADC channel sample period
}                                               

void charger_protection()                               
{                
    /******************************************************************************************************/
    /* These are routines for protecting the charger and looking for reasons to shut it down.             */
    /* For instance in every state there is a desire to monitor for a negative change in                  */
    /* voltage to indicate that the termination voltage has been reached.                                 */
    /* These routines are based on a fast loop that runs once per second and a slow loop that             */
    /* runs once every 16 seconds.                                                                        */
    /******************************************************************************************************/
    unsigned char a;
    // <editor-fold defaultstate="collapsed" desc="1s Fast Loop with Nested 16s Slow Loop checking Charging Safety">
    if (second_counter == 0)                            // Check timer to see if one second has elapsed
    {
        // <editor-fold defaultstate="collapsed" desc="Increment Charge Timer. Shutdown Charger if Maximum Charge Time has been reached">
        second_counter = T0COUNTSPERSEC;	// Reset the 1-second countdown timer (based on 512uS per every other T0 overflow)
        if (cd.charger_state != CHARGER_OFF)			// Are we charging?
        {
            cd.charge_timer ++;	
            if (cd.charge_timer >= cs.charge_time_max)	// Are we timed-out? (Has the maximum charge time as set in the charger setting been reached?)
            {
                if (cs.charge_time_max != 0)
                {
                    cd.charger_state = CHARGER_OFF;			// If Yes, then shutdown the charger
                    cd.status.timeout = 1;	// And log the shutdown reason     
                }
            }
        }
        //</editor-fold>
        
/***************************************************************************************************************/
/* Next is a slower loop that watches cd.charge_timer and executes every so many seconds. Usually 16 or 64.    */
/* The purpose of the loop is to look at various conditions to determine if the charger needs to be shut down. */
/***************************************************************************************************************/
    //<editor-fold defaultstate="collapsed" desc="Contains a 16s loop that checks for dT/dt and dV/dt as appropriate per chemistry.">
//        if ((cd.charge_timer & 0x000F) == 0x0000)            // 16 seconds increments of the total charge time counter
		if ((cd.charge_timer & 0x003F) == 0x0000)            // 64 seconds increments of the total charge time counter
        {                                              

/*********************************************************/
/* Add up all the deviations between successive samples. */
/*********************************************************/    
	        // <editor-fold defaultstate="collapsed" desc="Integrate Voltage, Temperature, & Current">
	        cd.dvdt = (cd.adc_vbat - _adc_vbat[3]) +    
	                 (_adc_vbat[3] - _adc_vbat[2]) +
	                 (_adc_vbat[2] - _adc_vbat[1]) +
	                 (_adc_vbat[1] - _adc_vbat[0]);
            
	        cd.dtdt = (cd.adc_temp - _adc_temp[3]) +
	                (_adc_temp[3] - _adc_temp[2]) +
	                (_adc_temp[2] - _adc_temp[1]) +
	                (_adc_temp[1] - _adc_temp[0]);

	        cd.didt = (cd.adc_iout - _adc_iout[3]) +
	                (_adc_iout[3] - _adc_iout[2]) +
	                (_adc_iout[2] - _adc_iout[1]) +
	                (_adc_iout[1] - _adc_iout[0]);
	        //</editor-fold>
        
/*********************************************************/
/* Shift the accumulated values of the array down.       */
/*********************************************************/            
	        // <editor-fold defaultstate="collapsed" desc="Roll the values over time.">
	        _adc_vbat[0] = _adc_vbat[1];
	        _adc_vbat[1] = _adc_vbat[2];
	        _adc_vbat[2] = _adc_vbat[3];
	        _adc_vbat[3] = cd.adc_vbat;

	        _adc_temp[0] = _adc_temp[1];
	        _adc_temp[1] = _adc_temp[2];
	        _adc_temp[2] = _adc_temp[3];
	        _adc_temp[3] = cd.adc_temp;

	        _adc_iout[0] = _adc_iout[1];
	        _adc_iout[1] = _adc_iout[2];
	        _adc_iout[2] = _adc_iout[3];
	        _adc_iout[3] = cd.adc_iout;           
	        //</editor-fold>        
        
	        // <editor-fold defaultstate="collapsed" desc="Check for precondition failures.">
	        if((cd.precondition_timer < cs.precondition_max_time) &&
#ifdef ENABLE_LIION
	            (cd.charger_state == CHARGER_LIION_PRECONDITION) |
#else
	            0 |
#endif            
#ifdef ENABLE_LIFEPO4
	            (cd.charger_state == CHARGER_LIFEPO4_PRECONDITION) |  
#else
	            0 |
#endif
#ifdef ENABLE_NIMH
	            (cd.charger_state == CHARGER_NIMH_RESTORE)       |
#else 
	            0 |
#endif
#ifdef ENABLE_VRLA_FAST
	            (cd.charger_state == CHARGER_VRLAF_RESTORE) |
#else 
            	0 |        
#endif
#ifdef ENABLE_VRLA_CC
	            (cd.charger_state == CHARGER_VRLA_PRECONDITION)
#else
            	0        
#endif
        		)   
        	{    
	            if (cd.dvdt < (cs.pc_dvdt)) 
	            {
	                /* Countdown the blanking interval */
	                if (cd.dvdt_count != 0) 
	                {
	                    cd.dvdt_count--;
	                }
	            } 
	            else /* Reset the blanking interval */ 
	            {
	                cd.dvdt_count = DxDT_BLANK_INTERVAL;
	            }
	        }

	        //</editor-fold>
        
	        // <editor-fold defaultstate="collapsed" desc="NIMH, VRLA, VRLA_FAST: Detect Termination Conditions">
/***************************************************************************************************************/            
/* Only enable the following slope detection for battery types that implement it.                              */
/* NIMH, VRLA, and VRLA_FAST charging slope detection are implemented below. Every 16 (or 64) seconds the most */
/* significant element of a four element array (e.g. _adc_vbat[3]) will be populated with a current ADC sample */
/* for Vbat and Temp. Each 16 (or 64) seconds every element in the array is shifted one less significant in    */
/* in the array.  There is a DxDT blanking interval to allow the loop to run enough to populate these          */
/* arrays for comparison.                                                                                      */
/***************************************************************************************************************/

            #if defined(ENABLE_NIMH) || defined(ENABLE_VRLA_CC) || defined(ENABLE_VRLA_FAST)                                                            
            
/*************************************************************************************************************/
/* VRLAF_RAPID, VRLA_CC, NiMH: If we have a negative dV/dt slope, countdown unless blanking.                 */
/* Also, over a longer 32s (or 128s) period check for a positive dV/dt slope, and countdown unless blanking. */
/*************************************************************************************************************/
            // <editor-fold defaultstate="collapsed" desc="VRLAF_RAPID, VRLA_CC, NiMH: Negative dV/dt Check, Decrement Counter">
                         
            if((cd.rapid_charge_timer > cs.dvdt_blank_time) &&
#ifdef ENABLE_NIMH
            (cd.charger_state == CHARGER_NIMH_RAPID)       |
#else 
            0 |
#endif
#ifdef ENABLE_VRLA_FAST
            (cd.charger_state == CHARGER_VRLAF_RAPID) |
#else 
            0 |
#endif
#ifdef ENABLE_VRLA_CC
            (cd.charger_state == CHARGER_VRLA_CC)
#else
            0        
#endif
            )  
            {
                //Looking for a negative slope
                if (cd.dvdt < (cs.neg_dvdt)) 
                {
                    /* Countdown the blanking interval */
                    if (cd.dvdt_count != 0) 
                    {
                        cd.dvdt_count--;
                    }
                } 
                else /* Reset the blanking interval */ 
                {
                    cd.dvdt_count = DxDT_BLANK_INTERVAL;
                }
                
#if defined(ENABLE_VRLA_CC) || defined(ENABLE_VRLA_FAST)
                /*********************************************************************************************************************/
                /* Over a 32 second period check for a Positive dV/dt Slope.                     */
                /*********************************************************************************************************************/                  
                cd.dvdt = (_adc_vbat[3] - _adc_vbat[1]);  //Difference of the present reading against the one from 32s (or 128s) earlier.

                if (cd.dvdt <= cs.pos_dvdt_cc) 
                {
                    if (cd.Pdvdt_count != 0)    /* Countdown the blanking interval */
                    {
                        cd.Pdvdt_count--;
                    }
                } 
                else                          /* Reset the blanking interval */ 
                {
                    cd.Pdvdt_count = DxDT_BLANK_INTERVAL;
                }
                if (VRLAFdvdt) //VRLAFdvdt is set by the charge profile when cd.Pdvdt_count == 0;
                {
                    if (cd.dvdt < cs.pos_dvdt_cc) 
                    {
                        VRLAF_eof_dvdt = 1; //This end of charge flag isn't set though until the rate of change of Pdvdt stays below cs.pos_dvdt_cc.
                                            //This is basically looking for the rise in voltage to start slowing down (helps account for aging batteries).
                        cd.status.negdvdt = 0;
                    }
                }
#endif

            }
            //</editor-fold>            
/*********************************************************************************************************************/
/* Check for the change in current -dI/dt flattening out indicating high leakage before reaching trickle current     */
/*********************************************************************************************************************/            
            //<editor-fold defaultstate="collapsed" desc="VRLAF_TRICKLE dI/dt Check">
#ifdef ENABLE_VRLA_FAST
        if ((cd.charger_state == CHARGER_VRLAF_TRICKLE) &&
                    (cd.restore_timer < cs.restore_time_max) &&
                    (cd.restore_timer > cs.dvdt_blank_time))
            {            
                if (cd.didt >= cs.neg_didt_cv)
                {
                    if (cd.didt_count != 0)
                    {
                        cd.didt_count--;
                    }
                }            
                else
                {
                    cd.didt_count = DxDT_BLANK_INTERVAL;               
                }
            }
            if (VRLAFdidt)
            {
                if (cd.didt > cs.neg_didt_cv)
                    VRLAF_eof_didt = 1;
            }
#endif        
            //</editor-fold>
            
/**************************************************************/
/* If there is a thermistor, check to see if the dT/dt        */
/* slope is higher than the threshold. This algorithm assumes */
/* NTC thermistor and a NiMH battery chemistry.               */
/**************************************************************/
            // <editor-fold defaultstate="collapsed" desc="dT/dt Shutdown Detection">
            #ifdef ENABLE_TEMP_DTDT_SHUTDOWN   /* Terminate charge, dT/dt detection */

            if (cd.dtdt < cs.dtdt_slope)    // Negative slope (NTC Sensor): Rising temperatures indicates EOC
            {
                if (cd.dtdt_count != 0) 
                {
                    cd.dtdt_count--;        // Need multiple triggers to cause shutdown
                }
                else
                    cd.status.dtdtshdn = 1; // Shutdown due to rapidly rising temperature
            }
            else 
            {
                cd.dtdt_count = DxDT_BLANK_INTERVAL;
            }
            #endif
            //</editor-fold>
            
            #endif            
        //</editor-fold>
        //</editor-fold>   
        }
     //</editor-fold>
    
        
        /* END OF 1-MINUTE CONDITION CHECKS */
            
/***************************************************************************************************************/
/* Adjust output setpoint voltage based on ambient temperature                                                 */
/***************************************************************************************************************/        
    //<editor-fold defaultstate="collapsed" desc="Temp Sensor based output voltage adjustment">
#ifdef ENABLE_TEMP_SENSOR_VOUT_ADJUSTMENT
        if (cs.temp_sense_en == 1)
        {                      
            cs.vbat_cv = cs.vbat_cv_25C;
            cs.vbat_pc = cs.vbat_pc_25C;
            cs.vbat_fv = cs.vbat_fv_25C; 
            short temp_diff = (cd.adc_temp - cs.pack_temp_25C);
            unsigned short temp_counter = 0;           

            if (temp_diff >= 0)
            {
                while (temp_diff >= cs.temp_1C)
                {
                    temp_diff -= cs.temp_1C;
                    temp_counter++;
                }
                for (unsigned short i = 0; i <= temp_counter; i++)
                {
                    cs.vbat_cv = cs.vbat_cv - cs.vbat_1c_adjust;
                    cs.vbat_pc = cs.vbat_pc - cs.vbat_1c_adjust;
                    cs.vbat_fv = cs.vbat_fv - cs.vbat_1c_adjust;
                };
            }
            else
            {
                while (temp_diff <= -cs.temp_1C)
                {
                    temp_diff =+ cs.temp_1C;
                    temp_counter++;
                }
               
                for (unsigned short i = 0; i <= temp_counter; i++)
                {
                    cs.vbat_cv = cs.vbat_cv + cs.vbat_1c_adjust;
                    cs.vbat_pc = cs.vbat_pc + cs.vbat_1c_adjust;
                    cs.vbat_fv = cs.vbat_fv + cs.vbat_1c_adjust;
                };
            }
        }
      //</editor-fold>
        
/***************************************************************************************************************/
/* The final part of the fast loop is to increment the various charge state timers.                         */
/***************************************************************************************************************/ 
    //<editor-fold defaultstate="collapsed" desc="Increment Timers">

#endif        
        if(
#ifdef ENABLE_NIMH
            (cd.charger_state == CHARGER_NIMH_RESTORE) |
#else 
            0 |
#endif
#ifdef ENABLE_VRLA_FAST
            (cd.charger_state == CHARGER_VRLAF_TRICKLE)
#else 
            0      
#endif
        )  
            
        {
            cd.restore_timer++;
        }
        
        if(
#ifdef ENABLE_NIMH
            (cd.charger_state == CHARGER_NIMH_RAPID) |
#else 
            0 |
#endif
#ifdef ENABLE_VRLA_FAST
            (cd.charger_state == CHARGER_VRLAF_RAPID)
#else 
            0        
#endif
        )         
        {
            cd.rapid_charge_timer++;
        }
        
        if(
#ifdef ENABLE_LIION
            (cd.charger_state == CHARGER_LIION_PRECONDITION) |
#else
            0 |
#endif            
#ifdef ENABLE_LIFEPO4
            (cd.charger_state == CHARGER_LIFEPO4_PRECONDITION) |
#else
            0 |
#endif
#ifdef ENABLE_NIMH
            (cd.charger_state == CHARGER_NIMH_RESTORE)       |
#else 
            0 |
#endif
#ifdef ENABLE_VRLA_FAST
            (cd.charger_state == CHARGER_VRLAF_RESTORE) |
#else 
            0 |  
#endif
#ifdef ENABLE_VRLA_CC
            (cd.charger_state == CHARGER_VRLA_PRECONDITION)
#else
            0        
#endif
        )   
    
        {
            cd.precondition_timer++;
        }
    //</editor-fold>
    }  
    
    //cd.status.word = 0;                                 // Clear out the status word and re-populate  
    // </editor-fold>  

    // <editor-fold defaultstate="collapsed" desc="Final check for conditions that will force the charger off. Shutdown. Log the reason">
/*******************************************************************************************************************/
/* The last part of the protection routine checks the A2D values against safety thresholds for immediate force off.*/
/*******************************************************************************************************************/
    
    /* Input under voltage lockout */
    a = check_for_uvlo();
    if (a)
    {
        cd.status.vinuvlo = 1;
    }
    else
    {
        cd.status.vinuvlo = 0;
    }

	/* Output overvoltage */
	if (cd.adc_vbat >= cs.vbat_ov) 
    {
		cd.status.voutovlo = 1;
	}
    else
    {
        cd.status.voutovlo = 0;
    }
#ifdef ENABLE_TEMP_SENSOR
    if (cs.temp_sense_en == 1)
    {
        /* Pack under-temperature (assumes linear sensor) */ 
        if (cs.pack_temp_min < cd.adc_temp) 
        {
            cd.status.undertemp = 1;
        }
        else
        {
            cd.status.undertemp = 0;
        }

        /* Pack over-temperature (assumes linear sensor) */
        if (cs.pack_temp_max > cd.adc_temp)
        {
            cd.status.overtemp = 1;
        }
        else
        {
            cd.status.overtemp = 0;
        }
    }
#endif 
       
    
	/* Off button */
	if (shutdown /*|| !BUTTON_START*/) 
    {
        cd.status.complete = 1;
        cd.status.buttonshdn = 1;
	}
    
    /* Disable Command that leaves the battery connected for calibration */
    if (chrg_off_batt_connected)
    {
        connect_battery();
    }

	/* If we are shutting down, record why... (mask completed charge detection)
	 * it's a handy flag for status, but not unconditionally terminate charge. */
	if ((cd.charger_state != CHARGER_OFF) && ((cd.status.word & 0x7fff) != 0)) //If any faults are logged (except for complete), then set state to OFF
	{
        cd.charger_state = CHARGER_OFF;
//!! Causes bad data on last sampling
//		adc_done = 1;
		shutdown = 0; //clears the shutdown bit %%BRYAN%% probably should move up several lines into the if(shutdown) line
        LED_OnTime = LED_250;
        LED_OffTime = LED_250;
	
    //</editor-fold>
    }
    //</editor-fold>
}



void charger_state_machine()
{
    /* Update charge activity */
    
    short x;			// Return value from dec_iout() routine
    char i;
    short increment;

    switch (cd.charger_state) 
    {
        case CHARGER_OFF:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_OFF">
            LED_OnTime = LED_150;
            LED_OffTime = LED_850 ;
            /* Disable Charger and Set the output current to minimum */
            disable_charger();
            /* Indicate shutdown */
            shutdown = 0; //clears the shutdown bit

            /* Turn on charger only if there is not UVLO, */
            /* the device is configured, and the battery */
            /* voltage is below maximum */
            if( start &&	// Start enabled) and,
                ((cd.status.word & 0x7FFF) == 0))		// None of the status registers are set indicating error conditions 
            {
                start = 0;
                LED_OnTime = LED_250;
                LED_OffTime = LED_750 ;
                /* Charge timer */
                cd.charge_timer = 0;
                second_counter = T0COUNTSPERSEC;

                /* Clear all of the variables pertinent to charging */
                for (i = 0; i< 4; i++)
                {
                    _adc_vbat[i] = cd.adc_vbat;
                    _adc_temp[i] = cd.adc_temp;
                    _adc_iout[i] = cd.adc_iout;
                };

                cd.dvdt = 0;
                cd.dtdt = 0;
                cd.didt = 0;
                cd.dvdt_count = DxDT_BLANK_INTERVAL;
                cd.dtdt_count = DxDT_BLANK_INTERVAL;
                cd.didt_count = DxDT_BLANK_INTERVAL;                
                cd.restore_timer = 0;
                cd.rapid_charge_timer = 0;
                cd.status.word = 0;
                cd.precondition_timer = 0;
                cd.Pdvdt_count = DxDT_BLANK_INTERVAL;
                cd.status.complete = 0;
                cs.vbat_cv = bak_vbat_cv;
                
                enable_charger();	// Turn on the charger hardware (topology dependent function)

                /* Begin the charger state machine */
#ifdef ENABLE_LIION
                if (cs.chemistry == CHEMISTRY_LITHIUM) 
                {
                    cd.charger_state = CHARGER_LIION_START;
                } 
#endif
#ifdef ENABLE_VRLA_CC                    
                if (cs.chemistry == CHEMISTRY_VRLA_CC) 
                {
                    cd.charger_state = CHARGER_VRLA_START;
                } 
#endif             
#ifdef ENABLE_VRLA_FAST
                if (cs.chemistry == CHEMISTRY_VRLA_FAST) 
                {
                    cd.charger_state = CHARGER_VRLAF_START;                    
                } 
#endif                
#ifdef ENABLE_LIFEPO4
                if (cs.chemistry == CHEMISTRY_LIFEPO4) 
                {
                    cd.charger_state = CHARGER_LIFEPO4_START;
                } 
#endif                    
#ifdef ENABLE_SUPERCAP
                if (cs.chemistry == CHEMISTRY_SUPERCAP) 
                {
                    cd.charger_state = CHARGER_SUPERCAP_START;
                } 
#endif                    
#ifdef ENABLE_NIMH
                if (cs.chemistry == CHEMISTRY_NIMH)
                {
                    cd.charger_state = CHARGER_NIMH_START;
                }
#endif
                else  	/* Config error - User selected a non-supported chemistry in the state machine */
                {
                    cd.charger_state = CHARGER_OFF;
                    cd.status.complete = 1;
                    cd.status.configerror = 1;
                }
            }
            break;
            //</editor-fold>
    
    /* Lithium Ion */
        #ifdef ENABLE_LIION
        case CHARGER_LIION_START:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_LIION_START">
            if (cd.adc_vbat > (cs.vbat_cv >> 1)) // Crude start condition to make sure there *IS* a battery present
            {
                cd.charger_state = CHARGER_LIION_PRECONDITION;
            }
            break;
            //</editor-fold>
        case CHARGER_LIION_PRECONDITION:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_LIION_PRECONDITION">
            if (cd.adc_iout < cs.ibat_pc) 
            {
                inc_iout(1);
            } 
            else 
            {
              if (cd.adc_vbat > cs.vbat_pc) 
              {
                    cd.charger_state = CHARGER_LIION_CC;
              }
            }
            if ((cd.dvdt_count == 0) || (cd.precondition_timer > cs.precondition_max_time)) 
            {
                cd.status.preconditionfail = 1;
                cd.charger_state = CHARGER_OFF;
            }            
            break;
            // </editor-fold>
        case CHARGER_LIION_CC:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_LIION_CC">
            LED_OnTime = LED_500;
            LED_OffTime = LED_500;
            if (cd.adc_vbat < cs.vbat_cv) 
            {
                if (cd.adc_iout < cs.ibat_cc)
                {                    
#ifdef FAST_RAMP_ENABLED                                                    
                    if (cd.adc_iout < (cs.ibat_cc - FAST_RAMP_ADC_COUNTS) )	// If enabled, fast ramp the current only if it is far too low
                    {
                        increment = FAST_RAMP_INCREMENT;
                    }
                    else
                    {
                        Increment = 1;
                    }
#else
                    increment = 1;
#endif
                    inc_iout(increment);
                }
                else if (cd.adc_iout > (cs.ibat_cc + 40)) //%%BRYAN%% This is a little debug attempt at reining in overshoot
                {                                         //          when manually controlling the maximum duty cycle or OVREFOUT.
                    dec_iout(1);
                }
            } 
            else 
            {
                cd.charger_state = CHARGER_LIION_CV;
            }
            if ((cs.rapid_time_max != 0) && (cd.rapid_charge_timer >= cs.rapid_time_max))
            {
                cd.charger_state = CHARGER_LIION_CV;
            }
            break;
            // </editor-fold>
        case CHARGER_LIION_CV:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_LIION_CV">
            LED_OnTime  = LED_750;
            LED_OffTime = LED_250;
            if (cd.adc_vbat > cs.vbat_cv)                
            {
                x = dec_iout(1);	// Decrement the current
                if(!x)			// If we can't drop the current any lower, there is a problem. Shutdown
                {
                    cd.charger_state = CHARGER_OFF;
                    cd.status.complete = 1;
                    cd.status.didtshdn = 1; 	
                }
            }
            

            if (cd.adc_iout < cs.ibat_ct) 
            {
                cd.status.complete = 1;
                cd.charger_state = CHARGER_OFF;
            }
            break;
            // </editor-fold>
        #endif
    
    /* Lithium Iron Phoshpate */
        #ifdef ENABLE_LIFEPO4		
        case CHARGER_LIFEPO4_START:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_LIFEPO4">
            cd.charger_state = CHARGER_LIFEPO4_PRECONDITION;
            break;
            //</editor-fold>
        case CHARGER_LIFEPO4_PRECONDITION:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_LIFEPO4">
            if (cd.adc_iout < cs.ibat_pc) 
            {
              inc_iout(1);
            } 
            else 
            {
                if (cd.adc_vbat > cs.vbat_pc) 
                {
                    cd.charger_state = CHARGER_LIFEPO4_CC;
                }
            } 
            if ((cd.dvdt_count == 0) || (cd.precondition_timer > cs.precondition_max_time)) 
            {
                cd.status.preconditionfail = 1;
                cd.charger_state = CHARGER_OFF;
            }            
            break;
            //</editor-fold>
        case CHARGER_LIFEPO4_CC:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_LIFEPO4_CC">
            LED_OnTime = LED_500;
            LED_OffTime = LED_500;
            if (cd.adc_vbat < cs.vbat_cv) 
            {
                if (cd.adc_iout < cs.ibat_cc)
                {
#ifdef FAST_RAMP_ENABLED                                                    
                    if (cd.adc_iout < (cs.ibat_cc - FAST_RAMP_ADC_COUNTS) )	// If enabled, fast ramp the current only if it is far too low
                    {
                        increment = FAST_RAMP_INCREMENT;
                    }
                    else
                    {
                        Increment = 1;
                    }
#else
                    increment = 1;
#endif
                    inc_iout(increment);
                }
            } 
            else 
            {
                cd.charger_state = CHARGER_LIFEPO4_CV;
            }
            if ((cs.rapid_time_max != 0) && (cd.rapid_charge_timer >= cs.rapid_time_max))
            {
                cd.charger_state = CHARGER_LIFEPO4_CV;
            }
            break;
            //</editor-fold>
        case CHARGER_LIFEPO4_CV:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_LIFEPO4_CV">
            LED_OnTime = LED_750;
            LED_OffTime = LED_250;
            if (cd.adc_vbat > cs.vbat_cv) {
                if(x = dec_iout(1))			// If we can't drop the current any lower, there is a problem. Shutdown
                {
                    cd.charger_state = CHARGER_OFF;
                    cd.status.complete = 1;
                    cd.status.didtshdn = 1;
                }
            }
            if (cd.adc_iout < cs.ibat_ct) {
                cd.charger_state = CHARGER_OFF;
                cd.status.complete = 1;
            }
            break;
            //</editor-fold>
        #endif

    /* Nickel Metal Hydride */ 
        #ifdef ENABLE_NIMH
        /* If running the MCP19125, then search the comments below for special changes to code */
        case CHARGER_NIMH_START:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_NIMH_START">
            cd.charger_state = CHARGER_NIMH_RESTORE;
            break;
            //</editor-fold>
        case CHARGER_NIMH_RESTORE:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_NIMH_RESTORE">
            if (cd.adc_iout < cs.ibat_pc) 
            {
                inc_iout(1);
            } 
            else 
            {
                if (cd.adc_vbat > cs.vbat_pc) 
                {
                    cd.charger_state = CHARGER_NIMH_RAPID;
					cd.dvdt_count = DxDT_BLANK_INTERVAL;
                }
            }

            if ((cd.dvdt_count == 0) || (cd.precondition_timer > cs.precondition_max_time)) 
            {
                cd.status.preconditionfail = 1;
                cd.charger_state = CHARGER_OFF;
            }
            break;
            //</editor-fold>
        case CHARGER_NIMH_RAPID:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_NIMH_RAPID">
            LED_OnTime = LED_500;
            LED_OffTime = LED_500;            /* Does the current need to be incremented? */

            if (cd.adc_iout < cs.ibat_cc)
            {
                inc_iout(1);
            }

            /* Rapid charge timeout */
            if (cd.rapid_charge_timer >= cs.rapid_time_max) 
            {
                cd.charger_state = CHARGER_NIMH_TRICKLE;
            }

            /* Rapid charge complete, dV/dt detection */
            if (cd.dvdt_count == 0) 
            {

                cd.charger_state = CHARGER_NIMH_TRICKLE;
				cd.dvdt_count = DxDT_BLANK_INTERVAL;
            }

            /* Exceed the maximum pack voltage */

            if (cd.adc_vbat > cs.vbat_ov) 
            {

                cd.status.voutovlo = 1; // Have to decide if you want to fault out because the battery got to max voltage, or if you want to just drop to trickle charging.
            }
            break;
            //</editor-fold>
        case CHARGER_NIMH_TRICKLE:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_NIMH_TRICKLE">
            /* Trickle charge until the charge timer expires */
        #ifdef NIMH_TRICKLE_ENABLE            


            LED_OnTime = LED_750;
            LED_OffTime = LED_250;
            if (cd.adc_iout == 0) 
            {
                cd.charger_state = CHARGER_OFF;
                cd.status.complete = 1;
            }

            if ((cd.adc_iout - cs.ibat_ct) > 200) 
            {
                dec_iout(5);
            } 
            else if ((cd.adc_iout - cs.ibat_ct) > 0) 
            {
                dec_iout(1);
            }
            break;
            //</editor-fold>
        #endif
        #endif
            
    /* Valve-Regulated Fast Charge IUU Profile */
        #ifdef ENABLE_VRLA_FAST
        case CHARGER_VRLAF_START:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_VRLAF_START">
            cd.charger_state = CHARGER_VRLAF_RESTORE;
            break;
            // </editor-fold>
        case CHARGER_VRLAF_RESTORE: //Precondition
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_VRLAF_RESTORE">
            LED_OnTime = LED_250;
            LED_OffTime = LED_750;
            //<editor-fold defaultstate="collapsed" desc="Ramp Input current until it reaches Precharge Current. If Vbat is high enough go to Rapid Charge.">

            //If the battery is already higher than the precondition threshold, advance it to the next state
            if (cd.adc_vbat > cs.vbat_pc)  
            {
                cd.charger_state = CHARGER_VRLAF_RAPID;
                cd.dvdt_count = DxDT_BLANK_INTERVAL;
                cd.Pdvdt_count = DxDT_BLANK_INTERVAL;
                cd.didt_count = DxDT_BLANK_INTERVAL;
            }
            //Now check to see if the voltage is climbing (with some hysteresis), but not drawing current.
            //This is the case in voltage mode outputs which should be used in the VRLAF profile when no battery is present.
            //Here we will decrement to hold the voltage steady until a battery is attached or a test case fails the precondition below.
            else if (((cd.adc_vbat+20) > cs.vbat_pc) && (cd.adc_iout <  (cs.ibat_pc - 0x40)))//%%BRYAN%% Add a #DEFINE for the hard coded values here
            {
                dec_iout(1);
            }
            //If none of the previous conditions were met, then we're in normal ramp-up operation. Increment.
            else if (cd.adc_iout < cs.ibat_pc)
            {
                inc_iout(1);
            }
            //The last case is for strange conditions that don't meet the above criteria.
            else
            {
                dec_iout(1);
            }
            if ((cd.dvdt_count == 0) || (cd.precondition_timer > cs.precondition_max_time)) 
            {
                cd.status.preconditionfail = 1;
                cd.charger_state = CHARGER_OFF;
            }
            //</editor-fold>
            break;
            // </editor-fold>
        case CHARGER_VRLAF_RAPID:   // Constant Current
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_VRLAF_RAPID">
            LED_OnTime = LED_500;
            LED_OffTime = LED_500;
            
            // <editor-fold defaultstate="collapsed" desc="Ramp and Regulate Current until Vbat reaches CV Termination, then go to Trickle State">
            if (cd.adc_vbat < cs.vbat_cv) 
            {
                if (cd.adc_iout < cs.ibat_cc)
                {
#ifdef FAST_RAMP_ENABLED                                                    
                    if (cd.adc_iout < (cs.ibat_cc - FAST_RAMP_ADC_COUNTS) )	// If enabled, fast ramp the current only if it is far too low
                    {
                        increment = FAST_RAMP_INCREMENT;
                    }
                    else
                    {
                        Increment = 1;
                    }
#else
                    increment = 1;
#endif
                    inc_iout(increment);
                }    
                else if (cd.adc_iout > (cs.ibat_cc + 40))  //Some negative feedback to reign in any overshoot from outside factors
                {
                    dec_iout(1);
                }
            } 
            else 
            {
                cd.charger_state = CHARGER_VRLAF_TRICKLE;
                cd.restore_timer = 0; 
                cd.dvdt_count = DxDT_BLANK_INTERVAL;
                cd.Pdvdt_count = DxDT_BLANK_INTERVAL;      
                for(i = 0; i < 4; i++)
                {
                    _adc_vbat[i] = cd.adc_vbat; 
                }
                VRLAFdvdt = 0; 
                VRLAF_eof_dvdt = 0; 
            }
            // </editor-fold>
            
            // <editor-fold defaultstate="collapsed" desc="For protection, if the Rapid Charge Timer ellapses, then go to Trickle State">
            /* Rapid charge timeout */
            if ((cs.rapid_time_max != 0) && (cd.rapid_charge_timer > cs.rapid_time_max))            
            {
                cd.charger_state = CHARGER_VRLAF_TRICKLE;
                cd.restore_timer = 0;
                cd.dvdt_count = DxDT_BLANK_INTERVAL;
                cd.Pdvdt_count = DxDT_BLANK_INTERVAL;
                for(i = 0; i < 4; i++)
                {
                    _adc_vbat[i] = cd.adc_vbat; 
                }
                VRLAFdvdt = 0; 
                VRLAF_eof_dvdt = 0; 
                cs.vbat_cv = cd.adc_vbat;  //We are advancing to the constant voltage stage.  To avoid a potentially large voltage step. Sample and regulate current voltage.
            }
            //</editor-fold>

            // <editor-fold defaultstate="collapsed" desc="Check for Cell Polarization or Thermal Runaway, i.e. Voltage going negative, then stop Charger">
            /* Charge complete, negative dV/dt detection */
            //Indicator of thermal runaway. Either due to the wrong CV setting and/or temperature compensation of the CV setting.
            //It's also a little less common to skip to the Float stage here instead.
            if (cd.dvdt_count == 0) 
            {
                cd.status.negdvdt = 1;
                cd.charger_state = CHARGER_OFF;
                cd.status.complete = 1;
            }
            // </editor-fold>
            
            //<editor-fold defaultstate="collapsed" desc="Check for Plateuing of the voltage before reaching CV that's common in old batteries. Then go to Trickle Charge.">
            /* 1st Charge complete, positive dV/dt detection */
            //We're looking here for an indicator of an aging battery that can't hit the CV or a set point not correctly temperature corrected.  It'll start to plateau early.  
            if (cd.Pdvdt_count == 0) 
            {
                //cd.status.negdvdt = 1;
                cd.Pdvdt_count = DxDT_BLANK_INTERVAL;
                VRLAFdvdt = 1;  //sets a flag used in charger_protection() to start looking for a decrease in the Pdvdt slope and sets VRLAF_eof_dvdt
            }
            
            /* Fast Charge complete, end of dV/dt detection */
            //A decreasing slope was detected which is an indicator of early termination voltage on an aging battery.
            if (VRLAF_eof_dvdt) 
            {
                cd.charger_state = CHARGER_VRLAF_TRICKLE;
                for(i = 0; i < 4; i++)
                {
                    _adc_vbat[i] = cd.adc_vbat; 
                    _adc_iout[i] = cd.adc_iout;
                }            
                cd.dvdt_count = DxDT_BLANK_INTERVAL;
                cd.Pdvdt_count = DxDT_BLANK_INTERVAL;
                cd.didt_count = DxDT_BLANK_INTERVAL;
                VRLAFdvdt = 0;
                VRLAF_eof_dvdt = 0;
                VRLAFdidt = 0;
                VRLAF_eof_didt = 0;
                cs.vbat_cv = cd.adc_vbat; //We are advancing to the constant voltage stage.  To avoid a potentially large voltage step. Sample and regulate current voltage.
            }
            //</editor-fold>
            
            break;
            //</editor-fold>
        case CHARGER_VRLAF_TRICKLE: // Constant Voltage
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_VRLAF_TRICKLE">                                        
            LED_OnTime = LED_750;
            LED_OffTime = LED_250;
            
            // <editor-fold defaultstate="collapsed" desc="Regulate a Constant Voltage">
            if (cd.adc_vbat > cs.vbat_cv)
            {
                if (cd.adc_iout > cs.ibat_ct) 
                {
                    dec_iout(1);
                }
            }
            else if (cd.adc_vbat + 1 < cs.vbat_cv) //A little feedback to keep the output voltage at the knee.
            {
                inc_iout(1);
            }
            // </editor-fold>
            
            //<editor-fold defaultstate="collapsed" desc="When the charge current reaches trickle current threshold go to Float State">
            if (cd.adc_iout < cs.ibat_ct) 
            {
                cd.charger_state = CHARGER_VRLAF_FLOAT;
            }
            //</editor-fold>
            
            //<editor-fold defaultstate="collapsed" desc="Check for current leveling before reaching the trickle current threshold. Then go to Float State.">
            // It's possible for older batteries to have enough leakage that they will never taper off to a charge current below the 
            // trickle current threshold cs.ibat_ct.  The code below will look for successive negative slope (i.e. cd.didt_count decrementing to 0)
            // before setting the VRLAFdidt bit.  This bit will execute a comparison in the charger_protection() routine that will look for the 
            // negative slope to flatten out.  You'll see this by the cd.didt value converging on zero. 
            if (cd.didt_count == 0)
            {
                cd.didt_count = DxDT_BLANK_INTERVAL;
                VRLAFdidt = 1;
            }
            
            if (VRLAF_eof_didt)
            {
                cd.charger_state = CHARGER_VRLAF_FLOAT;
                cd.didt_count = DxDT_BLANK_INTERVAL;
                VRLAFdidt = 0;
                VRLAF_eof_didt = 0;
            }
            //</editor-fold>
            
            //<editor-fold defaultstate="collapsed" desc="Check to see if the Restore Time has ellapsed then go to Float State.">
            /* Rapid charge timeout */
            if (cd.restore_timer > cs.restore_time_max) 
            {
                cd.charger_state = CHARGER_VRLAF_FLOAT;
            }
            // </editor-fold>
            
            break;

            // </editor-fold>           
        case CHARGER_VRLAF_FLOAT: 
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_VRLAF_FLOAT">

            LED_OnTime = LED_850;
            LED_OffTime = LED_150;
            
            //<editor-fold defaultstate="collapsed" desc="Regulate a constant voltage until the timer expires.">
            if (cd.adc_iout < cs.ibat_fc) //Check to make sure the float current isn't going too high.  
            {
                if (cd.adc_vbat > cs.vbat_fv) 
                {

                    x = dec_iout(1);
                }
                else if ((cd.adc_vbat + 1) < cs.vbat_fv) // Add a little negative feedback to hold it at the float voltage
                {
                    inc_iout(1);
                }
            }
            else
            {
                x = dec_iout(1);
            }
            /*if(!x)			// If we can't drop the current any lower, there is a problem. Shutdown.  %%BRYAN%% need to log a reason here
            {
                cd.charger_state = CHARGER_OFF;
                cd.status.complete = 1;
                cd.status.didtshdn = 1; 
                NOP();
            }*/
            // </editor-fold>

            //There's a total charge time check in the main while loop against the maximum charge time
            
            break;
            //</editor-fold>            
        #endif
            
    /* Valve-Regulated Lead Acid (Sealed Lead Acid).*/
        #ifdef ENABLE_VRLA_CC
        case CHARGER_VRLA_START:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_VRLA_START">
            cd.charger_state = CHARGER_VRLA_PRECONDITION;
            break;
            //</editor-fold>
        case CHARGER_VRLA_PRECONDITION:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_VRLA_PRECONDITION">
            if (cd.adc_vbat > cs.vbat_pc)  
            {
                cd.charger_state = CHARGER_VRLA_CC;
                cd.dvdt_count = DxDT_BLANK_INTERVAL;
                cd.Pdvdt_count = DxDT_BLANK_INTERVAL;
                cd.didt_count = DxDT_BLANK_INTERVAL;
            }
            else if (cd.adc_iout < cs.ibat_pc) 
            {
                inc_iout(1);
            }             
            if ((cd.dvdt_count == 0) || (cd.precondition_timer > cs.precondition_max_time)) 
            {
                cd.status.preconditionfail = 1;
                cd.charger_state = CHARGER_OFF;
            }
            break;
            //</editor-fold>
        case CHARGER_VRLA_CC:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_VRLA_CC">
            LED_OnTime = LED_500;
            LED_OffTime = LED_500;
            if (cd.adc_vbat < cs.vbat_cv) 
            {
                if (cd.adc_iout < cs.ibat_cc)
                {
#ifdef FAST_RAMP_ENABLED                                                    
                    if (cd.adc_iout < (cs.ibat_cc - FAST_RAMP_ADC_COUNTS) )	// If enabled, fast ramp the current only if it is far too low
                    {
                        increment = FAST_RAMP_INCREMENT;
                    }
                    else
                    {
                        Increment = 1;
                    }
#else
                    increment = 1;
#endif
                    inc_iout(increment);
                }
            } 
            else 
            {
                cd.charger_state = CHARGER_VRLA_CV;
            }

            /* Charge complete, negative dV/dt detection */
            if (cd.dvdt_count == 0) 
            {
                cd.status.negdvdt = 1;
                cd.charger_state = CHARGER_OFF;
            }

            /* 1st Charge complete, positive dV/dt detection */
            if (cd.Pdvdt_count == 0) 
            {
                //cd.status.negdvdt = 1;
                cd.Pdvdt_count = DxDT_BLANK_INTERVAL;
                VRLAdvdt = 1;
            }

            /* Fast Charge complete, end of dV/dt detection */
            if (VRLA_eof_dvdt) 
            {
                //cd.status.negdvdt = 0;
                cd.charger_state = CHARGER_VRLA_CV;
                VRLAdvdt = 0;
                VRLA_eof_dvdt = 0;
                _adc_vbat[0] = cd.adc_vbat;
                _adc_vbat[1] = cd.adc_vbat;
                _adc_vbat[2] = cd.adc_vbat;
                _adc_vbat[3] = cd.adc_vbat;
                cd.dvdt_count = DxDT_BLANK_INTERVAL;
                cd.Pdvdt_count = DxDT_BLANK_INTERVAL;
            }
            
            if ((cs.rapid_time_max != 0) && (cd.rapid_charge_timer >= cs.rapid_time_max))
            {
                cd.charger_state = CHARGER_VRLA_CV;
            }
            break;
            // </editor-fold>
        case CHARGER_VRLA_CV:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_VRLA_CV">
            LED_OnTime = LED_750;
            LED_OffTime = LED_250;
            if (cd.adc_vbat > cs.vbat_cv)
            {
                x = dec_iout(1);
                if(!x)			// If we can't drop the current any lower, there is a problem. Shutdown
                {
                    cd.charger_state = CHARGER_OFF;
                    cd.status.complete = 1;
                    cd.status.didtshdn = 1;
                }
            }
            else
            {
                inc_iout(1);					// Use this to keep the output current at the knee of CV mode.
            }

            if (cd.adc_iout < cs.ibat_ct) 
            {
                cd.charger_state = CHARGER_OFF;
                cd.status.complete = 1;
            }
            break;
            // </editor-fold>     
        #endif

    /* Super Capacitor */
        #ifdef ENABLE_SUPERCAP
        case CHARGER_SUPERCAP_START:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_SUPERCAP_START">
            cd.charger_state = CHARGER_SUPERCAP_CC;
            break;
            //</editor-fold>
        case CHARGER_SUPERCAP_CC:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_SUPERCAP_CC">
            LED_OnTime = LED_500;
            LED_OffTime = LED_500;
            if (cd.adc_vbat < cs.vbat_cv) 
            {
                if (cd.adc_iout < cs.ibat_cc)
                {
#ifdef FAST_RAMP_ENABLED                                                    
                    if (cd.adc_iout < (cs.ibat_cc - FAST_RAMP_ADC_COUNTS) )	// If enabled, fast ramp the current only if it is far too low
                    {
                        increment = FAST_RAMP_INCREMENT;
                    }
                    else
                    {
                        Increment = 1;
                    }
#else
                    increment = 1;
#endif
                    inc_iout(increment);
                }
                else if (cd.adc_iout > (cs.ibat_cc + 40)) //%%BRYAN%% This is a little debug attempt at reining in overshoot
                {                                         //          when manually controlling the maximum duty cycle or OVREFOUT.
                    dec_iout(1);
                }
            } 
            else 
            {
                cd.charger_state = CHARGER_SUPERCAP_CV;
            }
            if ((cs.rapid_time_max != 0) && (cd.rapid_charge_timer >= cs.rapid_time_max))
            {
                cd.charger_state = CHARGER_SUPERCAP_CV;
            }
            break;
            // </editor-fold>
        case CHARGER_SUPERCAP_CV:
            // <editor-fold defaultstate="collapsed" desc="Case: CHARGER_SUPERCAP_CV">
            LED_OnTime = LED_750;
            LED_OffTime = LED_250 ;
            if (cd.adc_vbat > cs.vbat_cv) 
            {
                x = dec_iout(1);	// Decrement the current
                if(!x)			// If we can't drop the current any lower, there is a problem. Shutdown
                {
                    cd.charger_state = CHARGER_OFF;
                    cd.status.complete = 1;
                }
            }

            if (cd.adc_iout < cs.ibat_ct) 
            {
                cd.charger_state = CHARGER_OFF;
                cd.status.complete = 1;
            }
            break;
            // </editor-fold>
        #endif
            
    /* Unknown Charger State - Communicate not properly configured */
        default:
            // <editor-fold defaultstate="collapsed" desc="Case: Default">
            cd.charger_state = CHARGER_OFF;
            cd.status.configerror = 1;		// Tell the user that a mis-configuration happened
            //cd.status.complete = 1;
            break;
            //</editor-fold>
	}
}





