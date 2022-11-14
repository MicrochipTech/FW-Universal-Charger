#ifndef MULTICHEMCHARGER_MAIN_H
#define	MULTICHEMCHARGER_MAIN_H
#include <stdint.h>
#ifdef	__cplusplus
extern "C" {
#endif

#define FWREV (0x0467)          
//volatile const int Charger[256] @0xe60 = 0x3fff; /* %%BRYAN%% */


/***************************** SYSTEM CONFIGURATION **********************************
 *
 * This header file allows you to configure the base-code to implement
 * the exact feature-set you need.  These include the following features:
 *		Allowed Chemistry Types:
 *			Lithium Ion (Li-Ion)
 *			Lithium Iron Phosphate (LiFePO4 or LFP)
 *			Nickle Metal Hydride (NIMH)
 *			Nickle Cadnium (NICd
 *			Valve-Regulated Lead Acid (Sealed Lead Acid)Standard and Fast-charge available
 *
 * 		User Interface Options:
 *			Status LEDs (Charge Enabled, CC/CV mode, Charge Complete, Charge Fault)
 *			Analog Debug Mode (Enables probing of analog voltage inside the MCP19111)
 *
 *		GUI Enable:
 *			Enables the use of the PC Based GUI routines (takes program space to implement)
 *
 *		GUI Configuration:
 *			Switches between GUI parameter configuration and embedded .h file with saved
 *			configuration/calibration data.
 *
 *		COMMs selection:
 *			Selects the type of serial port for the GUI and other peripherals
 *
 *		Fast Ramp:
 *			Allow hardware to quickly ramp output current to setpoint for large current
 *
 */

/* Enable GUI Configuration*/
#define ENABLE_GUI_CONFIG   // **BRYAN** Okay I have commented out below the ifndef making the GUI 
                            // and Fixed Config's mutually exclusive.
    
							// Turning this #define on allows the GUI to write the configuration
							// and calibration registers of the target device.  The opposite of
//#ifndef ENABLE_GUI_CONFIG	// this is to hard code the configuration and calibration using
#define ENABLE_FIXED_CONFIG 	// the MultiChemCharger_Config.H' file.  ENABLE_FIXED_CONFIG
//#endif						// tells the 'LoadConfiguration() routine to use this external
							// file to get the data. Data is generated by the GUI using the
							// 'Write .H Configuration File' button on the 'Advanced' tab.
							// Place the resulting file in the project folder. It will
							// automatically be #included in this project.  If you still plan on 
                            // using the GUI to tinker with configurations while also using the 
                            // header file ability, then comment out the #ifdef in the 
                            // MultiChemcharger_Hardware.c file that enables the flash write routine.


/* Enable i2c routines */
#define ENABLE_SMBUS 	// Enable for GUI or any other external I2C device


/* Enabled Battery Chemistry Types
 *
 * Un-Comment each of the following chemistry types you want to support in your application.
 * Only these will be compiled into your code which frees up space.
 * All can be simultaneously enabled if memory is available to support it.
 *		Note that the GUI is requried to configure which type is implemented.  The final
 *		product must have a method to configure the unit for a specific chemistry type.
 *
*/
#define ENABLE_LIION 		// Enable Lithium Ion
//#define ENABLE_LIFEPO4 		// Enable Lithium Iron Phosphate
//#define ENABLE_NIMH 			// Enable Nickle Metal Hydride (NIMH) 
//#define NIMH_TRICKLE_ENABLE	// Enable trickle charge mode for NiMH.  Not often recommended except for very large cells.
//#define ENABLE_VRLA_CC	    // Sealed Lead Acid Standard Charge-Rate - this isn't a good profile Not Functional ATM, use VRLA_FAST for all lead acid.
//#define ENABLE_VRLA_FAST 	// Sealed Lead Acid Fast Charge-Rate
//#define ENABLE_SUPERCAP



/* USER INTERFACE LED OPTIONS */
/*	Turn on one of the following defines depending on the type of user interface you desire.
		ENABLE_STATUS_LEDS option:
			* LED D1 illuminated (port GPB2) indicates charging is active
			* LED D2 illuminated (port GPA0) indicates CV mode (battery roughly at 75-100% charged)
			* LED D3 illuminated (port GPA2) indicates charge complete
			* LED D4 illuminated (port GPA1) indicates charge fault mode

		ENABLE_BENCH_TEST_OUTOUT_PIN (MCP1911x designs only)
			* Enables the bench-test output pin and associated internal 'probing' of the IC
			* Utilizes port GPA0 (Diode D2 connection).  Best practice is to remove either D2 or R29 when using this mode.*/


// Un-comment only 1 of the following #defines to select the LED or bench-test modes
#define ENABLE_STATUS_LEDS 				// Enable status LEDs
//#define ENABLE_BENCH_TEST_OUTPUT_PIN	// This is only for MCP19111 based hardware %%BRYAN%% adjust this comment to be more agnostic
//#define FAST_RAMP_ENABLED 			// Enable this define if you want a fast ramp-up to the desired current. (!!CC mode only)
//#define FAST_RAMP_ADC_COUNTS (50)	// Number of ADC counts below ideal that triggers the fast-step routine
//#define FAST_RAMP_INCREMENT (3)		// Number of duty-cycle increments added baseline when in fast_ramp mode
//#define SOFT_START_ENABLED
//#define ENABLE_TEMP_SENSOR    		// Use temp sensor to limit charging when ambient is too cold or hot
//#define ENABLE_TEMP_SENSOR_VOUT_ADJUSTMENT // Adjust Vout based on temperature.  Common with VRLA batteries to adjust termination and float.
    
/*************************** Function prototypes **************************/
void init_variables(void);			// Initialize all variables to default values
void adc_capture(void);				// Routine to sample and average all ADC values
void charger_protection(void);		// Routines that check for reasons to shutdown the charger (Timeout, errors, etc)
void charger_state_machine(void);	// Charger State Machine
void update_user_interface(void);	// User Interface updates

// Hardware Specific Routines //

void get_adc_result(void);	// Get ADC result
void load_saved_variables(void); // Retrieve variables from Flash

/*********************** System Parameter Definitions *****************************
 *
 * The following are #defines for various parameters related to the charging function
 *
 */

/* Chemistry Types */
#define CHEMISTRY_LITHIUM (0)	// Lithium Ion
#define CHEMISTRY_NIMH (1)		// Nickle Metal Hydride
#define CHEMISTRY_VRLA_CC (2)	// Valve Regulated Lead-Acid Battery
#define CHEMISTRY_VRLA_FAST (3)	// Valve Regulated Lead-Acid Battery
#define CHEMISTRY_LIFEPO4 (4)	// Lithium Iron Phosphate
#define CHEMISTRY_SUPERCAP (5)  // Super Capacitor Bank

/* 1 minute timer for dV/dt and dT/dt detection */

#define DxDT_BLANK_INTERVAL (2)
#define ENABLE_TEMP_DTDT_SHUTDOWN

/* LED Timer Information */

#define	LED_150		4
#define	LED_250		7
#define	LED_500		15
#define	LED_750		21
#define	LED_850		24

  /* board status LEDs */
unsigned char LED_TMR, LED_OnTime, LED_OffTime;
volatile bit bLED_On;

/* For writing to the FLASH memory */
unsigned char flash_write_buf[8];  

#define CAL_BASE_ADDR (0xe60)  //This is the address in flash where the calibration (16 words) for the GUI are stored. Changed to 0xe60 from 0xe80 to make more room
                                //**IMPORTANT** if you change this address, then go change it in the MultiChemCharger_Values.h file
                                //and check the GUI's subroutines for generating the header file and change the address there as well.
#define SETTINGS_OFFSET (0x20) 
#define SETTINGS_ADDR (CAL_BASE_ADDR + SETTINGS_OFFSET) //This is the address in flash where the configuration (64 words) for the GUI are stored.

/******************************** System Structures *************************/


// Settings that control the charger, loaded from FLASH
// For production, these values can be hard-coded here and the GUI disabled
// The GUI reserves room for 128 bytes of data from this structure.

struct charger_settings_t {   
    unsigned short  uvlo_threshold_off;     // Input UV threshold for turn off
    unsigned short  uvlo_threshold_on;      // Input UV threshold to allow turn-on    
    unsigned short  vbat_ov;                // Output overvoltage shutdown threshold
    unsigned short  vbat_pc;                // Battery voltage PRE to CC   
    unsigned short  vbat_cv;                // Battery voltage CC to CV    
    unsigned short  ibat_pc;                // Preconditioning battery current
    unsigned short  ibat_cc;                // Charge current
    unsigned short  ibat_ct;                // Charge termination current        
    unsigned short  ovcfcon_min;            // Minimum setting for output DAC
    unsigned short  ovcfcon_max;            // Maximum setting for output DAC
    unsigned short  uvlo_adc_threshold_off; // Input UV threshold for ADC
    unsigned short  uvlo_adc_threshold_on;  // Input UV threshold for ADC
    unsigned short  restore_time_max;       // RESTORE charge maximum time
    unsigned short  dvdt_blank_time;        // dV/dt blank time
    unsigned short  rapid_time_max;         // RAPID charge maximum time
    short           dtdt_slope;             // Slope for dT/dt detection    
    unsigned short  pack_temp_min;          // Minimum pack temperature
    unsigned short  pack_temp_max;          // Maximum pack temperature   
    unsigned short  chemistry;              // Battery chemistry (LiIon/NiMH)    
    unsigned short  OVREFCON_OV;            // new for MCP19125 Reference for OV Comparator when enabled
    unsigned short  OVREFCON_CV;            // new for MCP19125 Reference for EA1 when enabled
    unsigned short  VREFCON_MAX;            // new for MCP19125 Maximum value for VREFCON, basically maximum output current    
    unsigned short  vbat_fv;                // Battery voltage for float stage for VRLA
    short           neg_dvdt;               // Threshold for a negative dV/dt 
    short           pos_dvdt_cc;            // Threshold for a positive dV/dt during constant current charge
    short           neg_didt_cv;            // Threshold for a negative dI/dt during constant voltage charge
    unsigned short  pack_temp_25C;
    short           temp_1C;
    unsigned short  temp_sense_en;    
    unsigned short  ibat_fc;                // Maximum float current    
    unsigned short  precondition_max_time;  // Maximum precondition time    
    short           pc_dvdt;                // Threshold for minimum dV/dt rise during precondition
    unsigned short  vbat_1c_adjust;
    unsigned short  charge_time_max;        // Maximum charge time before shut-off
    //Above are structure values that must be in that specific order for the load_saved_variables routine, Below not so much
    unsigned short  endian;                 // Endianness of the compiler
    unsigned short  fwrev;                  // Firmware revision
    unsigned short  vbat_pc_25C;
    unsigned short  vbat_cv_25C;
    unsigned short  vbat_fv_25C;
};


/* End of Charge (EOC) status of the charger that is included in the charger_data_t structure as byte 25 */
typedef union {
    //unsigned char word;
    unsigned short word;
    
    struct {
        unsigned vinuvlo            : 1;
        unsigned vinovlo            : 1;
        unsigned voutovlo           : 1;
        unsigned overtemp           : 1;
        unsigned undertemp          : 1;
        unsigned serialshdn         : 1;               
        unsigned buttonshdn         : 1;                 
        unsigned negdvdt            : 1;                
        unsigned dtdtshdn           : 1;
        unsigned timeout            : 1;
        unsigned preconditionfail   : 1;
        unsigned configerror        : 1;
        unsigned didtshdn           : 1;
        unsigned unused2            : 1;
        unsigned nobattery          : 1;
        unsigned complete           : 1;
    };
} charger_status_t;


/* Variables that control the charger */
/* In the GUI these are stored in the Profile_Data[32]*/
struct charger_data_t
{
  /* These are measured */
  unsigned short    adc_vin;            // [0-1] Input voltage
  unsigned short    adc_vbat;           // [2-3] Output (battery) voltage
  unsigned short    adc_iout;           // [4-5] Output (battery) current
  unsigned short    adc_vref;           // [6-7] Internal current vref
  unsigned short    adc_temp;           // [8-9] Temperature

  /* State information */
  unsigned short    charge_timer;       // [10-11] Charge timer (time to shut-off)
  unsigned char     charger_state;      // [12] State of the battery charger
  unsigned char     adc_reading;        // [13] ADC channel reading
  unsigned short    restore_timer; 		// [14-15] Voltage restore timer (CV)
  unsigned short    rapid_charge_timer;	// [16-17] Rapid charge timer (CC)
  short             dvdt;				// [18-19] dV/dt detection
  short             dtdt;				// [20-21] dT/dt detection
  unsigned char     dvdt_count;			// [22] Counter for negative dV/dt
  unsigned char     dtdt_count;			// [23] Counter for negative dT/dt
  charger_status_t  status;				// [24-25] Live status of charger
  unsigned char     Pdvdt_count;		// [26] Counter for positive dV/dt 
  unsigned char     ref_design_PCB;		// [27] Reference design number
  unsigned char     didt_count;         // [28] Counter for negative dI/dt
  unsigned short    precondition_timer; // [29-30] Precondition Timer (PC)
  short             didt;               // [31-32] dI/dt detection
}; 

enum adc_reading_t
{
    ADC_VIN = 0,
    ADC_VBAT,
    ADC_IOUT,
    ADC_VREF,
    ADC_TEMP,
} adc_reading;

enum charger_state_t {		// Valid Charger State #'s
    CHARGER_OFF = 0,
    CHARGER_LIION_START = 1,
    CHARGER_LIION_PRECONDITION = 2,
    CHARGER_LIION_CC = 3,
    CHARGER_LIION_CV = 4,
    CHARGER_NIMH_START = 5,
    CHARGER_NIMH_RESTORE = 6,
    CHARGER_NIMH_RAPID = 7,
    CHARGER_NIMH_TRICKLE = 8,
    CHARGER_VRLA_START = 9,
    CHARGER_VRLA_PRECONDITION = 10,
    CHARGER_VRLA_CC = 11,
    CHARGER_VRLA_CV = 12,
    CHARGER_VRLAF_START = 13,
    CHARGER_VRLAF_RESTORE = 14,
    CHARGER_VRLAF_RAPID = 15,
    CHARGER_VRLAF_TRICKLE = 16,
    CHARGER_VRLAF_FLOAT = 17,
    CHARGER_LIFEPO4_START = 18,
    CHARGER_LIFEPO4_PRECONDITION = 19,
    CHARGER_LIFEPO4_CC = 20,
    CHARGER_LIFEPO4_CV = 21,
    CHARGER_SUPERCAP_START = 22,
    CHARGER_SUPERCAP_CC = 23,
    CHARGER_SUPERCAP_CV = 24
};


/* SMBUS variables */
unsigned int smbusbuf;
unsigned int smbusaddr;
unsigned int flashaddr;
unsigned int flashwritecounter;
unsigned char smbusaddr_next;
unsigned int registeraddr;
bit flash_access;
bit sfr_access;
bit struct_access;
bit temperature_access;



#ifdef	__cplusplus
}
#endif

#endif	/* MULTICHEMCHARGER_MAIN_H */

