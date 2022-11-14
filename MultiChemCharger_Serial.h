

#ifndef MULTICHEMCHARGER_SERIAL_H
#define	MULTICHEMCHARGER_SERIAL_H

#ifdef	__cplusplus
extern "C" {
#endif

// Function Declarations
void init_serial_io(void);
void service_SMBUS();				// Service the SMBUS routine


/* Charger Commands from the GUI */
#define CMD_CHARGER_OFF (0x0)		// Charge_state state machine definitions
#define CMD_CHARGER_ON (0x1)
#define CMD_CHARGER_READCONFIG (0x2)
#define CMD_CHARGER_DISABLE (0x3)  /* Disable charger is a command that isn't really defined             */
                                 /* for use in the firmware.  At the moment sending this command       */
                                 /* will initiate a shutdown as if the button were pushed. That's all. */
#define CMD_FUEL_GAUGE_MODE (0x7)		// LED user interface configuration commands 
#define CMD_STATUS_LED_MODE (0x8)
#define CMD_ANALOG_DEBUG_MODE (0x9)

/* LED Interface Modes */
#define FUEL_GAUGE_MODE (1)
#define STATUS_LED_MODE (2)
#define ANALOG_MODE (3)

#ifdef	__cplusplus
}
#endif

#endif	/* MULTICHEMCHARGER_SERIAL_H */

