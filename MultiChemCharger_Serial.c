#include <xc.h>
#include <pic.h>
#include "MultiChemCharger_Main.h"
#include "MultiChemCharger_Hardware.h"
#include "MultiChemCharger_Serial.h"


void init_serial_io(void)  /* Initialize SMBUS */
{
    /* SMBUS variables */
    smbusaddr = 0;
    registeraddr = 0;
    flashaddr = 0;
    smbusaddr_next = 0;
    flash_access = 0;
    struct_access = 0;

#ifdef MCP19111_CHARGER_TOPOLOGY
	SSPADD = 0x20;// MSSP Address and baud Rate Register 1, Address is 0x20
    // <editor-fold defaultstate="collapsed" desc="SSPADD Register Bit Description">
                                        // 7-bit Slave Mode:
                                        // bit 7-1 7 bit address
                                        // bit 0 Unused
    // </editor-fold>
    SSPADD2 = 0x20;// MSSP Address 2 , Address is 0x20, but is disabled for basic I2C
    // <editor-fold defaultstate="collapsed" desc="SSPADD2 Register Bit Description">
                                        // 7-bit Slave Mode:
                                        // bit 7-1 7 bit address
                                        // bit 0, 1 = Enable address matching with SSPADD2, 0 =Disable
    // </editor-fold>
    SSPCON1 = 0x26;// SSP Control Register 1  //Here's a tip.  Setting 0x26 locks out GPA4 for any external function.
    // <editor-fold defaultstate="collapsed" desc="SSPCON1 Register Bit Description">
                                        // bit 7 - WCOL, Write Collision Detect bit; 0 = No Collision, 1 = Write to SSPBUF attempted during bad conditions
                                        // bit 6 - SSPOV, Receive Overflow Indicator Bit, 0 = No Overflow, 1 = Byte received in SSPBUF while still holding previous byte
                                        // bit 5 - SSPEN, Synchronous Serial Port Enable bit, 0 = GPIO, 1 = Serial Port
                                        // bit 4 - CKP, clock polarity select bit, Slave Mode: 1 = Enable clock, 0 = Stretch Low; Master Mode: Unused
                                        // bit 3-0 - SSPM<3:0> synchronous serial port mode select bits
                                        // 0110 = I2C Slave Mode, 7-bit
                                        // 0111 = I2C Slave Mode, 10-bit
                                        // 1000 = I2C Master Mode, Clock = Fosc/(4XSSPADD+1))
                                        // 1011 = I2C firmware controlled master mode (slave idle)
                                        // 1110 = I2C Slave mode, 7-bit address with start and stop bit interrupts enabled
                                        // 1111 = I2C Slave mode, 10-bit address with start and stop bit interrupts enabled
    // </editor-fold>
    SSPCON2 = 0x00;// SSP Control Register 2
    // <editor-fold defaultstate="collapsed" desc="SSPCON2 Register Bit Description">
        // bit 7 GCEN General Call Enable bit (in I2C slave mode only)) 0=Disabled, 1=Enable interrupt on general call 0x00 received in SSPSR
        // bit 6 ACKSTAT Acknowledge Status bit, 1=Ack not received, 0=Ack received
        // bit 5 ACKDT Acknowledge Data bit (in receive mode) Value transmitted when the user initiates an Ack sequence at the end of a receive
        //      1 = Not Acknowledged, 0 = Acknowledged
        // bit 4 ACKEN Acknowledge Sequence Enable bit (in I2C Master mode only) 0=Ack Sequence Idle, 1= Initiate Ack sequence on SDA/SCL and transmit ACKDT, automatically cleared by hardware.
        // bit 3 RCEN Receive Enable bit (I2C Master mode only) 0=Receive Idle, 1=Enabled Receive mode for I2C
        // bit 2 PEN Stop Condition Enable bit (I2C Master mode only) SCK Release Control, 0=Repeated stop condition idle, 1=Initiate stop condition on SDA and SCL pins, automatically cleared by hardware
        // bit 1 RSEN Repeated Start condition Enabled bit (in I2C Master mode only) 0=Repeated start condition idle, 1=Initiate repeated start condition on SDA and SCL pins, automatically cleared by hardware
        // bit 0 SEN Start Condition Enabled bit (in I2C Master mode only)) Master: 0=Start condition Idle, 1=Initiate Start condition on SDA/SCL pins, cleared by hardware
        //      Slave: 1=Clock stretching is enabled for both slave transmit and slave receive (stretch enabled), 0=clock stretching disabled.
    // </editor-fold>
    SSPCON3 = 0x05;// SSP Control Register 3
    // <editor-fold defaultstate="collapsed" desc="SSPCON3 Register Bit Description">
        // bit 7 ACKTIM Acknowledge Time Status bit, 1=I2C bus is in an Ack sequence, set on the 8th falling edge of SCL clock
        //      0=Not an Ack sequence, cleared on the 9th rising edge of SCL clock
        // bit 6 PCIE Stop Condition Interrupt Enable bit, 1=Enable Intr on detection of Stop condition, 0=stop detection interrupts are disabled
        // bit 5 SCIE Start Condition Interrupt Enable bit, 1=Enable interrupt on detection of start or restart conditions, 0=start detection interrupts are disabled
        // bit 4 BOEN Buffer Overwrite Enable bit, Slave: 1=SSPBUF is updated and ACK# is generated for a received address/data byte, ignoring thes tate
        //      of the SSPOV bit only if the BF bit = 0, 0=SSPBUF is only updated when SSPOV is clear
        // bit 3 SDAHT SDA Hold Time Selection bit, 1=Minimum of 300ns hold time on SDA after the falling edge of SCL, 0 = minimum of 100ns
        // bit 2 SBCDE Slave Mode Bus Collision Detect Enable bit; 1=Enable slave bus collision interrupts, 0 = Slave bus collision interrupts are disabled.
        // bit 1 AHEN Address Hold Enable bit (I2C Slave Mode only) 0=Address holding is disabled, 1=Following the 8th falling edge of SCL for a matching
        //      received address byte; CKP bit in the SSPCON1 register will be cleared and the SCL will be held low.
        // bit 0 DHEN Data Hold Enable bit (I2C Salve mode only) 0=Data holding is disabled, 1=Following the 8th falling edge of SCL for a received data byte; 
        //      slave hardware clears the CKP bit in the SSPCON1 register and SCL is held low.
    // </editor-fold>
    SSPIF = 0;
#endif    
    
#ifdef MCP19118_FOUR_SWITCH_BUCK_BOOST_CHARGER
	SSPADD = 0x20;// MSSP Address and baud Rate Register 1, Address is 0x20
    // <editor-fold defaultstate="collapsed" desc="SSPADD Register Bit Description">
                                        // 7-bit Slave Mode:
                                        // bit 7-1 7 bit address
                                        // bit 0 Unused
    // </editor-fold>
    SSPADD2 = 0x20;// MSSP Address 2 , Address is 0x20, but is disabled for basic I2C
    // <editor-fold defaultstate="collapsed" desc="SSPADD2 Register Bit Description">
                                        // 7-bit Slave Mode:
                                        // bit 7-1 7 bit address
                                        // bit 0, 1 = Enable address matching with SSPADD2, 0 =Disable
    // </editor-fold>
    SSPCON1 = 0x26;// SSP Control Register 1  //Here's a tip.  Setting 0x26 locks out GPA4 for any external function.
    // <editor-fold defaultstate="collapsed" desc="SSPCON1 Register Bit Description">
                                        // bit 7 - WCOL, Write Collision Detect bit; 0 = No Collision, 1 = Write to SSPBUF attempted during bad conditions
                                        // bit 6 - SSPOV, Receive Overflow Indicator Bit, 0 = No Overflow, 1 = Byte received in SSPBUF while still holding previous byte
                                        // bit 5 - SSPEN, Synchronous Serial Port Enable bit, 0 = GPIO, 1 = Serial Port
                                        // bit 4 - CKP, clock polarity select bit, Slave Mode: 1 = Enable clock, 0 = Stretch Low; Master Mode: Unused
                                        // bit 3-0 - SSPM<3:0> synchronous serial port mode select bits
                                        // 0110 = I2C Slave Mode, 7-bit
                                        // 0111 = I2C Slave Mode, 10-bit
                                        // 1000 = I2C Master Mode, Clock = Fosc/(4XSSPADD+1))
                                        // 1011 = I2C firmware controlled master mode (slave idle)
                                        // 1110 = I2C Slave mode, 7-bit address with start and stop bit interrupts enabled
                                        // 1111 = I2C Slave mode, 10-bit address with start and stop bit interrupts enabled
    // </editor-fold>
    SSPCON2 = 0x00;// SSP Control Register 2
    // <editor-fold defaultstate="collapsed" desc="SSPCON2 Register Bit Description">
        // bit 7 GCEN General Call Enable bit (in I2C slave mode only)) 0=Disabled, 1=Enable interrupt on general call 0x00 received in SSPSR
        // bit 6 ACKSTAT Acknowledge Status bit, 1=Ack not received, 0=Ack received
        // bit 5 ACKDT Acknowledge Data bit (in receive mode) Value transmitted when the user initiates an Ack sequence at the end of a receive
        //      1 = Not Acknowledged, 0 = Acknowledged
        // bit 4 ACKEN Acknowledge Sequence Enable bit (in I2C Master mode only) 0=Ack Sequence Idle, 1= Initiate Ack sequence on SDA/SCL and transmit ACKDT, automatically cleared by hardware.
        // bit 3 RCEN Receive Enable bit (I2C Master mode only) 0=Receive Idle, 1=Enabled Receive mode for I2C
        // bit 2 PEN Stop Condition Enable bit (I2C Master mode only) SCK Release Control, 0=Repeated stop condition idle, 1=Initiate stop condition on SDA and SCL pins, automatically cleared by hardware
        // bit 1 RSEN Repeated Start condition Enabled bit (in I2C Master mode only) 0=Repeated start condition idle, 1=Initiate repeated start condition on SDA and SCL pins, automatically cleared by hardware
        // bit 0 SEN Start Condition Enabled bit (in I2C Master mode only)) Master: 0=Start condition Idle, 1=Initiate Start condition on SDA/SCL pins, cleared by hardware
        //      Slave: 1=Clock stretching is enabled for both slave transmit and slave receive (stretch enabled), 0=clock stretching disabled.
    // </editor-fold>
    SSPCON3 = 0x05;// SSP Control Register 3
    // <editor-fold defaultstate="collapsed" desc="SSPCON3 Register Bit Description">
        // bit 7 ACKTIM Acknowledge Time Status bit, 1=I2C bus is in an Ack sequence, set on the 8th falling edge of SCL clock
        //      0=Not an Ack sequence, cleared on the 9th rising edge of SCL clock
        // bit 6 PCIE Stop Condition Interrupt Enable bit, 1=Enable Intr on detection of Stop condition, 0=stop detection interrupts are disabled
        // bit 5 SCIE Start Condition Interrupt Enable bit, 1=Enable interrupt on detection of start or restart conditions, 0=start detection interrupts are disabled
        // bit 4 BOEN Buffer Overwrite Enable bit, Slave: 1=SSPBUF is updated and ACK# is generated for a received address/data byte, ignoring thes tate
        //      of the SSPOV bit only if the BF bit = 0, 0=SSPBUF is only updated when SSPOV is clear
        // bit 3 SDAHT SDA Hold Time Selection bit, 1=Minimum of 300ns hold time on SDA after the falling edge of SCL, 0 = minimum of 100ns
        // bit 2 SBCDE Slave Mode Bus Collision Detect Enable bit; 1=Enable slave bus collision interrupts, 0 = Slave bus collision interrupts are disabled.
        // bit 1 AHEN Address Hold Enable bit (I2C Slave Mode only) 0=Address holding is disabled, 1=Following the 8th falling edge of SCL for a matching
        //      received address byte; CKP bit in the SSPCON1 register will be cleared and the SCL will be held low.
        // bit 0 DHEN Data Hold Enable bit (I2C Salve mode only) 0=Data holding is disabled, 1=Following the 8th falling edge of SCL for a received data byte; 
        //      slave hardware clears the CKP bit in the SSPCON1 register and SCL is held low.
    // </editor-fold>
    SSPIF = 0;
#endif
    
#ifdef MCP19119_4A_POWER_TOOL_CHARGER
	SSPADD = 0x20;
    // <editor-fold defaultstate="collapsed" desc="SSPADD Register Bit Description">
                                        // 7-bit Slave Mode:
                                        // bit 7-1 7 bit address
                                        // bit 0 Unused
    // </editor-fold>
    SSPCON1 = 0x26;
    // <editor-fold defaultstate="collapsed" desc="SSPCON1 Register Bit Description">
                                        // bit 7 - WCOL, Write Collision Detect bit; 0 = No Collision, 1 = Write to SSPBUF attempted during bad conditions
                                        // bit 6 - SSPOV, Receive Overflow Indicator Bit, 0 = No Overflow, 1 = Byte received in SSPBUF while still holding previous byte
                                        // bit 5 - SSPEN, Synchronous Serial Port Enable bit, 0 = GPIO, 1 = Serial Port
                                        // bit 4 - CKP, clock polarity select bit, Slave Mode: 1 = Enable clock, 0 = Stretch Low; Master Mode: Unused
                                        // bit 3-0 - SSPM<3:0> synchronous serial port mode select bits
                                        // 0110 = I2C Slave Mode, 7-bit
                                        // 0111 = I2C Slave Mode, 10-bit
                                        // 1000 = I2C Master Mode, Clock = Fosc/(4XSSPADD+1))
                                        // 1011 = I2C firmware controlled master mode (slave idle)
                                        // 1110 = I2C Slave mode, 7-bit address with start and stop bit interrupts enabled
                                        // 1111 = I2C Slave mode, 10-bit address with start and stop bit interrupts enabled
    // </editor-fold>
    SSPCON2 = 0x00;
    // <editor-fold defaultstate="collapsed" desc="SSPCON2 Register Bit Description">
        // bit 7 GCEN General Call Enable bit (in I2C slave mode only)) 0=Disabled, 1=Enable interrupt on general call 0x00 received in SSPSR
        // bit 6 ACKSTAT Acknowledge Status bit, 1=Ack not received, 0=Ack received
        // bit 5 ACKDT Acknowledge Data bit (in receive mode) Value transmitted when the user initiates an Ack sequence at the end of a receive
        //      1 = Not Acknowledged, 0 = Acknowledged
        // bit 4 ACKEN Acknowledge Sequence Enable bit (in I2C Master mode only) 0=Ack Sequence Idle, 1= Initiate Ack sequence on SDA/SCL and transmit ACKDT, automatically cleared by hardware.
        // bit 3 RCEN Receive Enable bit (I2C Master mode only) 0=Receive Idle, 1=Enabled Receive mode for I2C
        // bit 2 PEN Stop Condition Enable bit (I2C Master mode only) SCK Release Control, 0=Repeated stop condition idle, 1=Initiate stop condition on SDA and SCL pins, automatically cleared by hardware
        // bit 1 RSEN Repeated Start condition Enabled bit (in I2C Master mode only) 0=Repeated start condition idle, 1=Initiate repeated start condition on SDA and SCL pins, automatically cleared by hardware
        // bit 0 SEN Start Condition Enabled bit (in I2C Master mode only)) Master: 0=Start condition Idle, 1=Initiate Start condition on SDA/SCL pins, cleared by hardware
        //      Slave: 1=Clock stretching is enabled for both slave transmit and slave receive (stretch enabled), 0=clock stretching disabled.
    // </editor-fold>
    SSPCON3 = 0x05;
    // <editor-fold defaultstate="collapsed" desc="SSPCON3 Register Bit Description">
        // bit 7 ACKTIM Acknowledge Time Status bit, 1=I2C bus is in an Ack sequence, set on the 8th falling edge of SCL clock
        //      0=Not an Ack sequence, cleared on the 9th rising edge of SCL clock
        // bit 6 PCIE Stop Condition Interrupt Enable bit, 1=Enable Intr on detection of Stop condition, 0=stop detection interrupts are disabled
        // bit 5 SCIE Start Condition Interrupt Enable bit, 1=Enable interrupt on detection of start or restart conditions, 0=start detection interrupts are disabled
        // bit 4 BOEN Buffer Overwrite Enable bit, Slave: 1=SSPBUF is updated and ACK# is generated for a received address/data byte, ignoring thes tate
        //      of the SSPOV bit only if the BF bit = 0, 0=SSPBUF is only updated when SSPOV is clear
        // bit 3 SDAHT SDA Hold Time Selection bit, 1=Minimum of 300ns hold time on SDA after the falling edge of SCL, 0 = minimum of 100ns
        // bit 2 SBCDE Slave Mode Bus Collision Detect Enable bit; 1=Enable slave bus collision interrupts, 0 = Slave bus collision interrupts are disabled.
        // bit 1 AHEN Address Hold Enable bit (I2C Slave Mode only) 0=Address holding is disabled, 1=Following the 8th falling edge of SCL for a matching
        //      received address byte; CKP bit in the SSPCON1 register will be cleared and the SCL will be held low.
        // bit 0 DHEN Data Hold Enable bit (I2C Salve mode only) 0=Data holding is disabled, 1=Following the 8th falling edge of SCL for a received data byte; 
        //      slave hardware clears the CKP bit in the SSPCON1 register and SCL is held low.
    // </editor-fold>
    SSPIF = 0;
#endif    

#ifdef MCP19123_BUCK_BOOST_CHARGER
	SSPADD = 0x20;// MSSP Address and baud Rate Register 1, Address is 0x20
    // <editor-fold defaultstate="collapsed" desc="SSPADD Register Bit Description">
                                        // 7-bit Slave Mode:
                                        // bit 7-1 7 bit address
                                        // bit 0 Unused
    // </editor-fold>
    SSPADD2 = 0x20;// MSSP Address 2 , Address is 0x20, but is disabled for basic I2C
    // <editor-fold defaultstate="collapsed" desc="SSPADD2 Register Bit Description">
                                        // 7-bit Slave Mode:
                                        // bit 7-1 7 bit address
                                        // bit 0, 1 = Enable address matching with SSPADD2, 0 =Disable
    // </editor-fold>
    SSPCON1 = 0x26;// SSP Control Register 1  //Here's a tip.  Setting 0x26 locks out GPA4 for any external function.
    // <editor-fold defaultstate="collapsed" desc="SSPCON1 Register Bit Description">
                                        // bit 7 - WCOL, Write Collision Detect bit; 0 = No Collision, 1 = Write to SSPBUF attempted during bad conditions
                                        // bit 6 - SSPOV, Receive Overflow Indicator Bit, 0 = No Overflow, 1 = Byte received in SSPBUF while still holding previous byte
                                        // bit 5 - SSPEN, Synchronous Serial Port Enable bit, 0 = GPIO, 1 = Serial Port
                                        // bit 4 - CKP, clock polarity select bit, Slave Mode: 1 = Enable clock, 0 = Stretch Low; Master Mode: Unused
                                        // bit 3-0 - SSPM<3:0> synchronous serial port mode select bits
                                        // 0110 = I2C Slave Mode, 7-bit
                                        // 0111 = I2C Slave Mode, 10-bit
                                        // 1000 = I2C Master Mode, Clock = Fosc/(4XSSPADD+1))
                                        // 1011 = I2C firmware controlled master mode (slave idle)
                                        // 1110 = I2C Slave mode, 7-bit address with start and stop bit interrupts enabled
                                        // 1111 = I2C Slave mode, 10-bit address with start and stop bit interrupts enabled
    // </editor-fold>
    SSPCON2 = 0x00;// SSP Control Register 2
    // <editor-fold defaultstate="collapsed" desc="SSPCON2 Register Bit Description">
        // bit 7 GCEN General Call Enable bit (in I2C slave mode only)) 0=Disabled, 1=Enable interrupt on general call 0x00 received in SSPSR
        // bit 6 ACKSTAT Acknowledge Status bit, 1=Ack not received, 0=Ack received
        // bit 5 ACKDT Acknowledge Data bit (in receive mode) Value transmitted when the user initiates an Ack sequence at the end of a receive
        //      1 = Not Acknowledged, 0 = Acknowledged
        // bit 4 ACKEN Acknowledge Sequence Enable bit (in I2C Master mode only) 0=Ack Sequence Idle, 1= Initiate Ack sequence on SDA/SCL and transmit ACKDT, automatically cleared by hardware.
        // bit 3 RCEN Receive Enable bit (I2C Master mode only) 0=Receive Idle, 1=Enabled Receive mode for I2C
        // bit 2 PEN Stop Condition Enable bit (I2C Master mode only) SCK Release Control, 0=Repeated stop condition idle, 1=Initiate stop condition on SDA and SCL pins, automatically cleared by hardware
        // bit 1 RSEN Repeated Start condition Enabled bit (in I2C Master mode only) 0=Repeated start condition idle, 1=Initiate repeated start condition on SDA and SCL pins, automatically cleared by hardware
        // bit 0 SEN Start Condition Enabled bit (in I2C Master mode only)) Master: 0=Start condition Idle, 1=Initiate Start condition on SDA/SCL pins, cleared by hardware
        //      Slave: 1=Clock stretching is enabled for both slave transmit and slave receive (stretch enabled), 0=clock stretching disabled.
    // </editor-fold>
    SSPCON3 = 0x05;// SSP Control Register 3
    // <editor-fold defaultstate="collapsed" desc="SSPCON3 Register Bit Description">
        // bit 7 ACKTIM Acknowledge Time Status bit, 1=I2C bus is in an Ack sequence, set on the 8th falling edge of SCL clock
        //      0=Not an Ack sequence, cleared on the 9th rising edge of SCL clock
        // bit 6 PCIE Stop Condition Interrupt Enable bit, 1=Enable Intr on detection of Stop condition, 0=stop detection interrupts are disabled
        // bit 5 SCIE Start Condition Interrupt Enable bit, 1=Enable interrupt on detection of start or restart conditions, 0=start detection interrupts are disabled
        // bit 4 BOEN Buffer Overwrite Enable bit, Slave: 1=SSPBUF is updated and ACK# is generated for a received address/data byte, ignoring thes tate
        //      of the SSPOV bit only if the BF bit = 0, 0=SSPBUF is only updated when SSPOV is clear
        // bit 3 SDAHT SDA Hold Time Selection bit, 1=Minimum of 300ns hold time on SDA after the falling edge of SCL, 0 = minimum of 100ns
        // bit 2 SBCDE Slave Mode Bus Collision Detect Enable bit; 1=Enable slave bus collision interrupts, 0 = Slave bus collision interrupts are disabled.
        // bit 1 AHEN Address Hold Enable bit (I2C Slave Mode only) 0=Address holding is disabled, 1=Following the 8th falling edge of SCL for a matching
        //      received address byte; CKP bit in the SSPCON1 register will be cleared and the SCL will be held low.
        // bit 0 DHEN Data Hold Enable bit (I2C Salve mode only) 0=Data holding is disabled, 1=Following the 8th falling edge of SCL for a received data byte; 
        //      slave hardware clears the CKP bit in the SSPCON1 register and SCL is held low.
    // </editor-fold>
    SSPIF = 0;
#endif
        
#ifdef MCP19125_ADM00745_CHARGER
	SSPADD = 0x20;// MSSP Address and baud Rate Register 1, Address is 0x20
    // <editor-fold defaultstate="collapsed" desc="SSPADD Register Bit Description">
                                        // 7-bit Slave Mode:
                                        // bit 7-1 7 bit address
                                        // bit 0 Unused
    // </editor-fold>
    //SSPADD2 = 0x20;// MSSP Address 2 , Address is 0x20, but is disabled for basic I2C
    // <editor-fold defaultstate="collapsed" desc="SSPADD2 Register Bit Description">
                                        // 7-bit Slave Mode:
                                        // bit 7-1 7 bit address
                                        // bit 0, 1 = Enable address matching with SSPADD2, 0 =Disable
    // </editor-fold>
    SSPCON1 = 0x26;// SSP Control Register 1  //Here's a tip.  Setting 0x26 locks out GPA4 for any external function.
    // <editor-fold defaultstate="collapsed" desc="SSPCON1 Register Bit Description">
                                        // bit 7 - WCOL, Write Collision Detect bit; 0 = No Collision, 1 = Write to SSPBUF attempted during bad conditions
                                        // bit 6 - SSPOV, Receive Overflow Indicator Bit, 0 = No Overflow, 1 = Byte received in SSPBUF while still holding previous byte
                                        // bit 5 - SSPEN, Synchronous Serial Port Enable bit, 0 = GPIO, 1 = Serial Port
                                        // bit 4 - CKP, clock polarity select bit, Slave Mode: 1 = Enable clock, 0 = Stretch Low; Master Mode: Unused
                                        // bit 3-0 - SSPM<3:0> synchronous serial port mode select bits
                                        // 0110 = I2C Slave Mode, 7-bit
                                        // 0111 = I2C Slave Mode, 10-bit
                                        // 1000 = I2C Master Mode, Clock = Fosc/(4XSSPADD+1))
                                        // 1011 = I2C firmware controlled master mode (slave idle)
                                        // 1110 = I2C Slave mode, 7-bit address with start and stop bit interrupts enabled
                                        // 1111 = I2C Slave mode, 10-bit address with start and stop bit interrupts enabled
    // </editor-fold>
    SSPCON2 = 0x00;// SSP Control Register 2
    // <editor-fold defaultstate="collapsed" desc="SSPCON2 Register Bit Description">
        // bit 7 GCEN General Call Enable bit (in I2C slave mode only)) 0=Disabled, 1=Enable interrupt on general call 0x00 received in SSPSR
        // bit 6 ACKSTAT Acknowledge Status bit, 1=Ack not received, 0=Ack received
        // bit 5 ACKDT Acknowledge Data bit (in receive mode) Value transmitted when the user initiates an Ack sequence at the end of a receive
        //      1 = Not Acknowledged, 0 = Acknowledged
        // bit 4 ACKEN Acknowledge Sequence Enable bit (in I2C Master mode only) 0=Ack Sequence Idle, 1= Initiate Ack sequence on SDA/SCL and transmit ACKDT, automatically cleared by hardware.
        // bit 3 RCEN Receive Enable bit (I2C Master mode only) 0=Receive Idle, 1=Enabled Receive mode for I2C
        // bit 2 PEN Stop Condition Enable bit (I2C Master mode only) SCK Release Control, 0=Repeated stop condition idle, 1=Initiate stop condition on SDA and SCL pins, automatically cleared by hardware
        // bit 1 RSEN Repeated Start condition Enabled bit (in I2C Master mode only) 0=Repeated start condition idle, 1=Initiate repeated start condition on SDA and SCL pins, automatically cleared by hardware
        // bit 0 SEN Start Condition Enabled bit (in I2C Master mode only)) Master: 0=Start condition Idle, 1=Initiate Start condition on SDA/SCL pins, cleared by hardware
        //      Slave: 1=Clock stretching is enabled for both slave transmit and slave receive (stretch enabled), 0=clock stretching disabled.
    // </editor-fold>
    SSPCON3 = 0x05;// SSP Control Register 3
    // <editor-fold defaultstate="collapsed" desc="SSPCON3 Register Bit Description">
        // bit 7 ACKTIM Acknowledge Time Status bit, 1=I2C bus is in an Ack sequence, set on the 8th falling edge of SCL clock
        //      0=Not an Ack sequence, cleared on the 9th rising edge of SCL clock
        // bit 6 PCIE Stop Condition Interrupt Enable bit, 1=Enable Intr on detection of Stop condition, 0=stop detection interrupts are disabled
        // bit 5 SCIE Start Condition Interrupt Enable bit, 1=Enable interrupt on detection of start or restart conditions, 0=start detection interrupts are disabled
        // bit 4 BOEN Buffer Overwrite Enable bit, Slave: 1=SSPBUF is updated and ACK# is generated for a received address/data byte, ignoring thes tate
        //      of the SSPOV bit only if the BF bit = 0, 0=SSPBUF is only updated when SSPOV is clear
        // bit 3 SDAHT SDA Hold Time Selection bit, 1=Minimum of 300ns hold time on SDA after the falling edge of SCL, 0 = minimum of 100ns
        // bit 2 SBCDE Slave Mode Bus Collision Detect Enable bit; 1=Enable slave bus collision interrupts, 0 = Slave bus collision interrupts are disabled.
        // bit 1 AHEN Address Hold Enable bit (I2C Slave Mode only) 0=Address holding is disabled, 1=Following the 8th falling edge of SCL for a matching
        //      received address byte; CKP bit in the SSPCON1 register will be cleared and the SCL will be held low.
        // bit 0 DHEN Data Hold Enable bit (I2C Salve mode only) 0=Data holding is disabled, 1=Following the 8th falling edge of SCL for a received data byte; 
        //      slave hardware clears the CKP bit in the SSPCON1 register and SCL is held low.
    // </editor-fold>
    SSPIF = 0;
#endif    
    
#ifdef PIC16F1716_MCP1631_SEPIC
	SSPADD = 0x20;	
    SSP1CON1 = 0x26;
    SSP1CON2 = 0x00;
    SSP1CON3 = 0x05;
    SSP1IF = 0;			// Clear interrupt flag
#endif

#ifdef PIC16F883_MCP1631_SEPIC
	SSPADD = 0x20;	
    SSPCON = 0x26;
    SSPCON2 = 0x00;
//    SSP1CON3 = 0x05;	This part doesn't have this register set extension to the MSSP
    SSPIF = 0;			// Clear interrupt flag
#endif
}


void service_SMBUS()
{
#ifdef ENABLE_SMBUS
    char a;                                 // General purpose local variables a & b & c
    unsigned short b;
    short c;
    extern struct charger_data_t cd;        // Charger Live Data structure called 'cd'
    extern struct charger_settings_t cs;	// Charger Settings structure called 'cs'
    extern bit chrg_off_batt_connected; 
    extern bit shutdown;
    extern bit start;
    
    SERIAL_INT_FLAG = 0;                    // Clear interupt flag
    //SSPIF = 0;
    b = SSPSTAT;                            // Get current status of i2c   
    // <editor-fold defaultstate="collapsed" desc="SSPSTAT Register Bit Description ">
                                        // bit 7 SMP: Data Input Sample bit, 1=Slew rate disabled for 100kHz and 1MHz, 0=enabled for 400kHz
                                        // bit 6 CKE: clock Edge Select bit, 1=SMBus compliant, 0=Disable SMBus specific inputs
                                        // bit 5 D/A#: Data/Address# bit, 1=last byte was data, 0=last byte was address
                                        // bit 4 P: Stop bit, 1=stop bit detected, 0=stop bit not detected last
                                        // bit 3 S: Start bit, 1=start bit detected, 0=start bit not detected last
                                        // bit 2 R/W#: Read/Write# Information, In Slave mode 1=Read, 0=Write. In master mode 1=transmit in progress, 0=not transmitting
                                        // bit 1 UA: Update Address bit (10b I2C mode only), 1=User needs to update address in SSPADD, 0=address update not needed
                                        // bit 0 BF: Buffer Full Status bit, Receive Mode 1=Receive complete, SSPBUF is full, 0=Receive not complete, SSPBUF empty.
                                        //           In Transmit mode 1=Data transmit in progress (not including Ack and Stop bits) and SSPBUF is full, 
                                        //           In Transmit mode 0=transmit not complete (not including Ack and Stop bits) and SSPBUF is empty.
    // </editor-fold>
    
    switch (b & 0x25)                       // Mask for address/data type, R/W direction, and buffer full (data-ready) bits
    {
        // <editor-fold defaultstate="collapsed" desc="Case 0x01 - Address Byte, Written, SSPBUF Full">
        case 0x01:                          // Slave receive (master write), an address byte was just sent 
            a = SSPBUF;                     // Get address from i2c Buffer (Clears SSPBUF)
            CKP = 1;                        // Release clock (from clock stretch function)
            smbusaddr_next = 0x2;             // Set flag that next byte will be data 
            break;
        // </editor-fold>
            
        // <editor-fold defaultstate="collapsed" desc="Case 0x21 - Data Byte, Written, SSPBUF Full">    

        case 0x21:                          // Slave receive (master write), a data byte was just sent 
            a = SSPBUF;                     // Get data from i2c buffer
            
            // <editor-fold defaultstate="collapsed" desc="Op-Code Process, First 2 bytes">                
            if (smbusaddr_next > 0)        // SMB Address command received already, this is data
            {
                if (smbusaddr_next & 0x2)
                {
                    flash_access = 0;           // Clear flash_access bit
                    sfr_access = 0;
                    struct_access = 0;
                    temperature_access = 0;
                    if ((a & 0xC0) == 0x80)
                    {
                        flash_access = 1;
                    }
                    else if ((a & 0xC0) == 0x40)
                    {
                        sfr_access = 1;
                    }
                    else if ((a & 0xC0) == 0xC0)
                    {
                        struct_access = 1;
                    }
                    /* Address data */
                    smbusaddr = (unsigned int)((a & 0x3f) << 8);		// Strip off top two high-bits
                    flashaddr = (unsigned int)((a & 0x3f) << 8);		// Put in flashaddr register (used for memory reads)
                    flashwritecounter = 0;		// Reset bit counter
                    registeraddr = (unsigned int)((a & 0x3f) << 8);
                }               
                else
                {
                    if (a == 0x40) //Since we don't want a GUI peak/poke to start messing around in flash this is a rigidly controlled 'op-code'
                    {
                        temperature_access = 1;
                    }
                    smbusaddr |= a;		// Strip off high-bit
                    flashaddr |= a;		// Put in flashaddr register (used for memory reads)
                    registeraddr |= a;
                }
                
                /* Data is next after two repetitions */
                smbusaddr_next--;			// Flag that next byte should be data 
            }
            //</editor-fold>
            
            // <editor-fold defaultstate="collapsed" desc="Charge Commands">
            else if (!flash_access && !sfr_access && !struct_access)   // control command received
            {
                switch(a) /* Commands */
                {
                    case CMD_CHARGER_OFF:
                        shutdown = 1;	 
                        //chrg_off_batt_connected = 0;
                        start = 0;
                        break;

                    case CMD_CHARGER_ON:
                        //chrg_off_batt_connected = 0;
                        start = 1;
                        cd.status.word = 0x0000; // Clear the shutdown cause
                        break;

                    case CMD_CHARGER_READCONFIG:
                        shutdown = 1;
                        load_saved_variables();
                        break;

                    case CMD_CHARGER_DISABLE:
                        chrg_off_batt_connected = 1;
                        shutdown = 1;
                        break;

                    case CMD_FUEL_GAUGE_MODE:
                        break;

                    case CMD_STATUS_LED_MODE:
                        break;

                    case CMD_ANALOG_DEBUG_MODE:
                        break;
                     
                    default: 				
                        break;							
                }             
            }
            //</editor-fold>   
            
            // <editor-fold defaultstate="collapsed" desc="SFR Writes">            
            else if (sfr_access)
            {
                unsigned short *p;           
                /* Writing to RAM */                           
                p = (unsigned short *)registeraddr;
                *p = a;
                registeraddr ++;
            }
            // </editor-fold>            
            
            // <editor-fold defaultstate="collapsed" desc="Flash Writes">
            else if (flash_access)  /* Calibration or configuration information */
            {
                /* Writing to flash, disable battery charger until */
                /* power cycle or SMBUS enable. */
                chrg_off_batt_connected = 1;

                flash_write_buf[flashwritecounter] = a;	// Fill write buffer with data at requested location  -
                flashwritecounter++;					// Increment the counter

                #ifdef ENABLE_GUI_CONFIG

                c = write_flash(CAL_BASE_ADDR + (flashaddr & 0xfc), flashwritecounter); //aligns the address on 4 16bit words.

                if (c > 0) // counter at correct length, iterate.
                { 
                    flashwritecounter = 0;              // Reset counter for next group of bits 
                }
                #endif
                if (smbusaddr & 0x01)
                {
                    flashaddr ++;
                }
                smbusaddr ++;
            }
            //</editor-fold>            
            CKP = 1;
            break;
        // </editor-fold>
            
        // <editor-fold defaultstate="collapsed" desc="Case 0x05 - Address Byte, Read, SSPBUF Full, Transmit in Progress">
        case 0x05: /* Slave transmit (master read), address just sent */
            a = SSPBUF;
        // </editor-fold>
           
        // <editor-fold defaultstate="collapsed" desc="Case 0x24 - Data Byte, Read, SSPBUF empty, Transmit Not Complete">
        case 0x24: /* Slave transmit (master read), data just sent */
            /* To ensure that data is sent with the high/low bytes intact */
            /* we buffer the transmitted data.*/
                
            // <editor-fold defaultstate="collapsed" desc="Reading Profile and Settings Structures">
            if (!flash_access && !sfr_access) // No opcode bits are set so it's a read of the profile structure
            {
                unsigned char *p; 
                if (struct_access)
                {
                    p = (unsigned char *) &cs;
                }
                else
                {
                    p = (unsigned char *) &cd;  // Data                                        
                }
                p += (smbusaddr );            // Add the offset to the pointer location
                SSPBUF = *p;                        // Send data pointed to by p
                //p++;
                //smbusbuf = *p;                      // Buffer the next data byte
            }
            //</editor-fold>
                
            // <editor-fold defaultstate="collapsed" desc="Reading SFR Registers (RAM)">
            else if (sfr_access /*&& !flash_access*/)
            {
                unsigned short *p;
                p = (unsigned short *)registeraddr;
                SSPBUF = *p;
                //p++;
                //smbusbuf = *p;
            }
            //</editor-fold>
                
            // <editor-fold defaultstate="collapsed" desc="Reading Flash">
            else if ((smbusaddr & 0x01) == 0)                // Even address
            { 
                if (/*!sfr_access &&*/ flash_access)
                {
                    if (temperature_access)
                    {
                        b = read_flash(TEMP_CAL_ADDR) & 0x03FF;
                    } 
                    else
                    {
                        b = read_flash(CAL_BASE_ADDR + flashaddr);
                    }
                  SSPBUF = b;                           // little endian                  
                  smbusbuf = b >> 8;
                }               
                else
                {
                    SSPBUF = 0xFF;
                    smbusbuf = 0xFF;
                }
            }
            else
            {
                SSPBUF = smbusbuf;
                flashaddr ++;
            }
            /*if (smbusaddr & 0x01) 
            {
                flashaddr ++;
            }*/
            //</editor-fold>
            
            smbusaddr ++;
            registeraddr ++;
            CKP = 1;
            break;
        // </editor-fold>
            
        // <editor-fold defaultstate="collapsed" desc="Case Default">
        default:
            break;
        // </editor-fold>
    }                                           

    if (SSPOV == 1)                                     // Receive overflow flag set?
    {
        SSPOV = 0;                                      // Reset it
    }
#endif
}                                                    