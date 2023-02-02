// MultiChemCharger_Values.h
// This file provides the hard-coded configuration file required when
// you want a fixed-function charger and don't want to use the GUI to
// configure the system in manufacturing. Which will be most cases.
// 
// To use this file, please enable the #define in MultiChemCharger_Main.h
// labeled '#define ENABLE_FIXED_CONFIG'.
// 
// Note that the only data used by the PIC Firmware is the block below that includes
// descriptions.  All others are used by the GUI to re-populate the configuration there.
//
// Also note that all values below are in ADC counts and represent for the most part the key
// transtion point in the charging algorithm such as the CC/CV transition.
// You can manually edit this file instead of using the GUI by changing these parameters
// based on proper analysis of the circuit.  Changing voltage divider ratios on inputs would
// be a good example of why this would be necessary.
// 
// The Cal_Data_Array is not used by the firmware at all, only by the GUI to determine what
// the ADC counts should be to realize the desired value entered in the GUI.
// 
// 
 
#ifdef ENABLE_FIXED_CONFIG
 
#asm
 
psect text,class=CODE,local,abs,ovrld,delta=2
org 0xe60        // Change this line if you change the location in memory for the config/cal data
_Cal_Data_Array:
    dw 0119h
    dw 052Dh
    dw 09B8h
    dw 020Bh
    dw 0308h
    dw 0503h
    dw 035Bh
    dw 05A9h
    dw 0781h
    dw 006Bh
    dw 03E5h
    dw 07C3h
    dw 02DFh
    dw 04D4h
    dw 0663h
    dw 0320h
    dw 04AFh
    dw 07CFh
    dw 00FAh
    dw 0741h
    dw 3FFFh
    dw 3FFFh
    dw 3FFFh
    dw 3FFFh
    dw 3FFFh
    dw 3FFFh
    dw 3FFFh
    dw 3FFFh
    dw 3FFFh
    dw 3FFFh
    dw 3FFFh
    dw 3FFFh
#endasm
// Configuration Data: 0xE80
#asm
 
psect text,class=CODE,local,abs,ovrld,delta=2
org 0xe80
_Config_Data_Array:
    dw 0012h  // 0 uvlo_threshold_off;      // SFR Setting - UVLO off threshold
    dw 0018h  // 2 uvlo_threshold_on;       // SFR Setting - UVLO on threshold
    dw 05E9h  // 4 vbat_ov;                 // ADC Value - Output overvoltage shutdown threshold
    dw 041Dh  // 6 vbat_pc;                 // ADC Value - Battery voltage PRE to CC
    dw 05C6h  // 8 vbat_cv;                 // ADC Value - Battery voltage CC to CV
    dw 0110h  // 10 ibat_pc;                // ADC Value - Preconditioning battery current
    dw 077Ch  // 12 ibat_cc;                // ADC Value - Charge current
    dw 01C0h  // 14 ibat_ct;                // ADC Value - Charge termination current
    dw 0001h  // 16 ovcfcon_min;            // SFR Comparison - Minimum setting for output DAC
    dw 00FFh  // 18 ovcfcon_max;            // SFR Comparison - Maximum setting for output DAC
    dw 020Bh  // 20 uvlo_adc_threshold_off; // ADC Value - Input UV threshold for turn off (ADC)
    dw 028Ah  // 22 uvlo_adc_threshold_on;  // ADC Value - Input UV threshold to allow turn-on (ADC)
    dw 0000h  // 24 restore_time_max;       // Counter Value & TextBox - RESTORE charge maximum time (in seconds, the GUI displays minutes)
    dw 0096h  // 26 dvdt_blank_time;        // Counter Value & TextBox - dV/dt blank time (in seconds, the GUI displays minutes)
    dw 0000h  // 28 rapid_time_max;         // Counter Value & TextBox - RAPID charge maximum time (in seconds, the GUI displays minutes)
    dw 0000h  // 30 dtdt_slope;             // ADC Value - dt/dt negative delta_t ADC change to terminate charge
    dw 0000h  // 32 pack_temp_min;          // ADC Value - Minimum pack temperature threshold
    dw 0FFFh  // 34 pack_temp_max;          // ADC Value - Maximum pack temperature threshold
    dw 0000h  // 36 chemistry;              // Constant & TextBox - Chemistry (0 = Li-Ion, 1 = NiMH, 2 = VRLA CCCP, 3 = VRLA Fast, 4 = LiFePO4)
    dw 0000h  // 38 ovrefcon_ov;            // SFR Setting - MCP19125 specific - OVREFCON_OV sets the reference for OV comparator when enabled
    dw 0000h  // 40 ovrefcon_cv;            // SFR Setting - MCP19125 specific - OVREFCON_CV sets the reference for EA1 when enabled
    dw 0000h  // 42 vrefcon_max;            // SFR Setting - MCP19125 specific - Maximum value allowed for VREFCON - Basically max output current
    dw 05C6h  // 44 vbat_fv;                // ADC Value - Battery voltage for float
    dw 3FFEh  // 46 neg_dvdt;               // Constant & TextBox - Threshold for Negative dV/dt
    dw 0006h  // 48 pos_dvdt_cc;            // Constant & TextBox - Threshold for Positive dV/dt in CC mode
    dw 0004h  // 50 neg_didt_cv;            // Constant & TextBox - Threshold for Negative dI/dt in CV mode
    dw 0000h  // 52 pack_temp_25C;          // ADC Value - The calculated value for a 25C reading for reference
    dw 0FFFh  // 54 temp_1C;                // ADC Value - Represents the number of counts for a calculated 1C change
    dw 0000h  // 56 temp_sense_en;          // Constant & TextBox - Is temperature Enabled
    dw 0110h  // 58 ibat_fc;                // ADC Value - Maximum allowed Float Current
    dw 012Ch  // 60 precondition_max_time   // Counter Value & TextBox - Maximum time charger will try to restore/precondition a battery
    dw 000Ah  // 62 pc_dvdt                 // Constant & TextBox - Threshold for Positive dV/dt in PC mode
    dw 0000h  // 64 vbat_1C_adjust          // ADC Value - The adjustment to float voltage for every 1C change from 25C
    dw 0000h  // 66 charge_time_max;        // Counter Value & TextBox - Low Byte Maximum total charge time (in seconds, the GUI displays in minutes)
    dw 0000h  // 68 charge_time_max;        // Counter Value & TextBox - High Byte Maximum charge time before shut-off
    dw 3FFFh  // 70
    dw 3FFFh  // 72
    dw 3FFFh  // 74
    dw 0E74h  // 76 cell voltage            // TextBox
    dw 0003h  // 78 # of Cells              // TextBox
    dw 0BB8h  // 80 Precondition Voltage    // TextBox
    dw 0064h  // 82 Precondition Current    // TextBox
    dw 05DCh  // 84 Charge Current          // TextBox
    dw 00FAh  // 86 Termination Current     // TextBox
    dw 01F4h  // 88 ADC Vref                // TextBox
    dw 000Ah  // 90 ADC Size                // TextBox
    dw 000Ah  // 92 Temp Slope V/C          // TextBox
    dw 0000h  // 94 Minimum Temperature     // TextBox
    dw 003Ch  // 96 Maximum Temperautre     // TextBox
    dw 1068h  // 98 Termination Voltage     // TextBox
    dw 0064h  // 100 Wire Resistance in Ohms // TextBox
    dw 1068h  // 102 Float Voltage           // TextBox
    dw 10CCh  // 104 Over Voltage            // TextBox
    dw 0096h  // 106 dtdt Slope              // TextBox
    dw 2710h  // 108 UVLO Rising             // TextBox
    dw 1F40h  // 110 UVLO Falling            // TextBox
    dw 0064h  // 112 Maximum Float Current   // TextBox
    dw 3FFFh  // 114
    dw 3FFFh  // 116
    dw 3FFFh  // 118
    dw 3FFFh  // 120
    dw 3FFFh  // 122
    dw 3FFFh  // 124
    dw 3FFFh  // 126
#endasm
 
 
#endif
