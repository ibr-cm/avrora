/**
 * Copyright (c) 2004-2005, Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the University of California, Los Angeles nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package avrora.sim.mcu;

import avrora.arch.avr.AVRProperties;
import avrora.arch.legacy.LegacyInterpreter;
import avrora.core.Program;
import avrora.sim.*;
import avrora.sim.clock.ClockDomain;
import avrora.sim.energy.Energy;
import cck.util.Arithmetic;
import java.util.HashMap;

/**
 * The <code>ATMega128RFA1</code> class represents the ATMega128RFA1 microcontroller from Atmel. This
 * microcontroller has 128Kb code, 16KB SRAM, 4KB EEPROM, and a host of internal devices such as
 * ADC, SPI, timers, and an internal AT86RF231-like 802.15.4 radio.
 *
 * @author Ben L. Titzer
 * @author Pekka Nikander
 * @author David A. Kopf
 */
public class ATMega128RFA1 extends ATMegaFamily {

    public static final int _1kb = 1024;

    public static final int ATMEGA128RFA1_IOREG_SIZE = 512 - 32;
    public static final int ATMEGA128RFA1_SRAM_SIZE = 16 * _1kb;
    public static final int ATMEGA128RFA1_FLASH_SIZE = 128 * _1kb;
    public static final int ATMEGA128RFA1_EEPROM_SIZE = 4 * _1kb;
    public static final int ATMEGA128RFA1_NUM_PINS = 65;
    public static final int ATMEGA128RFA1_NUM_INTS = 73;

    public static final int MODE_IDLE       = 1;
    public static final int MODE_ADCNRED    = 2;
    public static final int MODE_POWERDOWN  = 3;
    public static final int MODE_POWERSAVE  = 4;
    public static final int MODE_RESERVED1  = 5;
    public static final int MODE_RESERVED2  = 6;
    public static final int MODE_STANDBY    = 7;
    public static final int MODE_EXTSTANDBY = 8;

    protected static final String[] idleModeNames = {
        "Active",
        "Idle",
        "ADC Noise Reduction",
        "Power Down",
        "Power Save",
        "RESERVED 1",
        "RESERVED 2",
        "Standby",
        "Extended Standby"
    };

    //TODO:power consumption of each mode
    private static final double[] modeAmpere = {
        0.0075667,
        0.0033433,
        0.0009884,
        0.0001158,
        0.0001237,
        0.0,
        0.0,
        0.0002356,
        0.0002433
    };


    protected static final int[] wakeupTimes = { //TODO:
        0, 0, 0, 1000, 1000, 0, 0, 6, 6
    };

    protected final ActiveRegister SMCR_reg;

    private static final int[][] transitionTimeMatrix  = FiniteStateMachine.buildBimodalTTM(idleModeNames.length, 0, wakeupTimes, new int[wakeupTimes.length]);

    /**
     * The <code>props</code> field stores a static reference to a properties
     * object shared by all of the instances of this microcontroller. This object
     * stores the IO register size, SRAM size, pin assignments, etc.
     */
    public static final AVRProperties props;

    static {
        // statically initialize the pin assignments for this microcontroller
        HashMap pinAssignments = new HashMap(150);
        HashMap interruptAssignments = new HashMap(50);

        addPin(pinAssignments, 1, "PF2", "ADC2", "DIG2");
        addPin(pinAssignments, 2, "PF3", "ADC3", "DIG4");
        addPin(pinAssignments, 3, "PF4", "ADC4", "TCK");
        addPin(pinAssignments, 4, "PF5", "ADC5", "TMS");
        addPin(pinAssignments, 5, "PF6", "ADC6", "TDO");
        addPin(pinAssignments, 6, "PF7", "ADC7", "TDI");
        addPin(pinAssignments, 7, "AVSS_RFP");        
        addPin(pinAssignments, 8, "RFP");        
        addPin(pinAssignments, 9, "RFN");        
        addPin(pinAssignments,10, "AVSS_RFN");        
        addPin(pinAssignments,11, "TST");        
        addPin(pinAssignments,12, "RSTN");        
        addPin(pinAssignments,13, "RSTON");        
        addPin(pinAssignments,14, "PG0", "DIG3");        
        addPin(pinAssignments,15, "PG1", "DIG1");        
        addPin(pinAssignments,16, "PG2", "AMR");        
        addPin(pinAssignments,17, "PG3", "TOSC2");        
        addPin(pinAssignments,18, "PG4", "TOSC1");        
        addPin(pinAssignments,19, "PG5", "OC0B");        
        addPin(pinAssignments,20, "DVSS", "DSVSS");         
        addPin(pinAssignments,21, "DVDD"); 
        addPin(pinAssignments,22, "DVDD.b"); 
        addPin(pinAssignments,23, "DEVDD"); 
        addPin(pinAssignments,24, "DVSS.b"); 
        addPin(pinAssignments,25, "PD0", "SCL", "INT0");
        addPin(pinAssignments,26, "PD1", "SDA", "INT1");
        addPin(pinAssignments,27, "PD2", "RXD1", "INT2");
        addPin(pinAssignments,28, "PD3", "TXD1", "INT3");
        addPin(pinAssignments,29, "PD4", "ICP1");
        addPin(pinAssignments,30, "PD5", "XCK1");
        addPin(pinAssignments,31, "PD6", "T1");
        addPin(pinAssignments,32, "PD7", "T0");
        addPin(pinAssignments,33, "CLK");
        addPin(pinAssignments,34, "DEVDD.b");
        addPin(pinAssignments,35, "DVSS.c");
        addPin(pinAssignments,36, "PB0", "SSN", "PCINT0");
        addPin(pinAssignments,37, "PB1", "SCK", "PCINT1");
        addPin(pinAssignments,38, "PB2", "MOSI", "PDI", "PCINT2");
        addPin(pinAssignments,39, "PB3", "MISO", "PDO", "PCINT3");
        addPin(pinAssignments,40, "PB4", "OC2A", "PCINT4");
        addPin(pinAssignments,41, "PB5", "OC1A", "PCINT5");
        addPin(pinAssignments,42, "PB6", "OC1B", "PCINT6");
        addPin(pinAssignments,43, "PB7", "OC0A", "OC1C", "PCINT7");
        addPin(pinAssignments,44, "DEVDD.c");
        addPin(pinAssignments,45, "DVSS.d");
        addPin(pinAssignments,46, "PE0", "RXD0", "PCINT8");
        addPin(pinAssignments,47, "PE1", "TXD0");
        addPin(pinAssignments,48, "PE2", "XCK0", "AIN0");
        addPin(pinAssignments,49, "PE3", "OC3A", "AIN1");
        addPin(pinAssignments,50, "PE4", "OC3B", "INT4");
        addPin(pinAssignments,51, "PE5", "OC3C", "INT5");
        addPin(pinAssignments,52, "PE6", "T3", "INT6");
        addPin(pinAssignments,53, "PE7", "ICP3", "INT7", "CLK0");
        addPin(pinAssignments,54, "DEVDD.d");
        addPin(pinAssignments,55, "DVSS.e");
        addPin(pinAssignments,56, "XTAL2");
        addPin(pinAssignments,57, "XTAL1");
        addPin(pinAssignments,58, "AVSS");
        addPin(pinAssignments,59, "EVDD");
        addPin(pinAssignments,60, "AVDD");
        addPin(pinAssignments,61, "AVSS.b", "ASVSS");
        addPin(pinAssignments,62, "AREF");    
        addPin(pinAssignments,63, "PF0", "ADC0");       
        addPin(pinAssignments,64, "PF1", "ADC1");        
 
        RegisterLayout rl = new RegisterLayout(ATMEGA128RFA1_IOREG_SIZE, 8);

        // extended IO registers for the internal radio
        // only need to define those that do some action upon writing
/*
        rl.addIOReg("TRXFBEND", 0x1DF);
        // 128 byte TRX Buffer
        rl.addIOReg("TRXFBST", 0x160);

        rl.addIOReg("TST_RX_LENGTH", 0x15B);

        rl.addIOReg("TST_CTRL_DIGI", 0x156);

        rl.addIOReg("CSMA_BE", 0x14F);
        rl.addIOReg("CSMA_SEED_1", 0x14E);
        rl.addIOReg("CSMA_SEED_0", 0x14D);
        rl.addIOReg("XAH_CTRL_0", 0x14C);
        rl.addIOReg("IEEE_ADDR_7", 0x14B);
        rl.addIOReg("IEEE_ADDR_6", 0x14A);
        rl.addIOReg("IEEE_ADDR_5", 0x149);
        rl.addIOReg("IEEE_ADDR_4", 0x148);
        rl.addIOReg("IEEE_ADDR_3", 0x147);
        rl.addIOReg("IEEE_ADDR_2", 0x146);
        rl.addIOReg("IEEE_ADDR_1", 0x145);
        rl.addIOReg("IEEE_ADDR_0", 0x144);
        rl.addIOReg("PAN_ID_1", 0x143);
        rl.addIOReg("PAN_ID_0", 0x142);
        rl.addIOReg("SHORT_ADDR_1", 0x141);
        rl.addIOReg("SHORT_ADDR_0", 0x140); 

        rl.addIOReg("MAN_ID_1", 0x13F);
        rl.addIOReg("MAN_ID_0", 0x13E);
        rl.addIOReg("VERSION_NUM", 0x13D);
        rl.addIOReg("PART_NUM", 0x13C);
        rl.addIOReg("PLL_DCU", 0x13B);
        rl.addIOReg("PLL_CF", 0x13A);

        rl.addIOReg("FTN_CTRL", 0x138);
        rl.addIOReg("XAH_CTRL_1", 0x137);

        rl.addIOReg("RX_SYN", 0x135);

        rl.addIOReg("XOSC_CTRL", 0x132);
        rl.addIOReg("BATMON", 0x131);
        rl.addIOReg("VREG_CTRL", 0x130);   
*/
        rl.addIOReg("IRQ_STATUS", 0x12F);
        rl.addIOReg("IRQ_MASK", 0x12E);
        rl.addIOReg("ANT_DIV", 0x12D);
        rl.addIOReg("TRX_CTRL_2", 0x12C);
        rl.addIOReg("SFD_VALUE", 0x12B);
        rl.addIOReg("RX_CTRL", 0x12A);
        rl.addIOReg("CCA_THRES", 0x129);
        rl.addIOReg("PHY_CC_CCA", 0x128);
        rl.addIOReg("PHY_ED_LEVEL", 0x127);
        rl.addIOReg("PHY_RSSI", 0x126);
        rl.addIOReg("PHY_TX_PWR", 0x125);
        rl.addIOReg("TRX_CTRL_1", 0x124);
        rl.addIOReg("TRX_CTRL_0", 0x123);
        rl.addIOReg("TRX_STATE", 0x122, "TRAC_STATUS[2:0],TRX_CMD[4:0]");
  //      rl.addIOReg("TRX_STATUS", 0x121, "CCA_DONE, CCA_STATUS, TST_STATUS, TRX_STATUS[4:0]");

        rl.addIOReg("AES_KEY", 0x11F);
        rl.addIOReg("AES_STATE", 0x11E);
        rl.addIOReg("AES_STATUS", 0x11D);
        rl.addIOReg("AES_CTRL", 0x11C);

        rl.addIOReg("TRXPR", 0x119, "......,SLPTR,TRXRST");

        // extended io registers for mcu
        rl.addIOReg("DPDS1", 0x117);
        rl.addIOReg("DPDS0", 0x116);
        rl.addIOReg("DRTRAM0", 0x115);
        rl.addIOReg("DRTRAM1", 0x114);
        rl.addIOReg("DRTRAM2", 0x113);
        rl.addIOReg("DRTRAM3", 0x112);
        rl.addIOReg("LLDRH", 0x111);
        rl.addIOReg("LLDRL", 0x110);
        rl.addIOReg("LLCR", 0x10F);

        rl.addIOReg("OCR5CH", 0x10D);
        rl.addIOReg("OCR5CL", 0x10C);
        rl.addIOReg("OCR5BH", 0x10B);
        rl.addIOReg("OCR5BL", 0x10A);
        rl.addIOReg("OCR5AH", 0x109);
        rl.addIOReg("OCR5AL", 0x108);
        rl.addIOReg("ICR5H", 0x107);
        rl.addIOReg("ICR5L", 0x106);
        rl.addIOReg("TCNT5H", 0x105);
        rl.addIOReg("TCNT5L", 0x104);

        rl.addIOReg("TCCR5C", 0x102);
        rl.addIOReg("TCCR5B", 0x101);
        rl.addIOReg("TCCR5A", 0x100);

        rl.addIOReg("SCOCR1HH", 0xD8);
        rl.addIOReg("SCOCR1HL", 0xD7);
        rl.addIOReg("SCOCR1LH", 0xD6);
        rl.addIOReg("SCOCR1LL", 0xD5);
        rl.addIOReg("SCOCR2HH", 0xD4);
        rl.addIOReg("SCOCR2HL", 0xD3);
        rl.addIOReg("SCOCR2LH", 0xD2);
        rl.addIOReg("SCOCR2LL", 0xD1);
        rl.addIOReg("SCOCR3HH", 0xD0);
        rl.addIOReg("SCOCR3HL", 0xCF);
        rl.addIOReg("SCOCR3LH", 0xCE);
        rl.addIOReg("SCOCR3LL", 0xCD);
        rl.addIOReg("SCTSRHH", 0xCC);
        rl.addIOReg("SCTSRHL", 0xCB);
        rl.addIOReg("SCTSRLH", 0xCA);
        rl.addIOReg("SCTSRLL", 0xC9);
        rl.addIOReg("SCBTSRHH", 0xC8);
        rl.addIOReg("SCBTSRHL", 0xC7);
        rl.addIOReg("SCBTSRLH", 0xC6);
        rl.addIOReg("SCBTSRLL", 0xC5);
        rl.addIOReg("SCCNTHH", 0xC4);
        rl.addIOReg("SCCNTHL", 0xC3);
        rl.addIOReg("SCCNTLH", 0xC2);
        rl.addIOReg("SCCNTLL", 0xC1);
        rl.addIOReg("SCIRQS", 0xC0);
        rl.addIOReg("SCIRQM", 0xBF);

        rl.addIOReg("SCSR", 0xBE);
        rl.addIOReg("SCCR1", 0xBD);
        rl.addIOReg("SCCR0", 0xBC);

        rl.addIOReg("UDR1", 0xAE);
        rl.addIOReg("UBRR1H", 0xAD);
        rl.addIOReg("UBRR1L", 0xAC);

        rl.addIOReg("UCSR1C", 0xAA);
        rl.addIOReg("UCSR1B", 0xA9);
        rl.addIOReg("UCSR1A", 0xA8);

        rl.addIOReg("UDR0", 0xA6);
        rl.addIOReg("UBRR0H", 0xA5);
        rl.addIOReg("UBRR0L", 0xA4);

        rl.addIOReg("UCSR0C", 0xA2);
        rl.addIOReg("UCSR0B", 0xA1);
        rl.addIOReg("UCSR0A", 0xA0);

        rl.addIOReg("TWAMR", 0x9D);
        rl.addIOReg("TWCR", 0x9C);
        rl.addIOReg("TWDR", 0x9B);
        rl.addIOReg("TWAR", 0x9A);
        rl.addIOReg("TWSR", 0x99);
        rl.addIOReg("TWBR", 0x98);

        rl.addIOReg("ASSR", 0x96);

        rl.addIOReg("OCR2B", 0x94);
        rl.addIOReg("OCR2A", 0x93);
        rl.addIOReg("TCNT2", 0x92);
        rl.addIOReg("TCCR2B", 0x91);
        rl.addIOReg("TCCR2A", 0x90);

        rl.addIOReg("OCR4CH", 0x8D);
        rl.addIOReg("OCR4CL", 0x8C);
        rl.addIOReg("OCR4BH", 0x8B);
        rl.addIOReg("OCR4BL", 0x8A);
        rl.addIOReg("OCR4AH", 0x89);
        rl.addIOReg("OCR4AL", 0x88);
        rl.addIOReg("ICR4H", 0x87);
        rl.addIOReg("ICR4L", 0x86);
        rl.addIOReg("TCNT4H", 0x85);
        rl.addIOReg("TCNT4L", 0x84);

        rl.addIOReg("TCCR4C", 0x82);
        rl.addIOReg("TCCR4B", 0x81);
        rl.addIOReg("TCCR4A", 0x80);

        rl.addIOReg("OCR3CH", 0x7D);
        rl.addIOReg("OCR3CL", 0x7C);
        rl.addIOReg("OCR3BH", 0x7B);
        rl.addIOReg("OCR3BL", 0x7A);
        rl.addIOReg("OCR3AH", 0x79);
        rl.addIOReg("OCR3AL", 0x78);
        rl.addIOReg("ICR3H", 0x77);
        rl.addIOReg("ICR3L", 0x76);
        rl.addIOReg("TCNT3H", 0x75);
        rl.addIOReg("TCNT3L", 0x74);

        rl.addIOReg("TCCR3C", 0x72, "FOC3A,FOC3B,FOC3C,.....");
        rl.addIOReg("TCCR3B", 0x71, "ICNC3,ICES3,.,WGM3[3:2],CS3[2:0]");
        rl.addIOReg("TCCR3A", 0x70, "COM3A[1:0],COM3B[1:0],COM3C[1:0],WGM3[1:0]");

        rl.addIOReg("OCR1CH", 0x6D);
        rl.addIOReg("OCR1CL", 0x6C);
        rl.addIOReg("OCR1BH", 0x6B);
        rl.addIOReg("OCR1BL", 0x6A);
        rl.addIOReg("OCR1AH", 0x69);
        rl.addIOReg("OCR1AL", 0x68);
        rl.addIOReg("ICR1H", 0x67);
        rl.addIOReg("ICR1L", 0x66);
        rl.addIOReg("TCNT1H", 0x65);
        rl.addIOReg("TCNT1L", 0x64);

        rl.addIOReg("TCCR1C", 0x62, "FOC1A,FOC1B,FOC1C,.....");
        rl.addIOReg("TCCR1B", 0x61, "ICNC1,ICES1,.,WGM1[3:2],CS1[2:0]");
        rl.addIOReg("TCCR1A", 0x60, "COM1A[1:0],COM1B[1:0],COM1C[1:0],WGM1[1:0]");
        rl.addIOReg("DIDR1", 0x5F);
        rl.addIOReg("DIDR0", 0x5E);
        rl.addIOReg("DIDR2", 0x5D);
        rl.addIOReg("ADMUX", 0x5C);
        rl.addIOReg("ADCSRB", 0x5B);
        rl.addIOReg("ADCSRA", 0x5A);
        rl.addIOReg("ADCH", 0x59);
        rl.addIOReg("ADCL", 0x58);
        rl.addIOReg("ADCSRC", 0x57);

        rl.addIOReg("NEMCR", 0x55);

        rl.addIOReg("TIMSK5", 0x53);
        rl.addIOReg("TIMSK4", 0x52);
        rl.addIOReg("TIMSK3", 0x51);
        rl.addIOReg("TIMSK2", 0x50);
        rl.addIOReg("TIMSK1", 0x4F);
        rl.addIOReg("TIMSK0", 0x4E);
        rl.addIOReg("PCMSK2", 0x4D);
        rl.addIOReg("PCMSK1", 0x4C);
        rl.addIOReg("PCMSK0", 0x4B);
        rl.addIOReg("EICRB", 0x4A);
        rl.addIOReg("EICRA", 0x49);
        rl.addIOReg("PCICR", 0x48);
        rl.addIOReg("BGCR", 0x47);
        rl.addIOReg("OSCCAL", 0x46);
        rl.addIOReg("PRR1", 0x45);
        rl.addIOReg("PRR0", 0x44);
        rl.addIOReg("PRR2", 0x43);

        rl.addIOReg("CLKPR", 0x41);
        rl.addIOReg("WDTCSR", 0x40);

        // lower 64 IO registers
        rl.addIOReg("SREG", 0x3F);
        rl.addIOReg("SPH", 0x3E);
        rl.addIOReg("SPL", 0x3D);

        rl.addIOReg("RAMPZ", 0x3B);

        rl.addIOReg("SPMCSR", 0x37);
        rl.addIOReg("MCUCR", 0x35);

        rl.addIOReg("MCUSR", 0x34);
        rl.addIOReg("SMCR", 0x33);

        rl.addIOReg("OCDR", 0x31);
        rl.addIOReg("ACSR", 0x30);

        rl.addIOReg("SPDR", 0x2E);
        rl.addIOReg("SPSR", 0x2D);
        rl.addIOReg("SPCR", 0x2C);
        rl.addIOReg("GPIOR2", 0x2B);
        rl.addIOReg("GPIOR1", 0x2A);
        
        rl.addIOReg("OCR0B", 0x28);
        rl.addIOReg("OCR0A", 0x27);
        rl.addIOReg("TCNT0", 0x26);
        rl.addIOReg("TCCR0B", 0x25);
        rl.addIOReg("TCCR0A", 0x24);
        rl.addIOReg("GTCCR", 0x23);
        rl.addIOReg("EEARH", 0x22);
        rl.addIOReg("EEARL", 0x21);
        rl.addIOReg("EEDR", 0x20);
        rl.addIOReg("EECR", 0x1F);
        rl.addIOReg("GPIOR0", 0x1E);
        rl.addIOReg("EIMSK", 0x1D);
        rl.addIOReg("EIFR", 0x1C);
        rl.addIOReg("PCIFR", 0x1B);
        rl.addIOReg("TIFR5", 0x1A);
        rl.addIOReg("TIFR4", 0x19);
        rl.addIOReg("TIFR3", 0x18);
        rl.addIOReg("TIFR2", 0x17);
        rl.addIOReg("TIFR1", 0x16);
        rl.addIOReg("TIFR0", 0x15);
        rl.addIOReg("PORTG", 0x14);
        rl.addIOReg("DDRG", 0x13);
        rl.addIOReg("PING", 0x12);
        rl.addIOReg("PORTF", 0x11);
        rl.addIOReg("DDRF", 0x10);
        rl.addIOReg("PINF", 0x0F);
        rl.addIOReg("PORTE", 0x0E);
        rl.addIOReg("DDRE", 0x0D);
        rl.addIOReg("PINE", 0x0C);
        rl.addIOReg("PORTD", 0x0B);
        rl.addIOReg("DDRD", 0x0A);
        rl.addIOReg("PIND", 0x09);
        rl.addIOReg("PORTC", 0x08);
        rl.addIOReg("DDRC", 0x07);
        rl.addIOReg("PINC", 0x06);
        rl.addIOReg("PORTB", 0x05);
        rl.addIOReg("DDRB", 0x04);
        rl.addIOReg("PINB", 0x03);
        rl.addIOReg("PORTA", 0x02);
        rl.addIOReg("DDRA", 0x01);
        rl.addIOReg("PINA", 0x00);

        addInterrupt(interruptAssignments, "RESET", 1);
        addInterrupt(interruptAssignments, "INT0", 2);
        addInterrupt(interruptAssignments, "INT1", 3);
        addInterrupt(interruptAssignments, "INT2", 4);
        addInterrupt(interruptAssignments, "INT3", 5);
        addInterrupt(interruptAssignments, "INT4", 6);
        addInterrupt(interruptAssignments, "INT5", 7);
        addInterrupt(interruptAssignments, "INT6", 8);
        addInterrupt(interruptAssignments, "INT7", 9);
        addInterrupt(interruptAssignments, "PCINT0", 10);
        addInterrupt(interruptAssignments, "PCINT1", 11);
        addInterrupt(interruptAssignments, "PCINT2", 12);
        addInterrupt(interruptAssignments, "WDT", 13);
        addInterrupt(interruptAssignments, "TIMER2 COMPA", 14);
        addInterrupt(interruptAssignments, "TIMER2 COMPB", 15);
        addInterrupt(interruptAssignments, "TIMER2 OVF", 16);
        addInterrupt(interruptAssignments, "TIMER1 CAPT", 17);
        addInterrupt(interruptAssignments, "TIMER1 COMPA", 18);
        addInterrupt(interruptAssignments, "TIMER1 COMPB", 19);
        addInterrupt(interruptAssignments, "TIMER1 COMPC", 20);
        addInterrupt(interruptAssignments, "TIMER1 OVF", 21);
        addInterrupt(interruptAssignments, "TIMER0 COMPA", 22);
        addInterrupt(interruptAssignments, "TIMER0 COMPB", 23);
        addInterrupt(interruptAssignments, "TIMER0 OVF", 24);
        addInterrupt(interruptAssignments, "SPI, STC", 25);
        addInterrupt(interruptAssignments, "USART0, RX", 26);
        addInterrupt(interruptAssignments, "USART0, UDRE", 27);
        addInterrupt(interruptAssignments, "USART0, TX", 28);
        addInterrupt(interruptAssignments, "ANALOG COMP", 29);
        addInterrupt(interruptAssignments, "ADC", 30);
        addInterrupt(interruptAssignments, "EE READY", 31);
        addInterrupt(interruptAssignments, "TIMER3 CAPT", 32);
        addInterrupt(interruptAssignments, "TIMER3 COMPA", 33);
        addInterrupt(interruptAssignments, "TIMER3 COMPB", 34);
        addInterrupt(interruptAssignments, "TIMER3 COMPC", 35);
        addInterrupt(interruptAssignments, "TIMER3 OVF", 36);
        addInterrupt(interruptAssignments, "USART1, RX", 37);
        addInterrupt(interruptAssignments, "USART1, UDRE", 38);
        addInterrupt(interruptAssignments, "USART1, TX", 39);
        addInterrupt(interruptAssignments, "TWI", 40);
        addInterrupt(interruptAssignments, "SPM READY", 41);
        addInterrupt(interruptAssignments, "TIMER4CAPT", 42);
        addInterrupt(interruptAssignments, "TIMER4 COMPA", 43);
        addInterrupt(interruptAssignments, "TIMER4 COMPB", 44);
        addInterrupt(interruptAssignments, "TIMER4 COMPC", 45);
        addInterrupt(interruptAssignments, "TIMER4 OVF", 46);
        addInterrupt(interruptAssignments, "TIMER5 CAPT", 47);
        addInterrupt(interruptAssignments, "TIMER5 COMPA", 48);
        addInterrupt(interruptAssignments, "TIMER5 COMPB", 49);
        addInterrupt(interruptAssignments, "TIMER5 COMPC", 50);
        addInterrupt(interruptAssignments, "TIMER5 OVF", 51);
        addInterrupt(interruptAssignments, "RESERVED51", 52);
        addInterrupt(interruptAssignments, "RESERVED52", 53);
        addInterrupt(interruptAssignments, "RESERVED53", 54);
        addInterrupt(interruptAssignments, "RESERVED54", 55);
        addInterrupt(interruptAssignments, "RESERVED55", 56);
        addInterrupt(interruptAssignments, "RESERVED56", 57);
        addInterrupt(interruptAssignments, "TRX24 PLL_LOCK", 58);
        addInterrupt(interruptAssignments, "TRX24 PLL_UNLOCK", 59);
        addInterrupt(interruptAssignments, "TRX24 RX_START", 60);
        addInterrupt(interruptAssignments, "TRX24 RX_END", 61);
        addInterrupt(interruptAssignments, "TRX24 CCA_ED_DONE", 62);
        addInterrupt(interruptAssignments, "TRX24 XAH_AMI", 63);
        addInterrupt(interruptAssignments, "TRX24 TX_END", 64);
        addInterrupt(interruptAssignments, "TRX24 AWAKE", 65);
        addInterrupt(interruptAssignments, "SCNT CMP1", 66);
        addInterrupt(interruptAssignments, "SCNT CMP2", 67);
        addInterrupt(interruptAssignments, "SCNT_CMP3", 68);
        addInterrupt(interruptAssignments, "SCNT_OVFL", 69);
        addInterrupt(interruptAssignments, "SCNT BACKOFF", 70);
        addInterrupt(interruptAssignments, "AES READY", 71);
        addInterrupt(interruptAssignments, "BAT LOW", 72);

        props = new AVRProperties(ATMEGA128RFA1_IOREG_SIZE, // number of io registers
                ATMEGA128RFA1_SRAM_SIZE, // size of sram in bytes
                ATMEGA128RFA1_FLASH_SIZE, // size of flash in bytes
                ATMEGA128RFA1_EEPROM_SIZE, // size of eeprom in bytes
                ATMEGA128RFA1_NUM_PINS, // number of pins
                ATMEGA128RFA1_NUM_INTS, // number of interrupts
                new ReprogrammableCodeSegment.Factory(ATMEGA128RFA1_FLASH_SIZE, 7),
                pinAssignments, // the assignment of names to physical pins
                rl, // the assignment of names to IO registers
                interruptAssignments);

    }

    public static class Factory implements MicrocontrollerFactory {

        /**
         * The <code>newMicrocontroller()</code> method is used to instantiate a microcontroller instance for the
         * particular program. It will construct an instance of the <code>Simulator</code> class that has all the
         * properties of this hardware device and has been initialized with the specified program.
         *
         * @param sim
         *@param p the program to load onto the microcontroller @return a <code>Microcontroller</code> instance that represents the specific hardware device with the
         *         program loaded onto it
         */
        public Microcontroller newMicrocontroller(int id, Simulation sim, ClockDomain cd, Program p) {//current avrora source
            return new ATMega128RFA1(id, sim, cd, p);
        //public Microcontroller newMicrocontroller(int id, ClockDomain cd, Program p) {  //old avrora jar
         //   return new ATMega128RFA1(id, cd, p);
        }

    }
    public ATMega128RFA1(int id, Simulation sim, ClockDomain cd, Program p) {//for current avrora source
    //public ATMega128RFA1(int id, ClockDomain cd, Program p) {// for old avrora jar
        super(cd, props, new FiniteStateMachine(cd.getMainClock(), MODE_ACTIVE, idleModeNames, transitionTimeMatrix));
        simulator = sim.createSimulator(id, LegacyInterpreter.FACTORY, this, p);//for current avrora source
        //simulator = new Simulator(id, LegacyInterpreter.FACTORY, this, p);// for old avrora jar
        interpreter = (AtmelInterpreter)simulator.getInterpreter();
        SMCR_reg = getIOReg("SMCR");
        installPins();
        installDevices();
        new Energy("CPU", modeAmpere, sleepState, simulator.getEnergyControl());//for current avrora source
        //new Energy("CPU", modeAmpere, sleepState);//for old avrora jar
        byte SRAM = interpreter.getDataByte(0xff);
    }

    protected void installPins() {
        for (int cntr = 0; cntr < properties.num_pins; cntr++)
            pins[cntr] = new ATMegaFamily.Pin(cntr);
    }
    protected FlagRegister TIFR0_reg,MTIFR0_reg;
    protected MaskRegister TIMSK0_reg,MTIMSK0_reg;
    protected FlagRegister TIFR2_reg,MTIFR2_reg;
    protected MaskRegister TIMSK2_reg,MTIMSK2_reg;
    protected FlagRegister RIFR_reg,MRIFR_reg;
    protected MaskRegister RIMSK_reg,MRIMSK_reg;
    
    protected void installDevices() {
        // set up the external interrupt mask and flag registers and interrupt range
  //      EIFR_reg = buildInterruptRange(true, "EIMSK", "EIFR", 2, 8);

        // set up the timer mask and flag registers and interrupt range

        TIFR0_reg = buildInterruptRange(false, "TIMSK0", "TIFR0", 24, 3);
        TIMSK0_reg = (MaskRegister)getIOReg("TIMSK0");
        int[] TIFR0_mapping = {24, 22, 23, -1, -1, -1, -1, -1};
        MTIFR0_reg = new FlagRegister(interpreter, TIFR0_mapping);
        MTIMSK0_reg = new MaskRegister(interpreter, TIFR0_mapping);
        installIOReg("TIMSK0", MTIMSK0_reg);
        installIOReg("TIFR0", MTIFR0_reg);

        TIFR1_reg = buildInterruptRange(false, "TIMSK1", "TIFR1", 21, 5);
        TIMSK1_reg = (MaskRegister)getIOReg("TIMSK1");
        int[] TIFR1_mapping = {21, 18, 19, 20, -1, 17, -1, -1};
        MTIFR1_reg = new FlagRegister(interpreter, TIFR1_mapping);
        MTIMSK1_reg = new MaskRegister(interpreter, TIFR1_mapping);
        installIOReg("TIMSK1", MTIMSK1_reg);
        installIOReg("TIFR1", MTIFR1_reg);

        TIFR2_reg = buildInterruptRange(false, "TIMSK2", "TIFR2", 16, 3);
        TIMSK2_reg = (MaskRegister)getIOReg("TIMSK2");
        int[] TIFR2_mapping = {16, 14, 15, -1, -1, -1, -1, -1};
        MTIFR2_reg = new FlagRegister(interpreter, TIFR2_mapping);
        MTIMSK2_reg = new MaskRegister(interpreter, TIFR2_mapping);
        installIOReg("TIMSK2", MTIMSK2_reg);
        installIOReg("TIFR2", MTIFR2_reg);

        TIFR3_reg = buildInterruptRange(false, "TIMSK3", "TIFR3", 36, 5);
        TIMSK3_reg = (MaskRegister)getIOReg("TIMSK3");
        int[] TIFR3_mapping = {36, 33, 34, 35, -1, 32, -1, -1};
        MTIFR3_reg = new FlagRegister(interpreter, TIFR3_mapping);
        MTIMSK3_reg = new MaskRegister(interpreter, TIFR3_mapping);
        installIOReg("TIMSK3", MTIMSK3_reg);
        installIOReg("TIFR3", MTIFR3_reg);

        // set up the radio mask and flag registers and interrupt range
        RIFR_reg = buildInterruptRange(true, "IRQ_MASK", "IRQ_STATUS", 65, 8);
        RIMSK_reg = (MaskRegister)getIOReg("IRQ_MASK");
        int[] RIFR_mapping = {58, 59, 60, 61, 62, 63, 64, 65};
        MRIFR_reg = new FlagRegister(interpreter, RIFR_mapping);
        MRIMSK_reg = new MaskRegister(interpreter, RIFR_mapping);
        installIOReg("IRQ_MASK", MRIMSK_reg);
        installIOReg("IRQ_STATUS", MRIFR_reg);

        addDevice(new Timer0A());
        addDevice(new Timer1A(3));
        addDevice(new Timer2A());
        addDevice(new Timer3A(3));

//      buildPort('A');  //No PORTA pins
        buildPort('B');
//      buildPort('C');  //No PORTC pins
        buildPort('D');
        buildPort('E');
        buildPort('F');
//      buildPort('G'); //Not all PORTG pins

        addDevice(new EEPROM(properties.eeprom_size, this));
        addDevice(new USART("0", this));
        addDevice(new USART("1", this));

   //     addDevice(new SPI(this));
        addDevice(new RADIO(this));
        addDevice(new ADC(this, 8));
    }


    // permutation of sleep mode bits in the register (high order bits first)
        private static final int[] SMCR_sm_perm = { 3, 2, 1 };

    protected int getSleepMode() {
        byte value = SMCR_reg.read();
        boolean sleepEnable = Arithmetic.getBit(value, 0);

        if ( sleepEnable )
            return Arithmetic.getBitField(value, SMCR_sm_perm) + 1;
        else
            return MODE_IDLE;
    }

}
