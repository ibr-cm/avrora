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
 * The <code>ATMega1284p</code> class represents the ATMega1284p microcontroller from Atmel. This
 * microcontroller has 128Kb code, 16KB SRAM, 4KB EEPROM, and a host of internal devices such as
 * ADC, SPI, timers.
 *
 * @author Ben L. Titzer
 * @author Pekka Nikander
 * @author David A. Kopf
 */
public class ATMega1284p extends ATMegaFamily {
    public static final int _1kb = 1024;
    
    public static final int ATMEGA1284P_IOREG_SIZE = 512 - 32;
    public static final int ATMEGA1284P_SRAM_SIZE = 16 * _1kb;
    public static final int ATMEGA1284P_FLASH_SIZE = 128 * _1kb;
    public static final int ATMEGA1284P_EEPROM_SIZE = 4 * _1kb;
 // public static final int ATMEGA1284P_NUM_PINS = 41;  //PDIP
    public static final int ATMEGA1284P_NUM_PINS = 65;  //TQFP   
    public static final int ATMEGA1284P_NUM_INTS = 36;

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

    //power consumption of each mode
    //TODO: calculate current from PRR bits
/* 8 MHz @ 3v3 @ 25C with watchdog, estimates from preliminary data sheet graphs */
    private static final double[] modeAmpere = {
        0.0044,
        0.0006,
        0.000055,
        0.0000046,
        0.00009,   //WDT disabled
        0.0,
        0.0,
        0.000016, // Oscillator runs, WDT disabled, wake in 6 cycles
        0.000016  //?Oscillator runs, wake in 6 clock cycles
    };

    //BOD disabled during sleep gives 60us wake time = 480 clocks @ 8 MHz

    protected static final int[] wakeupTimes = { //In clock cycles TODO:
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
    if (false) {  //PDIP
    } else {      //TQFP
        addPin(pinAssignments, 1, "PB5", "PCINT13", "ICP3", "MOSI");
        addPin(pinAssignments, 2, "PB6", "PCINT14", "OC3A", "MISO");
        addPin(pinAssignments, 3, "PB7", "PCINT15", "OC3B", "SCK");
        addPin(pinAssignments, 4, "RESET");
        addPin(pinAssignments, 5, "VCC");
        addPin(pinAssignments, 6, "GND");
        addPin(pinAssignments, 7, "XTAL2");
        addPin(pinAssignments, 8, "XTAL1");
        addPin(pinAssignments, 9, "PD0", "PCINT24", "RXD0", "T3");
        addPin(pinAssignments,10, "PD1", "PCINT25", "TXD0");
        addPin(pinAssignments,11, "PD2", "PCINT26", "RXD1", "INT0");
        addPin(pinAssignments,12, "PD3", "PCINT27", "TXD1", "INT1");
        addPin(pinAssignments,13, "PD4", "PCINT28", "XCK1", "OC1B");
        addPin(pinAssignments,14, "PD5", "PCINT29", "OC1A");
        addPin(pinAssignments,15, "PD6", "PCINT30", "OC2B", "ICP");
        addPin(pinAssignments,16, "PD7", "PCINT31", "OC2A");
        addPin(pinAssignments,17, "VCC.b");
        addPin(pinAssignments,18, "GND.b");
        addPin(pinAssignments,19, "PC0", "PCINT16", "SCL");
        addPin(pinAssignments,20, "PC1", "PCINT17", "SCA");
        addPin(pinAssignments,21, "PC2", "PCINT18", "TCK");
        addPin(pinAssignments,22, "PC3", "PCINT19", "TMS");
        addPin(pinAssignments,23, "PC4", "PCINT20", "TDO");
        addPin(pinAssignments,24, "PC5", "PCINT21", "TDI");
        addPin(pinAssignments,25, "PC6", "PCINT22", "TOSC1");
        addPin(pinAssignments,26, "PC7", "PCINT23", "TOSC2");
        addPin(pinAssignments,27, "AVCC");
        addPin(pinAssignments,28, "GND.c");
        addPin(pinAssignments,29, "AREF");
        addPin(pinAssignments,30, "PA7", "PCINT7");
        addPin(pinAssignments,31, "PA6", "PCINT6");
        addPin(pinAssignments,32, "PA5", "PCINT5");
        addPin(pinAssignments,33, "PA4", "PCINT4");
        addPin(pinAssignments,34, "PA3", "PCINT3");
        addPin(pinAssignments,35, "PA2", "PCINT2");
        addPin(pinAssignments,36, "PA1", "PCINT1");
        addPin(pinAssignments,37, "PA0", "PCINT0");
        addPin(pinAssignments,38, "VCC.c");
        addPin(pinAssignments,39, "GND.d");
        addPin(pinAssignments,40, "PB0", "PCINT8", "XCK0", "T0");
        addPin(pinAssignments,41, "PB1", "PCINT9", "CLKO", "T1");
        addPin(pinAssignments,42, "PB2", "PCINT10", "INT2", "AIN0");
        addPin(pinAssignments,43, "PB3", "PCINT11", "OC0A", "AIN1");
        addPin(pinAssignments,44, "PB4", "PCINT12", "OC0B", "SS");
    }
        RegisterLayout rl = new RegisterLayout(ATMEGA1284P_IOREG_SIZE, 8);
        // extended io registers for mcu
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

        rl.addIOReg("ADMUX", 0x5C);
        rl.addIOReg("ADCSRB", 0x5B);
        rl.addIOReg("ADCSRA", 0x5A);
        rl.addIOReg("ADCH", 0x59);
        rl.addIOReg("ADCL", 0x58);

        rl.addIOReg("PCMSK3", 0x53);

        rl.addIOReg("TIMSK3", 0x51);
        rl.addIOReg("TIMSK2", 0x50);
        rl.addIOReg("TIMSK1", 0x4F);
        rl.addIOReg("TIMSK0", 0x4E);
        rl.addIOReg("PCMSK2", 0x4D);
        rl.addIOReg("PCMSK1", 0x4C);
        rl.addIOReg("PCMSK0", 0x4B);

        rl.addIOReg("EICRA", 0x49);
        rl.addIOReg("PCICR", 0x48);

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

        rl.addIOReg("TIFR3", 0x18);
        rl.addIOReg("TIFR2", 0x17);
        rl.addIOReg("TIFR1", 0x16);
        rl.addIOReg("TIFR0", 0x15);

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
        addInterrupt(interruptAssignments, "PCINT0", 5);
        addInterrupt(interruptAssignments, "PCINT1", 6);
        addInterrupt(interruptAssignments, "PCINT2", 7);
        addInterrupt(interruptAssignments, "PCINT3", 8);
        
        addInterrupt(interruptAssignments, "WDT", 9);
        addInterrupt(interruptAssignments, "TIMER2 COMPA", 10);
        addInterrupt(interruptAssignments, "TIMER2 COMPB", 11);
        addInterrupt(interruptAssignments, "TIMER2 OVF", 12);
        addInterrupt(interruptAssignments, "TIMER1 CAPT", 13);
        addInterrupt(interruptAssignments, "TIMER1 COMPA", 14);
        addInterrupt(interruptAssignments, "TIMER1 COMPB", 15);
        addInterrupt(interruptAssignments, "TIMER1 OVF", 16);
        addInterrupt(interruptAssignments, "TIMER0 COMPA", 17);
        addInterrupt(interruptAssignments, "TIMER0 COMPB", 18);
        addInterrupt(interruptAssignments, "TIMER0 OVF", 19);
        addInterrupt(interruptAssignments, "SPI, STC", 20);
        addInterrupt(interruptAssignments, "USART0, RX", 21);
        addInterrupt(interruptAssignments, "USART0, UDRE", 22);
        addInterrupt(interruptAssignments, "USART0, TX", 23);
        addInterrupt(interruptAssignments, "ANALOG COMP", 24);
        addInterrupt(interruptAssignments, "ADC", 25);
        addInterrupt(interruptAssignments, "EE READY", 26);
        addInterrupt(interruptAssignments, "TWI", 27);
        addInterrupt(interruptAssignments, "SPM READY", 28);
        addInterrupt(interruptAssignments, "USART1, RX", 29);
        addInterrupt(interruptAssignments, "USART1, UDRE", 30);
        addInterrupt(interruptAssignments, "USART1, TX", 31);
        addInterrupt(interruptAssignments, "TIMER3 CAPT", 32);
        addInterrupt(interruptAssignments, "TIMER3 COMPA", 33);
        addInterrupt(interruptAssignments, "TIMER3 COMPB", 34);
        addInterrupt(interruptAssignments, "TIMER3 OVF", 35);

        props = new AVRProperties(ATMEGA1284P_IOREG_SIZE, // number of io registers
                ATMEGA1284P_SRAM_SIZE, // size of sram in bytes
                ATMEGA1284P_FLASH_SIZE, // size of flash in bytes
                ATMEGA1284P_EEPROM_SIZE, // size of eeprom in bytes
                ATMEGA1284P_NUM_PINS, // number of pins
                ATMEGA1284P_NUM_INTS, // number of interrupts
                new ReprogrammableCodeSegment.Factory(ATMEGA1284P_FLASH_SIZE, 7),
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
        public Microcontroller newMicrocontroller(int id, Simulation sim, ClockDomain cd, Program p) { //for current avrora source
        return new ATMega1284p(id, sim, cd, p);
        //public Microcontroller newMicrocontroller(int id, ClockDomain cd, Program p) {   // for old avrora jar
        //   return new ATMega1284p(id, cd, p);
        }

    }

     public ATMega1284p(int id, Simulation sim, ClockDomain cd, Program p) {//for current avrora source
    //public ATMega1284p(int id, ClockDomain cd, Program p) {// for old avrora jar
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
    
    protected void installDevices() {
        // set up the external interrupt mask and flag registers and interrupt range
  //      EIFR_reg = buildInterruptRange(true, "EIMSK", "EIFR", 2, 8);

        TIFR0_reg = buildInterruptRange(false, "TIMSK0", "TIFR0", 19, 3);
        TIMSK0_reg = (MaskRegister)getIOReg("TIMSK0");
        int[] TIFR0_mapping = {19, 17, 18, -1, -1, -1, -1, -1};
        MTIFR0_reg = new FlagRegister(interpreter, TIFR0_mapping);
        MTIMSK0_reg = new MaskRegister(interpreter, TIFR0_mapping);
        installIOReg("TIMSK0", MTIMSK0_reg);
        installIOReg("TIFR0", MTIFR0_reg);

        TIFR1_reg = buildInterruptRange(false, "TIMSK1", "TIFR1", 16, 4);
        TIMSK1_reg = (MaskRegister)getIOReg("TIMSK1");
        int[] TIFR1_mapping = {16, 14, 15, -1, -1, 13, -1, -1};
        MTIFR1_reg = new FlagRegister(interpreter, TIFR1_mapping);
        MTIMSK1_reg = new MaskRegister(interpreter, TIFR1_mapping);
        installIOReg("TIMSK1", MTIMSK1_reg);
        installIOReg("TIFR1", MTIFR1_reg);

        TIFR2_reg = buildInterruptRange(false, "TIMSK2", "TIFR2", 12, 3);
        TIMSK2_reg = (MaskRegister)getIOReg("TIMSK2");
        int[] TIFR2_mapping = {12, 10, 11, -1, -1, -1, -1, -1};
        MTIFR2_reg = new FlagRegister(interpreter, TIFR2_mapping);
        MTIMSK2_reg = new MaskRegister(interpreter, TIFR2_mapping);
        installIOReg("TIMSK2", MTIMSK2_reg);
        installIOReg("TIFR2", MTIFR2_reg);

        TIFR3_reg = buildInterruptRange(false, "TIMSK3", "TIFR3", 35, 4);
        TIMSK3_reg = (MaskRegister)getIOReg("TIMSK3");
        int[] TIFR3_mapping = {35, 33, 34, -1, -1, 32, -1, -1};
        MTIFR3_reg = new FlagRegister(interpreter, TIFR3_mapping);
        MTIMSK3_reg = new MaskRegister(interpreter, TIFR3_mapping);
        installIOReg("TIMSK3", MTIMSK3_reg);
        installIOReg("TIFR3", MTIFR3_reg);

        addDevice(new Timer0A());
        addDevice(new Timer1A(2));
        addDevice(new Timer2A());
        addDevice(new Timer3A(2));

        buildPort('A');
        buildPort('B');
        buildPort('C');
        buildPort('D');

        addDevice(new EEPROM(properties.eeprom_size, this));
        addDevice(new USART("0", this));
        addDevice(new USART("1", this));

        addDevice(new SPI(this));
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
