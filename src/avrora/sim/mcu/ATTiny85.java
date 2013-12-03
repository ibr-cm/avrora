/**
 * Copyright (c) 2012, Regents of the University of California
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
 * The <code>ATTiny85</code> class represents the microcontroller from Atmel. This
 * microcontroller has 8KB code, 512B SRAM, 512B EEPROM, and a host of internal devices such as
 * ADC, SPI, timers.
 *
 * @author David Kopf
 */
public class ATTiny85 extends ATMegaFamily {
    public static final int _1kb = 1024;
    
    public static final int MCU_IOREG_SIZE  = 64;
    public static final int MCU_SRAM_SIZE   = 512;
    public static final int MCU_FLASH_SIZE  = 8 * _1kb;
    public static final int MCU_PAGE_SIZE   = 32;
    public static final int MCU_EEPROM_SIZE = 512;
    public static final int MCU_NUM_PINS    = 9;  //PDIP SOIC TSSOP  
 // public static final int MCU_NUM_PINS    = 20;  //QFN MLF  
    public static final int MCU_NUM_INTS    = 16;//16?

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
        addPin(pinAssignments, 1, "PB5", "PCINT5", "RESET", "ADC0", "dW");
        addPin(pinAssignments, 2, "PB3", "PCINT3", "XTAL1", "CLKI", "NOC1B", "ADC3");
        addPin(pinAssignments, 3, "PB4", "PCINT4", "XTAL2", "CLK0", "OC1B", "ADC2");
        addPin(pinAssignments, 4, "GND");
        addPin(pinAssignments, 5, "PB0", "PCINT0", "AREF", "NOC1A", "OC0A", "AIN0", "SDA", "DI", "MOSI");
        addPin(pinAssignments, 6, "PB1", "PCINT1", "OC1A", "OC0B", "AIN1", "DO", "MISO");
        addPin(pinAssignments, 7, "PB2", "PCINT2", "INT0", "T0", "ADC1", "SCL", "USCK", "SCK");
        addPin(pinAssignments, 8, "VCC");
    }
        RegisterLayout rl = new RegisterLayout(MCU_IOREG_SIZE, 8);
        // extended io registers for mcu

        // lower 64 IO registers
        rl.addIOReg("SREG", 0x3F);
        rl.addIOReg("SPH", 0x3E);
        rl.addIOReg("SPL", 0x3D);

        rl.addIOReg("GIMSK", 0x3B);
        rl.addIOReg("GIFR", 0x3A);
        rl.addIOReg("TIMSK", 0x39);
        rl.addIOReg("TIFR", 0x38);
        rl.addIOReg("SPMCSR", 0x37);

        rl.addIOReg("MCUCR", 0x35);
        rl.addIOReg("MCUSR", 0x34);
        rl.addIOReg("TCCR0", 0x33);//TODO FIX TIMERS
     //   rl.addIOReg("TCCR0B", 0x33);
        rl.addIOReg("TCNT0", 0x32);
        rl.addIOReg("OSCCAL", 0x31);
        rl.addIOReg("TCCR1", 0x30);
        rl.addIOReg("TCNT1", 0x2F);
        rl.addIOReg("OCR1A", 0x2E);
        rl.addIOReg("OCR1C", 0x2D);
        rl.addIOReg("GTCCR", 0x2C);
        rl.addIOReg("OCR1B", 0x2B);
        rl.addIOReg("TCCR0A", 0x2A);
       // rl.addIOReg("OCR0A", 0x29);  
        rl.addIOReg("OCR0", 0x29);             
        rl.addIOReg("OCR0B", 0x28);
        rl.addIOReg("PLLCSR", 0x27);
        rl.addIOReg("CLKPR", 0x26);
        rl.addIOReg("DT1A", 0x25);
        rl.addIOReg("DT1B", 0x24);
        rl.addIOReg("DTPS1", 0x23);
        rl.addIOReg("DWDR", 0x22);
        rl.addIOReg("WDTCR", 0x21);
        rl.addIOReg("PRR", 0x20);
        rl.addIOReg("EEARH", 0x1F);
        rl.addIOReg("EEARL", 0x1E);
        rl.addIOReg("EEDR", 0x1D);
        rl.addIOReg("EECR", 0x1C);

        rl.addIOReg("PORTB", 0x18);
        rl.addIOReg("DDRB", 0x17);
        rl.addIOReg("PINB", 0x16);
        rl.addIOReg("PCMSK", 0x15);
        rl.addIOReg("DIDR0", 0x14);
        rl.addIOReg("GPIOR2", 0x13);
        rl.addIOReg("GPIOR1", 0x12);
        rl.addIOReg("GPIOR0", 0x11);       
        rl.addIOReg("USIBR", 0x10);        
        rl.addIOReg("USIDR", 0x0F);          
        rl.addIOReg("USISR", 0x0E);
        rl.addIOReg("USICR", 0x0D);

        rl.addIOReg("ACSR", 0x08);
        rl.addIOReg("ADMUX", 0x07);
        rl.addIOReg("ADCSRA", 0x06);
        rl.addIOReg("ADCH", 0x05);
        rl.addIOReg("ADCL", 0x04);
        rl.addIOReg("ADCSRB", 0x03);
        rl.addIOReg("PORTA", 0x02);

        addInterrupt(interruptAssignments, "RESET", 1);
        addInterrupt(interruptAssignments, "INT0", 2);
        addInterrupt(interruptAssignments, "PCINT0", 3);
        addInterrupt(interruptAssignments, "TIMER1 COMPA", 4);
        addInterrupt(interruptAssignments, "TIMER1 OVF", 5);
        addInterrupt(interruptAssignments, "TIMER0 OVF", 6);
        addInterrupt(interruptAssignments, "EE_RDY", 7);
        addInterrupt(interruptAssignments, "ANA_COMP", 8);
        
        addInterrupt(interruptAssignments, "ADC", 9);
        addInterrupt(interruptAssignments, "TIMER1 COMPB", 10);
        addInterrupt(interruptAssignments, "TIMER0 COMPA", 11);
        addInterrupt(interruptAssignments, "TIMER0 COMPB", 12);
        addInterrupt(interruptAssignments, "WDT", 13);
        addInterrupt(interruptAssignments, "USI_START", 14);
        addInterrupt(interruptAssignments, "USI_OVF", 15);

        props = new AVRProperties(MCU_IOREG_SIZE, // number of io registers
                MCU_SRAM_SIZE, // size of sram in bytes
                MCU_FLASH_SIZE, // size of flash in bytes
                MCU_EEPROM_SIZE, // size of eeprom in bytes
                MCU_NUM_PINS, // number of pins
                MCU_NUM_INTS, // number of interrupts
                new ReprogrammableCodeSegment.Factory(MCU_FLASH_SIZE, MCU_PAGE_SIZE),
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
        return new ATTiny85(id, sim, cd, p);
        //public Microcontroller newMicrocontroller(int id, ClockDomain cd, Program p) {   // for old avrora jar
        //   return new ATTiny85(id, cd, p);
        }

    }

     public ATTiny85(int id, Simulation sim, ClockDomain cd, Program p) {//for current avrora source
    //public ATTiny85(int id, ClockDomain cd, Program p) {// for old avrora jar
        super(cd, props, new FiniteStateMachine(cd.getMainClock(), MODE_ACTIVE, idleModeNames, transitionTimeMatrix));
        simulator = sim.createSimulator(id, LegacyInterpreter.FACTORY, this, p);//for current avrora source
        //simulator = new Simulator(id, LegacyInterpreter.FACTORY, this, p);// for old avrora jar
        interpreter = (AtmelInterpreter)simulator.getInterpreter();
        SMCR_reg = getIOReg("MCUCR");
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
    protected FlagRegister TIFR_reg,MTIFR_reg;
    protected MaskRegister TIMSK_reg,MTIMSK_reg;
    
    protected void installDevices() {
/*
        TIFR_reg = buildInterruptRange(false, "TIMSK", "TIFR", 12, 6);
        TIMSK_reg = (MaskRegister)getIOReg("TIMSK");
        int[] TIFR_mapping = {-1 , 4, 10, 11, 12, 5, 6, -1};
        MTIFR_reg = new FlagRegister(interpreter, TIFR_mapping);
        MTIMSK_reg = new MaskRegister(interpreter, TIFR_mapping);
        installIOReg("TIMSK", MTIMSK_reg);
        installIOReg("TIFR", MTIFR_reg);
        */
/*
        TIFR1_reg = buildInterruptRange(false, "TIMSK1", "TIFR1", 16, 4);
        TIMSK1_reg = (MaskRegister)getIOReg("TIMSK1");
        int[] TIFR1_mapping = {16, 14, 15, -1, -1, 13, -1, -1};
        MTIFR1_reg = new FlagRegister(interpreter, TIFR1_mapping);
        MTIMSK1_reg = new MaskRegister(interpreter, TIFR1_mapping);
        installIOReg("TIMSK1", MTIMSK1_reg);
        installIOReg("TIFR1", MTIFR1_reg);
*/
    //    addDevice(new Timer0());
   //     addDevice(new Timer1(2));

        buildPort('B');

        addDevice(new EEPROM(properties.eeprom_size, this));
    //    addDevice(new USART("0", this));
    
        /* ADC4 is a temperature sensor.  Channels Vcc, Vbg and Gnd are always appended.
         * An application can connectADCInput() on those channels to change the voltage
         * dynamically, or change the default level using e.g.
         * ADC adc = (ADC)((AtmelMicrocontroller)mcu).getDevice("adc");
         * adc.VCC_LEVEL = 3.3f;
         * adc.GND_LEVEL = 0.1f;
         * adc.VBG_LEVEL = 1.1f;
         */
        addDevice(new ADC(this, 5));
    }


    // permutation of sleep mode bits in the register (high order bits first)
        private static final int[] SMCR_sm_perm = { 2, 1 };

    protected int getSleepMode() {
        byte value = SMCR_reg.read();
        boolean sleepEnable = Arithmetic.getBit(value, 0);

        if ( sleepEnable )
            return Arithmetic.getBitField(value, SMCR_sm_perm) + 1;
        else
            return MODE_IDLE;
    }

}
