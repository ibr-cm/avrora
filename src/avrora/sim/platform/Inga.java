/**
 * Copyright (c) 2013-2014, TU Braunschweig
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

package avrora.sim.platform;

import avrora.core.Program;
import avrora.sim.Simulation;
import avrora.sim.Simulator;
import avrora.sim.clock.ClockDomain;
import avrora.sim.mcu.ATMega1284p;
import avrora.sim.mcu.AtmelMicrocontroller;
import avrora.sim.mcu.Microcontroller;
import avrora.sim.mcu.SPI;
import avrora.sim.mcu.SPIDevice;
import avrora.sim.mcu.TWI;
import avrora.sim.platform.sensors.ADXL345;
import avrora.sim.platform.sensors.AT45DB;
import avrora.sim.platform.sensors.BMP085;
import avrora.sim.platform.sensors.Button;
import avrora.sim.platform.sensors.L3G4200D;
import avrora.sim.radio.AT86RF231Radio;
import avrora.sim.radio.ATmega128RFA1Radio;
import avrora.sim.util.LCX138;
import cck.text.Terminal;

/**
 * The <code>Inga</code> class is an implementation of the <code>Platform</code>
 * interface that represents
 * both a specific microcontroller and the devices connected to it. This
 * implements the Inga
 * configuration, an ATMega1284p with SPI connection to an AT86RF230 radio, and
 * some other peripherals.
 *
 * @author Ben L. Titzer
 * @author Daniel Lee
 * @author David Kopf
 * @author S. Willenborg
 */
public class Inga extends Platform {

    protected static final int MAIN_HZ = 8000000;
    protected static final Simulation mysim = null;
    public static class Factory implements PlatformFactory {

        /**
         * The <code>newPlatform()</code> method is a factory method used to create new instances of the
         * <code>Raven</code> class.
         * @param id the integer ID of the node
         * @param sim the simulation
         * @param p the program to load onto the node @return a new instance of the <code>Mica2</code> platform
         * @return new Inga platform
         */
        @Override
        public Platform newPlatform(int id, Simulation sim, Program p) {
            ClockDomain cd = new ClockDomain(MAIN_HZ);
            cd.newClock("external", 32768);

            return new Inga(new ATMega1284p(id, sim, cd, p));
        }
    }

    protected final Simulator sim;
    protected ATmega128RFA1Radio radio;
    protected LED.LEDGroup ledGroup;

    private Inga(Microcontroller m) {
        super(m);
        sim = m.getSimulator();
        addDevices();
    }

    /**
     * The <code>addDevices()</code> method is used to add
     * the external (off-chip) devices to the platform.
     */
    private void addDevices() {
        LED red = new LED(sim, Terminal.COLOR_RED, "Red");
        LED green = new LED(sim, Terminal.COLOR_GREEN, "Green");
        LED blue = new LED(sim, Terminal.COLOR_BLUE, "Blue");


        ledGroup = new LED.LEDGroup(sim, new LED[]{red, green, blue});
        addDevice("leds", ledGroup);

        //AtmelMicrocontroller amcu = (AtmelMicrocontroller)mcu;
        mcu.getPin("PD5").connectOutput(blue);
        mcu.getPin("PD6").connectOutput(green);
        mcu.getPin("PD7").connectOutput(red);

        // install the new AT86RF230 radio. Actually an AT86RF231.
        AT86RF231Radio radio = new AT86RF231Radio(mcu, MAIN_HZ * 2);
 //     mcu.getPin("PB7").connectOutput(radio.SCLK_pin);
        mcu.getPin(3).connectOutput(radio.SCLK_pin); //PB7
        mcu.getPin(1).connectOutput(radio.MOSI_pin); //PB5
        mcu.getPin(2).connectInput(radio.MISO_pin);  //PB6
        mcu.getPin(41).connectOutput(radio.RSTN_pin); //PB1
        mcu.getPin(43).connectOutput(radio.SLPTR_pin);//PB3
        mcu.getPin(44).connectOutput(radio.CS_pin);   //PB4

        SPI spi = (SPI) ((AtmelMicrocontroller) mcu).getDevice("spi");
        SPIDevice spi2 = (SPIDevice) ((AtmelMicrocontroller) mcu).getDevice("usart1");
        spi.connect(radio.spiInterface);
        addDevice("radio", radio);
        radio.RF231_interrupt = mcu.getProperties().getInterrupt("TIMER1 CAPT");
        TWI twi = (TWI) ((AtmelMicrocontroller) mcu).getDevice("twi");

        BMP085 pressure = new BMP085();
        twi.connect(pressure);
        addDevice("bmp085", pressure);

        L3G4200D gyroscope = new L3G4200D();
        twi.connect(gyroscope);
        addDevice("l3g4200d", gyroscope);

        LCX138 decoder = new LCX138();
        mcu.getPin(32).connectOutput(decoder.A0);
        mcu.getPin(31).connectOutput(decoder.A1);
        mcu.getPin(30).connectOutput(decoder.A2);
        decoder.E1.write(false);
        decoder.E2.write(false);
        decoder.E3.write(true);
        addDevice("lcx138", decoder);

        AT45DB flash = new AT45DB();
        flash.connectCS(decoder.O1);
        spi2.connect(flash);
        addDevice("at45db", flash);

        ADXL345 accelerometer = new ADXL345();
        accelerometer.connectCS(decoder.O2);
        spi2.connect(accelerometer);
        addDevice("adxl345", accelerometer);
        
        Button button = new Button();
        mcu.getPin(42).connectInput(button.output);
        addDevice("button", button);
    }

}
