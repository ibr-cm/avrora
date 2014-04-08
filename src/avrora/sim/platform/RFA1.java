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

package avrora.sim.platform;

import avrora.core.Program;
import avrora.sim.Simulator;
import avrora.sim.Simulation;
import avrora.sim.clock.ClockDomain;
import avrora.sim.mcu.*;
import avrora.sim.platform.devices.LightSensor;
import avrora.sim.platform.sensors.SensorBoard;
//import avrora.sim.platform.sensors.AccelSensor;
//import avrora.sim.platform.sensors.AccelSensorPower;
import avrora.sim.radio.*;
import cck.text.Terminal;
import avrora.sim.types.SingleSimulation;

/**
 * The <code>RFA1</code> class is an implementation of the <code>Platform</code> interface that represents
 * both a specific microcontroller and the devices connected to it. This implementation uses the standalone
 * ATMega128RFA1 microcontroller with no peripherals.
 *
 * @author Ben L. Titzer, Daniel Lee, David Kopf
 */
public class RFA1 extends Platform {

    protected static final int MAIN_HZ = 8000000;
    protected static final Simulation mysim = null;
    public static class Factory implements PlatformFactory {

        /**
         * The <code>newPlatform()</code> method is a factory method used to create new instances of the
         * <code>RFA1</code> class.
         * @param id the integer ID of the node
         * @param sim the simulation
         * @param p the program to load onto the node @return a new instance of the <code>Mica2</code> platform
         */
        public Platform newPlatform(int id, Simulation sim, Program p) {
            ClockDomain cd = new ClockDomain(MAIN_HZ);
            cd.newClock("external", 32768);

            return new RFA1(new ATMega128RFA1(id, sim, cd, p));
        }
    }

    protected final Simulator sim;
    protected ATmega128RFA1Radio radio;
 //   protected RADIO radioDevice;
 //   protected CC2420Radio radio;
 //      protected ATMega128RFA1Radio radio;
  //  protected SensorBoard sensorboard;
 //   protected ExternalFlash externalFlash;
 //   protected LightSensor lightSensor;
 //   protected AccelSensor accelXSensor;
 //   protected AccelSensor accelYSensor;
    protected LED.LEDGroup ledGroup;

    private RFA1(Microcontroller m) {
        super(m);
        sim = m.getSimulator();
        addDevices();
    }

    /**
     * The <code>addDevices()</code> method is used to add the external (off-chip) devices to the
     * platform. The bare platform has no external devices.
     */
    protected void addDevices() {
        LED red = new LED(sim, Terminal.COLOR_RED, "Red");
        LED green = new LED(sim, Terminal.COLOR_GREEN, "Green");
        LED blue = new LED(sim, Terminal.COLOR_BLUE, "Blue");

        ledGroup = new LED.LEDGroup(sim, new LED[] { red, green, blue });
        addDevice("leds", ledGroup);

        AtmelMicrocontroller amcu = (AtmelMicrocontroller)mcu;

        mcu.getPin("PD5").connectOutput(blue);
        mcu.getPin("PD6").connectOutput(green);
        mcu.getPin("PD7").connectOutput(red);

        // install the internal 128RFA1 radio
      //               CC2420Radio radio = new CC2420Radio(mcu, MAIN_HZ * 2);
      ATmega128RFA1Radio radio = new ATmega128RFA1Radio(mcu, MAIN_HZ * 2);
  //          ATMega128RFA1Radio radio = new ATMega128RFA1Radio(mcu, MAIN_HZ * 2);
  //      mcu.getPin("PA5").connectOutput(radio.VREN_pin);
  //      mcu.getPin("PA6").connectOutput(radio.RSTN_pin);
     //   ADC adc = (ADC)amcu.getDevice("adc");
     //   adc.connectADCInput(radio.adcInterface, 0);
        RADIO radioDevice = (RADIO)amcu.getDevice("radio");
    //    SPI spi = (SPI)amcu.getDevice("spi");
    //    spi.connect(radio.spiInterface);
        radioDevice.connect(radio, radioDevice);
        addDevice("radio", radio);
    //    radio.FIFOP_interrupt = mcu.getProperties().getInterrupt("INT6");
        // install the input capture pin.
        Timer16Bit timer1 = (Timer16Bit)amcu.getDevice("timer1");
     //   radio.setSFDView(timer1.getInputCapturePin());

        // sensor board
 //       sensorboard = new SensorBoard(sim);
        // external flash uses PA3
  //      externalFlash = new ExternalFlash(mcu, 2048, 264);
        // acceleration sensors
//        AccelSensorPower asp = new AccelSensorPower(amcu, "PC4");
 //       accelXSensor = new AccelSensor(amcu, 3, asp);
//        addDevice("accelx-sensor", accelXSensor);
 //       accelYSensor = new AccelSensor(amcu, 4, asp);
//        addDevice("accely-sensor", accelYSensor);
        // light sensor
   //     lightSensor = new LightSensor(amcu, 1, "PE5");
  //      addDevice("light-sensor", lightSensor);
    }

}
