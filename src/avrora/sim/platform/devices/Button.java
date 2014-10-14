/**
 * Copyright (c) 2014, TU Braunschweig
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

package avrora.sim.platform.devices;

import avrora.sim.mcu.Microcontroller;
import avrora.sim.platform.sensors.PushSensorSource;
import avrora.sim.platform.sensors.Sensor;
import avrora.sim.platform.sensors.SensorSource;
import java.util.LinkedList;
import java.util.List;

/**
 * A Simple button that can be connected to any microcontroller input pin.
 *
 * @author Enrico Joerns
 */
public class Button extends Sensor implements PushSensorSource.SensorSourceListener {

    private final boolean activeLow;
    private SensorSource source;
    private boolean pinHigh = false;
    private List<Microcontroller.Pin.InputListener> listeners = new LinkedList<>();

    private final Channel[] channels = new Channel[]{
        new Channel("Button", 0.0, 1.0, 0.0)
    };
    
    /**
     * Creates button.
     * 
     * @param activeLow If set to true, button press (input > 0) pulls the outPushSensorSource.SensorSourceListenerput pin low,
     * if set to false, button press pulls the output pin hight.
     */
    public Button(boolean activeLow) {
        this.activeLow = activeLow;
        source = new PushSensorSource();
    }

    /**
     * Creates active low button.
     */
    public Button() {
        this(true);
    }

    @Override
    public Channel[] getChannels() {
        return channels;
    }

    @Override
    public void setSensorSource(SensorSource src) {
        source = src;
        if (src instanceof PushSensorSource) {
            ((PushSensorSource) source).addDataUpdateListener(this);
        }
    }

    /* button output pin */
    public Microcontroller.Pin.Input output = new Microcontroller.Pin.Input() {

        @Override
        public boolean read() {
            return pinHigh;
        }

        @Override
        public void registerListener(Microcontroller.Pin.InputListener listener) {
            listeners.add(listener);
        }

        @Override
        public void unregisterListener(Microcontroller.Pin.InputListener listener) {
            if (listeners.contains(listener)) {
                listeners.remove(listener);
            }
        }
    };

    /* Forward calls by sensor source to registered controller input pins */
    @Override
    public void onNewData(int idx, double value) {
        pinHigh = activeLow ? (value == 0) : (value != 0);
        for (Microcontroller.Pin.InputListener l : listeners) {
            /* map double data input to boolean where 0.0 is false
             * and everything else is true */
            l.onInputChanged(output, pinHigh);
        }
    }

}
