/**
 * Copyright (c) 2014 TU Braunschweig
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

package avrora.sim.platform.sensors;

import java.util.LinkedList;
import java.util.List;

/**
 * A simple user-settable Sensor Source.
 *
 * @author Enrico Jorns
 */
public class PushSensorSource implements SensorSource {
    
    List<SensorSourceListener> listeners = new LinkedList<>();
    double[] data;

    /**
     * The <code>read()</code> method is called by the simulation
     * when a new read of the sensor is requested or needed.
     *
     * @return Sensor value, or constant -1.0 to indicate missing data
     */
    @Override
    public final double read(int idx) {
//        System.out.printf("PushSensorSource.read(%d): ", idx);
        if (data == null) {
//            System.out.println("" + -1.0);
            return -1.0;
        }
//        System.out.println("" + data[idx]);
        return data[idx];
    }

    /**
     * The <code>setSensorSource()</code> method allows the user
     * to set custom sensor data for the sensor.
     *
     * @param data
     */
    public final void setSensorData(double[] data) {
//        System.out.print("PushSensorSource.setSensorData(");
        for(double d : data) {
            System.out.print(String.valueOf(d) + ",");
        }
//        System.out.println(")");
        this.data = data;
        if (!listeners.isEmpty()) {
            for(int idx = 0; idx < data.length; idx++) {
                notifyListeners(idx, data[idx]);
            }
        }
    }
    
    /**
     * 
     * @param idx
     * @param data 
     */
    public final void setSensorData(int idx, double data) {
        this.data[idx] = data;
        notifyListeners(idx, this.data[idx]);
    }

    /**
     * 
     * @param idx
     * @param value 
     */
    protected void notifyListeners(int idx, double value) {
        for (SensorSourceListener l : listeners) {
            l.onNewData(idx, value);
        }
    }

    /**
     * Adds an update listener.
     *
     * A listener could be used for buttons or for generating interrupt events.
     *
     * @param l
     */
    public void addDataUpdateListener(SensorSourceListener l) {
        if (!listeners.contains(l)) {
            listeners.add(l);
        }
    }

    public interface SensorSourceListener {

        void onNewData(int idx, double value);
    }

}
