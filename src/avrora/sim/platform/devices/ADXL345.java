/**
 * Copyright (c) 2013-2014, TU Braunschweig. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the University of California, Los Angeles nor the names
 * of its contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package avrora.sim.platform.devices;

import avrora.sim.RWRegister;
import avrora.sim.mcu.Microcontroller;
import avrora.sim.mcu.Microcontroller.Pin.InputListener;
import avrora.sim.mcu.SPI;
import avrora.sim.mcu.SPIDevice;
import avrora.sim.platform.sensors.Sensor;
import avrora.sim.platform.sensors.SensorSource;
import avrora.sim.state.RegisterUtil;
import avrora.sim.state.RegisterView;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Incomplete implementation of the ADXL345 3 axis digital accelerometer.
 * <p>
 * Currently only spi mode is supported.
 * <p>
 * Note: input values are in [mg]
 *
 * @author S. Willenborg
 */
public class ADXL345 extends Sensor implements SPIDevice, InputListener {

    private static final Logger logger = LoggerFactory.getLogger(ADXL345.class);

    /* source channel for x acceleration */
    private static final int X = 0;
    /* source channel for y acceleration */
    private static final int Y = 1;
    /* source channel for z acceleration */
    private static final int Z = 2;

    private SensorSource source;
    static final byte DEVICE_ID = (byte) 0xe5;
    Microcontroller.Pin.Input CS = null;
    int reg = -1;
    boolean is_read = true;
    // says if device is in full resolution mode (see dataseet)
    private boolean full_resolution = false;
    // scale factor to convert LSB to mg
    private double scale_factor = 3.9;

    byte offset_x = 0;
    byte offset_y = 0;
    byte offset_z = 0;

    final Channel[] channels = new Channel[]{
        new Channel("x", "mg", -2000.0, 2000.0, 0.0),
        new Channel("y", "mg", -2000.0, 2000.0, 0.0),
        new Channel("z", "mg", -2000.0, 2000.0, 0.0)
    };

    @Override
    public Channel[] getChannels() {
        return channels;
    }

    @Override
    public void setSensorSource(SensorSource src) {
        source = src;
    }

    // registers
    DataFormat DataFormat_reg = new DataFormat();
    PowerCtl PowerCtl_reg = new PowerCtl();
    BWRate BWRate_reg = new BWRate();

    private class BWRate extends RWRegister {

        static final int RATE0 = 0;
        static final int RATE1 = 1;
        static final int RATE2 = 2;
        static final int RATE3 = 3;
        static final int LOW_POWER = 4;

        public BWRate() {
            value = 0x0A; // Dafaults to 100Hz
        }
    }

    private class PowerCtl extends RWRegister {

        static final int WAKEUP0 = 0;
        static final int WAKEUP1 = 1;
        static final int SLEEP = 2;
        static final int MEASURE = 3;
        static final int AUTO_SLEEP = 4;
        static final int LINK = 5;

    }

    private class DataFormat extends RWRegister {

        static final int RANGE0 = 0;
        static final int RANGE1 = 1;
        static final int JUSTIFY = 2;
        static final int FULL_RES = 3;
        static final int INT_INVERT = 5;
        static final int SPI = 6;
        static final int SELF_TEST = 7;

        final RegisterView _range = RegisterUtil.bitRangeView(this, RANGE0, RANGE1);

        final RegisterView _justify = RegisterUtil.bitView(this, JUSTIFY);
        final RegisterView _full_res = RegisterUtil.bitView(this, FULL_RES);
        final RegisterView _int_invert = RegisterUtil.bitView(this, INT_INVERT);
        final RegisterView _spi = RegisterUtil.bitView(this, SPI);
        final RegisterView _self_test = RegisterUtil.bitView(this, SELF_TEST);

    }

    private int mg_to_raw(int mg) {
        return (int) Math.round(mg / scale_factor);
    }

    public void connectCS(Microcontroller.Pin.Input cs) {
        CS = cs;
        if (cs != null) {
            cs.registerListener(this);
        }
    }

    private byte read(byte reg) {

        switch (reg & 0xff) {
            case 0x00:
                return DEVICE_ID;
            case 0x1e:
                return offset_x;
            case 0x1f:
                return offset_y;
            case 0x20:
                return offset_z;
            case 0x2c:
                return BWRate_reg.read();
            case 0x2d:
                return PowerCtl_reg.read();
            case 0x31: // DATA_FORMAT
                return DataFormat_reg.read();
            case 0x32:
            case 0x33:
                return (byte) (mg_to_raw((int) source.read(X)) >> (8 * (reg % 2)));
            case 0x34:
            case 0x35:
                return (byte) (mg_to_raw((int) source.read(Y)) >> (8 * (reg % 2)));
            case 0x36:
            case 0x37:
                return (byte) (mg_to_raw((int) source.read(Z)) >> (8 * (reg % 2)));
            default:
                logger.warn("Read access to {} not supported", reg & 0xFF);
        }
        return 0x00;
    }

    private void write(byte reg, byte data) {

        switch (reg & 0xff) {
            case 0x1e: // OFSX
                offset_x = data;
                break;
            case 0x1f: // OFSY
                offset_y = data;
                break;
            case 0x20: // OFSZ
                offset_z = data;
                break;
            case 0x2c:
                BWRate_reg.write(data);
                break;
            case 0x2d:
                PowerCtl_reg.write(data);
                break;
            case 0x31: // DATA_FORMAT
                setDataFormat(data);
                DataFormat_reg.write(data);
                break;
            default:
                System.out.printf("acc write %02x %02x\n", 0xff & reg, 0xff & data);
        }
    }

    private void setDataFormat(byte data) {
        // evaluate FULL_RES bit
        full_resolution = (data & (1 << DataFormat.FULL_RES)) != 0;
        // evaluate g range
        double lbound = 0.0, ubound = 0.0;
        switch (data & 0x03) {
            case 0: // +/- 2g
                lbound = -2000.0;
                ubound = 2000.0;
                if (!full_resolution) {
                    scale_factor = 3.9;
                }
                break;
            case 1: // +/- 4g
                lbound = -4000.0;
                ubound = 4000.0;
                if (!full_resolution) {
                    scale_factor = 7.8;
                }
                break;
            case 2: // +/- 8g
                lbound = -8000.0;
                ubound = 8000.0;
                if (!full_resolution) {
                    scale_factor = 15.6;
                }
                break;
            case 3: // +/- 16g
                lbound = -16000.0;
                ubound = 16000.0;
                if (!full_resolution) {
                    scale_factor = 31.2;
                }
                break;
        }
        for (Channel c : channels) {
            c.lbound = lbound;
            c.ubound = ubound;
        }
    }

    @Override
    public SPI.Frame exchange(SPI.Frame frame) {
        if (CS != null && CS.read()) {
            return SPI.newFrame((byte) 0x00);
        }

        byte result = 0x00;
        if (reg == -1) {
            reg = frame.data & 0x3F;

            is_read = (frame.data & 0x80) != 0;

        } else {
            if (is_read) {
                result = read((byte) reg);
//                System.out.printf("ADXL345:read(%x) : %d%n", reg, result & 0xFF);

            } else {
                write((byte) reg, frame.data);
            }
            reg++;
        }

        return SPI.newFrame(result);
    }

    @Override
    public void connect(SPIDevice d) {
        throw new UnsupportedOperationException("Not supported.");
    }

    @Override
    public void onInputChanged(Microcontroller.Pin.Input input, boolean newValue) {
        if (!newValue) {
            reg = -1;
        }
    }
}
