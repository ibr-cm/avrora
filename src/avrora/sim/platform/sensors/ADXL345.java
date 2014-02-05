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

package avrora.sim.platform.sensors;

import avrora.sim.RWRegister;
import avrora.sim.mcu.Microcontroller;
import avrora.sim.mcu.Microcontroller.Pin.InputListener;
import avrora.sim.mcu.SPI;
import avrora.sim.mcu.SPIDevice;
import avrora.sim.state.RegisterUtil;
import avrora.sim.state.RegisterView;

/**
 *
 * @author S. Willenborg
 */
public class ADXL345 implements SPIDevice, InputListener {

    final byte DEVICE_ID = (byte) 0xe5;
    Microcontroller.Pin.Input CS = null;
    int reg = -1;
    boolean is_read = true;

    byte offset_x = 0;
    byte offset_y = 0;
    byte offset_z = 0;

    int x = 0;
    int y = 0;
    int z = 1000;

    DataFormat DataFormat_reg = new DataFormat();
    PowerCtl PowerCtl_reg = new PowerCtl();
    BWRate BWRate_reg = new BWRate();
    private class BWRate extends RWRegister {

        public BWRate() {
            value = 0x0A;
        }

    }
    private class PowerCtl extends RWRegister {

    }
    private class DataFormat extends RWRegister {

        final int RANGE0 = 0;
        final int RANGE1 = 1;
        final int JUSTIFY = 2;
        final int FULL_RES = 3;
        final int INT_INVERT = 5;
        final int SPI = 6;
        final int SELF_TEST = 7;
        final RegisterView _range = RegisterUtil.bitRangeView(this, RANGE0, RANGE1);

        final RegisterView _justify = RegisterUtil.bitView(this, JUSTIFY);
        final RegisterView _full_res = RegisterUtil.bitView(this, FULL_RES);
        final RegisterView _int_invert = RegisterUtil.bitView(this, INT_INVERT);
        final RegisterView _spi = RegisterUtil.bitView(this, SPI);
        final RegisterView _self_test = RegisterUtil.bitView(this, SELF_TEST);


    }
    public int mg_to_raw(int mg) {
        // @TODO: implement correct conversion using the FULL_RESOLUTION and RAGE values
        return (int) (mg / 3.9);
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
            case 0x31:
                return DataFormat_reg.read();
            case 0x32:
            case 0x33:
                return (byte) (mg_to_raw(x) >> (8 * ( (reg % 2))));
            case 0x34:
            case 0x35:
                return (byte) (mg_to_raw(y) >> (8 * ( (reg % 2))));
            case 0x36:
            case 0x37:
                return (byte) (mg_to_raw(z) >> (8 * ( (reg % 2))));
            default:
                System.out.printf("acc read %02x (%02x)\n", 0xff & reg, 0x00);
        }
        return 0x00;
    }

    private void write(byte reg, byte data) {
        
        switch (reg & 0xff) {
            case 0x1e:
                offset_x = data;
                break;
            case 0x1f:
                offset_y = data;
                break;
            case 0x20:
                offset_z = data;
                break;
            case 0x2c:
                BWRate_reg.write(data);
                break;
            case 0x2d:
                PowerCtl_reg.write(data);
                break;
            case 0x31:
                DataFormat_reg.write(data);
                break;
            default:
                System.out.printf("acc write %02x %02x\n", 0xff & reg, 0xff & data);
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
