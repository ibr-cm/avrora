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
import avrora.sim.mcu.TWIData;
import avrora.sim.mcu.TWIDevice;
import avrora.sim.state.RegisterUtil;
import avrora.sim.state.RegisterView;

/**
 *
 * @author S. Willenborg
 */
public class L3G4200D extends Sensor implements TWIDevice {

    private final byte ADDRESS = (byte) 0xd2;
    private boolean active = false;
    private int writecount = 0;
    private byte reg = 0;

    private class CTRLReg1 extends RWRegister {

    }

    private class CTRLReg2 extends RWRegister {

    }

    private class CTRLReg3 extends RWRegister {

    }
    final int L3G4200D_DPSDIV_250G = 35;
    final int L3G4200D_DPSDIV_500G = 70;
    final int L3G4200D_DPSDIV_2000G = 280;
    private class CTRLReg4 extends RWRegister {
        final int DPS0 = 4;
        final int DPS1 = 5;
        final RegisterView _dps = RegisterUtil.bitRangeView(this, DPS0, DPS1);

    }

    private class CTRLReg5 extends RWRegister {

    }

    double x = 1;
    double y = 1.2;
    double z = 1.1;
    int temp = 29;

    CTRLReg1 ctrl_reg1 = new CTRLReg1();
    CTRLReg2 ctrl_reg2 = new CTRLReg2();
    CTRLReg3 ctrl_reg3 = new CTRLReg3();
    CTRLReg4 ctrl_reg4 = new CTRLReg4();
    CTRLReg5 ctrl_reg5 = new CTRLReg5();

    public int dps_to_raw(double dpsval) {
        int dpsdiv = L3G4200D_DPSDIV_250G;
        if (ctrl_reg4._dps.getValue() == 1) {
            dpsdiv = L3G4200D_DPSDIV_500G;
        } else if (ctrl_reg4._dps.getValue() == 2) {
            dpsdiv = L3G4200D_DPSDIV_2000G;
        }
        return (int) (((dpsval + .1) * 4000) / dpsdiv);
    }

    @Override
    public Boolean writeByte(byte data, boolean ack) {
        if (!active) {
            return null;
        }

        if (writecount == 0) {
            reg = (byte) (data & 0x7F);
        } else {
            switch (reg & 0xff) {
                case 0x20:
                    ctrl_reg1.write(data);
                    break;
                case 0x21:
                    ctrl_reg2.write(data);
                    break;
                case 0x22:
                    ctrl_reg3.write(data);
                    break;
                case 0x23:
                    ctrl_reg4.write(data);
                    break;
                case 0x24:
                    ctrl_reg5.write(data);
                    break;

            }
        }
        writecount++;
        return ack;

    }

    @Override
    public TWIData readByte(boolean ack) {
        if (!active) {
            return null;
        }
        byte result = 0x00;
        switch (reg & 0xff) {
            case 0x0f:
                result = ADDRESS | 1;
                break;
            case 0x20:
                result = ctrl_reg1.read();
                break;
            case 0x21:
                result = ctrl_reg2.read();
                break;
            case 0x22:
                result = ctrl_reg3.read();
                break;
            case 0x23:
                result = ctrl_reg4.read();
                break;
            case 0x24:
                result = ctrl_reg5.read();
                break;
            case 0x26:
                result = (byte) (25 - temp);
                break;
            case 0x28:
            case 0x29:
                result = (byte) (dps_to_raw(x) >> (8 * (reg % 2)));
                break;
            case 0x2A:
            case 0x2B:
                result = (byte) (dps_to_raw(y) >> (8 * (reg % 2)));
                break;
            case 0x2C:
            case 0x2D:
                result = (byte) (dps_to_raw(z) >> (8 * (reg % 2)));
                break;

        }
        
        reg++;
        return new TWIData(result, ack);
    }

    @Override
    public Boolean start(byte address, boolean write, boolean rep, boolean ack) {
        active = (address == ADDRESS);
        if (!active) {
            return null;
        }
        writecount = 0;
        return ack;
    }

    @Override
    public Boolean stop() {
        if (!active) {
            return null;
        }
        return true;
    }

}
