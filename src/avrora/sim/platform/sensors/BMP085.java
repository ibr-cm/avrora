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
package avrora.sim.platform.sensors;

import avrora.sim.mcu.TWIData;
import avrora.sim.mcu.TWIDevice;
import java.math.BigInteger;

/**
 *
 * @author S. Willenborg
 */
public class BMP085 extends Sensor implements TWIDevice {

    private final byte ADDRESS = (byte) 0xEE;
    private boolean active = false;
    private int writecount = 0;
    private byte reg = 0;
    final short AC1 = 408;
    final short AC2 = -72;
    final short AC3 = -14383;
    final short AC4 = 32741;
    final short AC5 = 32757;
    final short AC6 = 23153;
    final short B1 = 6190;
    final short B2 = 4;
    final short MB = -32768;
    final short MC = -8711;
    final short MD = 2868;

    private final short[] koeffz = {AC1, AC2, AC3, AC4, AC5, AC6, B1, B2, MB, MC, MD};
    int measure_data = 0x00;
    int measure_mode = 0x00;
    double cur_temp = 21.2;
    int cur_pressure = 99900;
    public BMP085() {
       
    }

    public final int convertTemperature(double temp) {
        int T = (int) (temp * 10);
        int converted = (int) (((Math.sqrt(MD * MD + (((T + 1) << 5) - (2 << 4)) * MD - (MC << 13) + (((T) * (T + 1)) << 8) + 64) - MD + ((T + 1) << 4) - 8) / AC5) * (1 << 14) + AC6);
        return converted;
    }

    public final int convertPressure(double temp, int pressure, int mode) {
        int x1 = (convertTemperature(temp) - AC6) * AC5 >> 15;

        int x2 = (MC * 2048) / (x1 + MD);
        int b5 = x1 + x2;

        int b6 = b5 - 4000;
        int b3 = ((((AC1) * 4 + (((B2 * ((b6 * b6) >> 12)) >> 11) + ((AC2 * b6) >> 11))) << mode) + 2) >> 2;
        int b4 = (AC4 * ((((((AC3 * b6) / 8192) + ((B1 * ((b6 * b6) / 4096)) / 65536)) + 2) / 4) + 32768)) / 32768;

        int pt = pressure;
        int p = pt;
        int bigger;
        if (pt <= 48361) {
            bigger = -1;
        } else if (pt <= 48384) {
            bigger = 1;
        } else if (pt <= 48512) {
            bigger = -1;
        } else if (pt <= 48640) {
            bigger = 1;
        } else if (pt <= 48673) {
            bigger = -1;
        } else if (pt <= 110080) {
            bigger = 1;
        } else if (pt <= 110120) {
            bigger = -1;
        } else if (pt <= 110336) {
            bigger = 1;
        } else if (pt <= 110476) {
            bigger = -1;
        } else if (pt <= 110593) {
            bigger = 1;
        } else if (pt <= 110833) {
            bigger = -1;
        } else if (pt <= 110850) {
            bigger = 1;
        } else {
            bigger = -1;
        }
        while (pt != (p + (((((((p / 256) * (p / 256) * 1519 * 2) / 65536) + ((-7357 * p) / 65536) + 3791))) >> 4))) {
            p += bigger;
        }
        int b7 = BigInteger.valueOf(p).multiply(BigInteger.valueOf(b4)).divide(BigInteger.valueOf(2)).intValue();

        return (b7 / (50000 >> mode) + b3 + 1) << (8 - mode);

    }

    private void refresh_values(byte measure_mode) {
        this.measure_mode = measure_mode;
        switch (measure_mode & 0xff) {
            case 0x2E: // Temperature
                measure_data = convertTemperature(cur_temp);
                break;
            case 0x34: // Pressure (osrs =0)
                measure_data = convertPressure(cur_temp, cur_pressure, 0);
                break;
            case 0x74: // Pressure (osrs =1)
                measure_data = convertPressure(cur_temp, cur_pressure, 1);
                break;
            case 0xB4: // Pressure (osrs =2)
                measure_data = convertPressure(cur_temp, cur_pressure, 2);
                break;
            case 0xF4: // Pressure (osrs =3)
                measure_data = convertPressure(cur_temp, cur_pressure, 3);
                break;
        }
    }

    public final byte getKoeffz(byte addr) {
        byte index = (byte) ((addr & 0xff) - 0xaa);
        return (byte) (((koeffz[index / 2] >> ((1 - (index % 2)) * 8))) & 0xff);
    }

    // eeprom 0xaa -0xbf
    // temp or preasure value: 0xf6(msb, 0xf7 lsb, 0xf8 xlsb
    @Override
    public Boolean writeByte(byte data, boolean ack) {
        if (!active) {
            return null;
        }
        if (writecount == 0) {
            reg = data;
        } else if (writecount == 1) {
            if ((reg & 0xff) == 0xf4) {
                refresh_values(data);
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

        byte result = (byte) 0x00;
        if (isEEPROM(reg)) {
            result = getKoeffz(reg);
        } else if (true) {
            int i = ((reg & 0xFF) - 0xF6);

            if ((measure_mode & 0xff) == 0x2E && i < 2) {
                result = (byte) (measure_data >> ((1 - i) * 8) & 0xff);
            } else if (i < 3) {
                result = (byte) (measure_data >> ((2 - i) * 8) & 0xff);
            }
        }
        reg++;
        return new TWIData(result, ack);
    }

    @Override
    public Boolean start(byte address, boolean rep, boolean ack) {
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

    private boolean isEEPROM(byte reg) {
        return (reg & 0xFF) >= 0xAA && (reg & 0xFF) <= 0xBF;
    }

}
