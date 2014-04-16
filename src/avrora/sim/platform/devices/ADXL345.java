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
import avrora.sim.Simulator;
import avrora.sim.clock.Clock;
import avrora.sim.mcu.Microcontroller;
import avrora.sim.mcu.Microcontroller.Pin.InputListener;
import avrora.sim.mcu.SPI;
import avrora.sim.mcu.SPIDevice;
import avrora.sim.platform.sensors.NullSensorSource;
import avrora.sim.platform.sensors.Sensor;
import avrora.sim.platform.sensors.SensorSource;
import avrora.sim.state.RegisterUtil;
import avrora.sim.state.RegisterView;
import java.util.LinkedList;
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
 * @author Enrico Jorns
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
    private int reg = -1;
    private boolean is_read = true;
    
    // says if device is in full resolution mode (see dataseet)
    private boolean full_resolution = false;
    // scale factor to convert LSB to mg
    private double scale_factor = 3.9;
    // sample rate in Hz
    private double sampleRate = -1;
    // if fifo should be bypassed
    private boolean bypass = true;
    private final Clock clock;


    final Channel[] channels = new Channel[]{
        new Channel("x", "mg", -2000.0, 2000.0, 0.0),
        new Channel("y", "mg", -2000.0, 2000.0, 0.0),
        new Channel("z", "mg", -2000.0, 2000.0, 0.0)
    };
    
    // 32 elements fixed-size fifo
    private final LimitedQueue<Integer[]> fifo = new LimitedQueue<>(32);
    
    public class LimitedQueue<E> extends LinkedList<E> {

        private final int limit;
        private boolean drop;

        public LimitedQueue(int limit) {
            this.limit = limit;
        }

        /**
         * Enables / disables drop mode. If enabled, fifo will drop data added
         * to full queue. If disabled fifio will remove eldest elements if data
         * is added to queue.
         *
         * @param drop True for enable, false for disable drop mode
         */
        public void enableDropMode(boolean drop) {
            this.drop = drop;
        }

        @Override
        public boolean add(E o) {
            // if drop mode is set, do not add element to full queue
            if (drop & size() >= limit) {
                return false;
            }
            super.add(o);
            // remove eldest element(s)
            while (size() > limit) {
                super.remove();
            }
            return true;
        }
    }
    
    
    public ADXL345(Simulator sim) {
        clock = sim.getClock();
        source = new NullSensorSource();
        applyBWRate();
        applyDataFormat();
        applyFifoCtrl();
        // initial start scheduler
        clock.insertEvent(dataUpdateEvent, clock.millisToCycles(1000 / sampleRate));
    }

    private final Simulator.Event dataUpdateEvent = new Simulator.Event() {

        @Override
        public void fire() {
            // reschedule
            clock.insertEvent(this, clock.millisToCycles(1000 / sampleRate));

            // XXX best way?
            if (source == null) {
                return;
            }
            // read new data
            int new_x = mg_to_raw(source.read(X));
            int new_y = mg_to_raw(source.read(Y));
            int new_z = mg_to_raw(source.read(Z));

            if (bypass) {
                // in bypass mode, new data is writtend directly to registers
                // XXX reguires synchronization on read site

                synchronized (DataX0_reg) {
                    DataX0_reg.setValue(new_x & 0xFF);
                    DataX1_reg.setValue((new_x >> 8) & 0xFF);
                    DataY0_reg.setValue(new_y & 0xFF);
                    DataY1_reg.setValue((new_y >> 8) & 0xFF);
                    DataZ0_reg.setValue(new_z & 0xFF);
                    DataZ1_reg.setValue((new_z >> 8) & 0xFF);
                }

            } else {
                // in all other modes, fifo is used
                fifo.add(new Integer[]{new_x, new_y, new_z});
            }
        }
    };

    @Override
    public Channel[] getChannels() {
        return channels;
    }

    @Override
    public void setSensorSource(SensorSource src) {
        source = src;
    }

    // -- Registers
    
    final OffsetReg OffsetX_reg = new OffsetReg();
    final OffsetReg OffsetY_reg = new OffsetReg();
    final OffsetReg OffsetZ_reg = new OffsetReg();
    final BWRate BWRate_reg = new BWRate();
    final PowerCtl PowerCtl_reg = new PowerCtl();
    final DataFormat DataFormat_reg = new DataFormat();
    final DataReg DataX0_reg = new DataReg();
    final DataReg DataX1_reg = new DataReg();
    final DataReg DataY0_reg = new DataReg();
    final DataReg DataY1_reg = new DataReg();
    final DataReg DataZ0_reg = new DataReg();
    final DataReg DataZ1_reg = new DataReg();
    final FifoCtl FifoCtl_reg = new FifoCtl();
    final FifoStatus FifoStatus_reg = new FifoStatus();

    private class OffsetReg extends RWRegister {

    }

    private class BWRate extends RWRegister {

        static final int RATE0 = 0;
        static final int RATE1 = 1;
        static final int RATE2 = 2;
        static final int RATE3 = 3;
        static final int LOW_POWER = 4;

        public BWRate() {
            value = 0x0A; // Dafaults to 100Hz
        }

        final RegisterView _rate = RegisterUtil.bitRangeView(this, RATE0, RATE3);
        final RegisterView _low_power = RegisterUtil.bitView(this, LOW_POWER);
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
    
    private class DataReg extends RWRegister {
        
    }
    
    private class FifoCtl extends RWRegister {
        static final int SAMPLES_L = 0;
        static final int SAMPLES_H = 4;
        static final int TRIGGER = 5;
        static final int FIFO_MODE_L = 6;
        static final int FIFO_MODE_H = 7;
        
        final RegisterView _samples = RegisterUtil.bitRangeView(this, SAMPLES_L, SAMPLES_H);
        final RegisterView _trigger = RegisterUtil.bitView(this, TRIGGER);
        final RegisterView _mode = RegisterUtil.bitRangeView(this, FIFO_MODE_L, FIFO_MODE_H);
    }
    
    private class FifoStatus extends RWRegister {
        static final int ENTRIES_L = 0;
        static final int ENTRIES_H = 5;
        static final int FIFO_TRIG = 7;
        
        final RegisterView _entries = RegisterUtil.bitRangeView(this, ENTRIES_L, ENTRIES_H);
        final RegisterView _fifo_trig = RegisterUtil.bitView(this, FIFO_TRIG);
    }
    
    private int mg_to_raw(double mg) {
        return (int) Math.round(mg / scale_factor);
    }

    public void connectCS(Microcontroller.Pin.Input cs) {
        CS = cs;
        if (cs != null) {
            cs.registerListener(this);
        }
    }
    
    private void fetchDataFromFifo() {
        logger.trace("Fetch new data from FIFO");
        Integer[] newData = fifo.remove();
        DataX0_reg.setValue(newData[0].intValue() & 0xFF);
        DataX1_reg.setValue((newData[0].intValue() << 8) & 0xFF);
        DataY0_reg.setValue(newData[1].intValue() & 0xFF);
        DataY1_reg.setValue((newData[1].intValue() << 8) & 0xFF);
        DataZ0_reg.setValue(newData[2].intValue() & 0xFF);
        DataZ1_reg.setValue((newData[2].intValue() << 8) & 0xFF);
    }

    private byte read(byte reg) {

        byte retval = 0x00;
        switch (reg & 0xff) {
            case 0x00:
                retval = DEVICE_ID;
                logger.trace("Read reg  0 (DEVICE_ID): {}", retval & 0xFF);
                break;
            case 0x1e: // OFSX
                retval = OffsetX_reg.read();
                logger.trace("Read reg 1e (OFSX): {}", retval & 0xFF);
                break;
            case 0x1f: // OFSY
                retval = OffsetY_reg.read();
                logger.trace("Read reg 1f (OFSY): {}", retval & 0xFF);
                break;
            case 0x20: // OFSZ
                retval = OffsetZ_reg.read();
                logger.trace("Read reg 20 (OFSZ): {}", retval & 0xFF);
                break;
            case 0x2c: // BW_RATE
                retval = BWRate_reg.read();
                logger.trace("Read reg 2c (BW_RATE): {}", retval & 0xFF);
                break;
            case 0x2d:
                retval = PowerCtl_reg.read();
                logger.trace("Read reg 2d (POWER_CTL): {}", retval & 0xFF);
                break;
            case 0x31: // DATA_FORMAT
                retval = DataFormat_reg.read();
                logger.trace("Read reg 31 (DATA_FORMAT): {}", retval & 0xFF);
                break;
            case 0x32: // DATAX0
                // XXX init fifo -> Register if read started
                if (!bypass) {
                    fetchDataFromFifo();
                }
                retval = DataX0_reg.read();
                logger.trace("Read reg 32 (DATAX0): {}", retval & 0xFF);
                break;
            case 0x33: // DATAX1
                if (!bypass && last_reg != 0x32) {
                    fetchDataFromFifo();
                }
                retval = DataX1_reg.read();
//                retval = (byte) (mg_to_raw((int) source.read(X)) >> (8 * (reg % 2)));
                logger.trace("Read reg 33 (DATAX1): {}", retval & 0xFF);
                break;
            case 0x34: // DATAY0
                if (!bypass && last_reg != 0x33) {
                    fetchDataFromFifo();
                }
                retval = DataY0_reg.read();
                logger.trace("Read reg 34 (DATAY0): {}", retval & 0xFF);
                break;
            case 0x35: // DATAY1
                if (!bypass && last_reg != 0x34) {
                    fetchDataFromFifo();
                }
                retval = DataY1_reg.read();
//                retval = (byte) (mg_to_raw((int) source.read(Y)) >> (8 * (reg % 2)));
                logger.trace("Read reg 35 (DATAY1): {}", retval & 0xFF);
                break;
            case 0x36: // DATAZ0
                if (!bypass && last_reg != 0x35) {
                    fetchDataFromFifo();
                }
                retval = DataZ0_reg.read();
                logger.trace("Read reg 36 (DATAZ0): {}", retval & 0xFF);
                break;
            case 0x37: // DATAZ1
                if (!bypass && last_reg != 0x36) {
                    fetchDataFromFifo();
                }
                retval = DataZ1_reg.read();
//                retval = (byte) (mg_to_raw((int) source.read(Z)) >> (8 * (reg % 2)));
                logger.trace("Read reg 37 (DATAZ1): {}", retval & 0xFF);
                break;
            case 0x38: // FIFO_CTL
                retval = FifoCtl_reg.read();
                logger.trace("Read reg 38 (FIFO_CTL): {}", retval & 0xFF);
                break;
            case 0x39: // FIFO_STATUS
                updateFifoStatus();
                retval = FifoStatus_reg.read();
                logger.trace("Read reg 39 (FIFO_STATUS): {}", retval & 0xFF);
                break;
            default:
                logger.warn("Read access to 0x{} not supported", Integer.toHexString(reg & 0xFF));
        }
        
//        logger.info("Read reg 0x{}: {}", Integer.toHexString(reg & 0xff), retval & 0xFF);
        
        return retval;
    }

    private void write(byte reg, byte data) {

        switch (reg & 0xff) {
            case 0x1e: // OFSX
                logger.trace("Write reg 1e (OFSX): {}", data & 0xFF);
                OffsetX_reg.write(data);
                break;
            case 0x1f: // OFSY
                logger.trace("Write reg 1f (OFSY): {}", data & 0xFF);
                OffsetY_reg.write(data);
                break;
            case 0x20: // OFSZ
                logger.trace("Write reg 20 (OFSZ): {}", data & 0xFF);
                OffsetZ_reg.write(data);
                break;
            case 0x2c: // BW_RATE
                logger.trace("Write reg 2c (BW_RATE): {}", data & 0xFF);
                BWRate_reg.write(data);
                applyBWRate();
                break;
            case 0x2d: // POWER_CTL
                logger.trace("Write reg 2d (POWER_CTL): {}", data & 0xFF);
                PowerCtl_reg.write(data);
                break;
            case 0x31: // DATA_FORMAT
                logger.trace("Write reg 31 (DATA_FORMAT): {}", data & 0xFF);
                DataFormat_reg.write(data);
                applyDataFormat();
                break;
            case 0x38: // FIFO_CTL
                logger.trace("Write reg 38 (FIFO_CTL): {}", data & 0xFF);
                FifoCtl_reg.write(data);
                applyFifoCtrl();
                break;
            default:
                logger.warn("Write access to 0x{} not supported", Integer.toHexString(reg & 0xFF));
        }
    }
    
    // -- apply functions (apply settings based on register data)

    private void applyBWRate() {
        int rate_code = BWRate_reg._rate.getValue();
        double rate = 0;
        switch (rate_code) {
            case 0xF:
                rate = 3200;
                break;
            case 0xE:
                rate = 1600;
                break;
            case 0xD:
                rate = 800;
                break;
            case 0xC:
                rate = 400;
                break;
            case 0xB:
                rate = 200;
                break;
            case 0xA:
                rate = 100;
                break;
            case 0x9:
                rate = 50;
                break;
            case 0x8:
                rate = 25;
                break;
            case 0x7:
                rate = 12.5;
                break;
            case 0x6:
                rate = 6.25;
                break;
            case 0x5:
                rate = 3.13;
                break;
            case 0x4:
                rate = 1.56;
                break;
            case 0x3:
                rate = 0.78;
                break;
            case 0x2:
                rate = 0.39;
                break;
            case 0x1:
                rate = 0.20;
                break;
            case 0x0:
                rate = 0.10;
                break;
        }
        if (rate != sampleRate) {
            sampleRate = rate;
        }

        boolean low_power = BWRate_reg._low_power.getValue() == 1;
    }

    private void applyDataFormat() {
        // evaluate FULL_RES bit
        full_resolution = DataFormat_reg._full_res.getValue() != 0;
        // evaluate g range
        double lbound = 0.0, ubound = 0.0;
        int range = DataFormat_reg._range.getValue();
        switch (range & 0x03) {
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
        logger.debug("Set to range [{} - {}]", lbound, ubound);
        for (Channel c : channels) {
            c.lbound = lbound;
            c.ubound = ubound;
        }
    }

    private void applyFifoCtrl() {
        int mode = FifoCtl_reg._mode.getValue();
        switch (mode) {
            case 0: // Bypass
                // 'placing the device into bypass mode clears fifo'
                fifo.clear();
                bypass = true;
                logger.info("Set to Bypass mode");
                break;
            case 1: // FIFO
                fifo.enableDropMode(true);
                bypass = false;
                logger.info("Set to FIFO mode");
                break;
            case 2: // Stream
                fifo.enableDropMode(false);
                bypass = false;
                logger.info("Set to Stream mode");
                break;
            case 3: // Trigger
                fifo.enableDropMode(true);
                bypass = false;
                logger.info("Set to Trigger mode");
                break;
        }
        boolean trigger = FifoCtl_reg._trigger.getValue() == 1;
        int samples = FifoCtl_reg._samples.getValue();
    }
    
    // -- Update functions (set register value)
    
    private void updateFifoStatus() {
        FifoStatus_reg._entries.setValue(fifo.size());
        // XXX FIFO_TRIG
    }
    
    private int last_reg = -1;
    private boolean multi_byte = false;

    @Override
    public SPI.Frame exchange(SPI.Frame frame) {
        if (CS != null && CS.read()) {
            return SPI.newFrame((byte) 0x00);
        }

        byte result = 0x00;
        if (reg == -1) { // awaiting r/w start byte

            is_read = (frame.data & 0x80) != 0;
            multi_byte = (frame.data & 0x40) != 0;
            reg = frame.data & 0x3F;

        } else {
            if (is_read) {
                result = read((byte) reg);
            } else {
                write((byte) reg, frame.data);
            }
            
            if (multi_byte) { // auto-increment on multi byte mode
                last_reg = reg;
                reg++;
            } else {
                last_reg = -1;
                reg = -1;
            }
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
            last_reg = -1;
            reg = -1;
        }
    }
}
