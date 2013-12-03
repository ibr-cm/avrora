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

package avrora.sim.mcu;

import avrora.sim.RWRegister;
import avrora.sim.Simulator;
import avrora.sim.state.RegisterView;
import avrora.sim.state.RegisterUtil;
import avrora.sim.clock.Clock;

/**
 * Base class of 8-bit timers. Timer0 and Timer2 are subclasses of this.
 *
 * @author Daniel Lee
 */
public abstract class Timer8Bit extends AtmelInternalDevice {
    public static final int MODE_NORMAL = 0;
    public static final int MODE_PWM = 1;
    public static final int MODE_CTC = 2;
    public static final int MODE_FASTPWM = 3;
    public static final int MAX = 0xff;
    public static final int BOTTOM = 0x00;

    final ControlRegister TCCRn_reg;
    final ControlRegisterA TCCRnA_reg;
    final ControlRegisterB TCCRnB_reg;
    final TCNTnRegister TCNTn_reg;
    final BufferedRegister OCRn_reg,OCRnB_reg;

    protected final int n; // number of timer. 0 for Timer0, 2 for Timer2

    protected Simulator.Event ticker;
    protected final Clock externalClock;
    protected Clock timerClock;

    protected int period;

    final AtmelMicrocontroller.Pin outputComparePin,outputComparePinB;
    final Simulator.Event[] tickers;

    /* pg. 93 of manual. Block compareMatch for one period after
     * TCNTn is written to. */
    boolean blockCompareMatch;
    final int OCIEn;
    final int TOIEn;
    final int OCFn;
    final int TOVn;
    final int ab;

    protected ATMegaFamily.FlagRegister TIFR_reg;
    protected ATMegaFamily.MaskRegister TIMSK_reg;

    final int[] periods;
    //ATMega timers
    protected Timer8Bit(AtmelMicrocontroller m, int ab, int n, int OCIEn, int TOIEn, int OCFn, int TOVn, int[] periods) {
        super("timer"+n, m);
        this.ab = ab;
        this.TOIEn = TOIEn;
        this.TOVn = TOVn;
        TCNTn_reg = new TCNTnRegister();
        OCRn_reg = new BufferedRegister();
        //B enables are the next bit up
        this.OCIEn = OCIEn;
        this.OCFn = OCFn;

        if (ab != 0) {
            TCCRn_reg = null; 
            TCCRnA_reg = new ControlRegisterA();
            TCCRnB_reg = new ControlRegisterB();
            OCRnB_reg = new BufferedRegister();
            TIFR_reg = (ATMegaFamily.FlagRegister)m.getIOReg("TIFR"+n);
            TIMSK_reg = (ATMegaFamily.MaskRegister)m.getIOReg("TIMSK"+n);
            outputComparePin = (AtmelMicrocontroller.Pin)microcontroller.getPin("OC"+n+"A");
            if(n!=2) //no pin on atmega128rfa1
                outputComparePinB = (AtmelMicrocontroller.Pin)microcontroller.getPin("OC"+n+"B");
            else
                outputComparePinB = null;
            installIOReg("TCCR"+n+"A", TCCRnA_reg);
            installIOReg("TCCR"+n+"B", TCCRnB_reg);
            installIOReg("TCNT"+n, TCNTn_reg);
            installIOReg("OCR"+n+"A", OCRn_reg);
  //          if(n!=2) //no pin on atmega128rfa1
            installIOReg("OCR"+n+"B", OCRnB_reg);
        } else {
            TCCRn_reg = new ControlRegister();
            TCCRnA_reg = null; 
            TCCRnB_reg = null; 
            OCRnB_reg = null;            
            TIFR_reg = (ATMegaFamily.FlagRegister)m.getIOReg("TIFR");
            TIMSK_reg = (ATMegaFamily.MaskRegister)m.getIOReg("TIMSK");
            outputComparePin = (AtmelMicrocontroller.Pin)microcontroller.getPin("OC"+n);
            outputComparePinB = null;
            installIOReg("TCCR"+n, TCCRn_reg);
            installIOReg("TCNT"+n, TCNTn_reg);
            installIOReg("OCR"+n, OCRn_reg);  
        }

        externalClock = m.getClock("external");
        timerClock = mainClock;
        this.n = n;
        this.periods = periods;

        tickers = new Simulator.Event[4];
        installTickers();
    }

    private void installTickers() {
        tickers[MODE_NORMAL] = new Mode_Normal();
        tickers[MODE_CTC] = new Mode_CTC();
        tickers[MODE_FASTPWM] = new Mode_FastPWM();
        tickers[MODE_PWM] = new Mode_PWM();
    }

    protected void compareMatch() {
   //   if (devicePrinter != null)  devicePrinter.println("Timer" + n + ".compareMatch (enabled: " + TIMSK_reg.readBit(OCIEn) + ')');
        // set the compare flag for this timer
        TIFR_reg.flagBit(OCFn);
        // if the mode is correct, modify pin OCn. but if the flag is
        // already connected to the pin, does this happen automatically
        // with the last previous call?
        //compareMatchPin();
    }

    protected void overflow() {
   //   if (devicePrinter != null) devicePrinter.println("Timer" + n + ".overFlow (enabled: " + TIMSK_reg.readBit(TOIEn) + ')');
        // set the overflow flag for this timer
        TIFR_reg.flagBit(TOVn);
    }

    /**
     * Overloads the write behavior of this class of register in order to implement compare match
     * blocking for one timer period.
     */
    protected class TCNTnRegister extends RWRegister {

        public void write(byte val) {
            value = val;
            blockCompareMatch = true;
        }

    }

    /**
     * <code>BufferedRegister</code> implements a register with a write buffer. In PWN modes, writes
     * to this register are not performed until flush() is called. In non-PWM modes, the writes are
     * immediate.
     */
    protected class BufferedRegister extends RWRegister {
        final RWRegister register;

        protected BufferedRegister() {
            this.register = new RWRegister();
        }

        public void write(byte val) {
            super.write(val);
            if (TCCRn_reg != null) {
                if (TCCRn_reg.mode == MODE_NORMAL || TCCRn_reg.mode == MODE_CTC) {
                    flush();
                }
            } else {
                flush();
           // TODO: handle AB
            }
        }

        public byte readBuffer() {
            return super.read();
        }

        public byte read() {
            return register.read();
        }

        protected void flush() {
            register.write(value);
        }
    }

    protected class ControlRegister extends RWRegister {
        public static final int FOCn = 7;
        public static final int WGMn0 = 6;
        public static final int COMn1 = 5;
        public static final int COMn0 = 4;
        public static final int WGMn1 = 3;
        public static final int CSn2 = 2;
        public static final int CSn1 = 1;
        public static final int CSn0 = 0;

        final RegisterView CSn = RegisterUtil.bitRangeView(this, 0, 2);
        final RegisterView COMn = RegisterUtil.bitRangeView(this, 4, 5);
        final RegisterView WGMn = RegisterUtil.permutedView(this, new byte[] {6, 3});

        int mode = -1;
        int scale = -1;

        public void write(byte val) {
            // hardware manual states that high order bit is always read as zero
            value = (byte)(val & 0x7f);

            if ((val & 0x80) != 0) {
                forcedOutputCompare();
            }

            // decode modes and update internal state
            int nmode = WGMn.getValue();
            int nscale = CSn.getValue();
            // if the scale or the mode has changed
            if (nmode != mode || nscale != scale) {
                if (ticker != null) timerClock.removeEvent(ticker);
                mode = nmode;
                scale = nscale;
                ticker = tickers[mode];
                period = periods[scale];
                if (period != 0) {
                    timerClock.insertEvent(ticker, period);
                }
                if (false && devicePrinter != null) {
                  if (period != 0)
                    devicePrinter.println("Timer" + n + " enabled: period = " + period + " mode = " + mode);
                  else
                    devicePrinter.println("Timer" + n + " disabled");
                }
            }
        }

        private void forcedOutputCompare() {

            int count = TCNTn_reg.read() & 0xff;
            int compare = OCRn_reg.read() & 0xff;

            // the non-PWM modes are NORMAL and CTC
            // under NORMAL, there is no pin action for a compare match
            // under CTC, the action is to clear the pin.

            // TODO: this implementation is probably not correct...
            if (count == compare) {
                switch (COMn.getValue()) {
                    case 1:
                        if (WGMn.getValue() == MODE_NORMAL || WGMn.getValue() == MODE_CTC)
                          outputComparePin.write(!outputComparePin.read()); // toggle
                        break;
                    case 2:
                        outputComparePin.write(false); // clear
                        break;
                    case 3:
                        outputComparePin.write(true); // set to true
                        break;
                }

            }
        }
    }

    //ATMega1284p TCCR2A and B register
    //TODO: perform the action on writes to either A or B
 
    protected class ControlRegisterA extends RWRegister {
        public static final int COMnA1 = 7;
        public static final int COMnA0 = 6;
        public static final int COMnB1 = 5;
        public static final int COMnB0 = 4;
        public static final int WGMn1  = 1;
        public static final int WGMn0  = 0;
        public void write(byte val) {
            super.write(val);
           // System.out.println("ControlregisterA write byte " + val + " to " + this);
        }
    }
 
    protected class ControlRegisterB extends RWRegister {
        public static final int FOCnA  = 7;
        public static final int FOCnB  = 6;
        public static final int WGMn2  = 3;
        public static final int CSn2   = 2;
        public static final int CSn1   = 1;
        public static final int CSn0   = 0;

        final RegisterView CSn = RegisterUtil.bitRangeView(this, 0, 2);

        int mode = -1;
        int scale = -1;

        public void write(byte val) {
            super.write(val);
          //  System.out.println("ControlregisterB write byte " + val + " to " + this);
            //  Combine the WGM bits to get the mode
           // System.out.println("controlregisterA is " + TCCRnA_reg.getValue());
            int nmode = (((value & 4) >>1 ) | (TCCRnA_reg.getValue() & 3));

            // Forced output compare only active in a non-PWM mode (WGM0 clear)
            if (((nmode & 1) != 0) && ((val & 0xC0) != 0)) {
                System.out.println("Forced output compare AB");
                forcedOutputCompare(mode, val);
            }

            // decode modes and update internal state

            int nscale = CSn.getValue();
         //   System.out.println("mode is " + nmode + " CSn is " + CSn.getValue());
            // if the scale or the mode has changed
            if (nmode != mode || nscale != scale) {
                if (ticker != null) timerClock.removeEvent(ticker);
                mode = nmode;
                scale = nscale;
                ticker = tickers[mode];
                period = periods[scale];
                if (period != 0) {
                    timerClock.insertEvent(ticker, period);
                }
                if (false && devicePrinter != null) {
                  if (period != 0)
                    devicePrinter.println("Timer" + n + " enabled: period = " + period + " mode = " + mode);
                  else
                    devicePrinter.println("Timer" + n + " disabled");
                }
            }
        }
        private void forcedOutputCompare(int mode, int value) {

            int count = TCNTn_reg.read() & 0xff;
            int compare = OCRn_reg.read() & 0xff;

            AtmelMicrocontroller.Pin pinAB;
            int com;
            if ((value & 0x80) != 0) {
                com = (TCCRnA_reg.getValue() & 0xC0) >> 6;
                pinAB = outputComparePin;
            } else {
                com = (TCCRnA_reg.getValue() & 0x30) >> 4;
                pinAB = outputComparePinB;
            }
            if (pinAB == null) return;
            // the non-PWM modes are NORMAL and CTC
            // under NORMAL, there is no pin action for a compare match
            // under CTC, the action is to clear the pin.

            // TODO: this implementation is probably not correct...
            if (count == compare) {
                switch (com) {
                    case 1:
                     //   if (WGMn.getValue() == MODE_NORMAL || WGMn.getValue() == MODE_CTC)
                          pinAB.write(!pinAB.read()); // toggle
                        break;
                    case 2:
                        pinAB.write(false); // clear
                        break;
                    case 3:
                        pinAB.write(true); // set to true
                        break;
                }

            }
        }
    }

    class Mode_Normal implements Simulator.Event {
        public void fire() {
            int ncount = (int)TCNTn_reg.read() & 0xff;
        // System.out.println("noamalfire, ncount = " + ncount + " BOTTOM " + BOTTOM + " MAX " + MAX);
            tickerStart(ncount);
            if (ncount >= MAX) {
                overflow();
                ncount = BOTTOM;
            }
            else {
                ncount++;
            }
            tickerFinish(this, ncount);
        }
    }

    class Mode_PWM implements Simulator.Event {
        protected byte increment = 1;
        public void fire() {
            // TODO: OCn handling
            int ncount = (int)TCNTn_reg.read() & 0xff;
            tickerStart(ncount);
            if (ncount >= MAX) {
                increment = -1;
                ncount = MAX;
                OCRn_reg.flush(); // pg. 102. update OCRn at TOP
            }
            else if (ncount <= BOTTOM) {
                overflow();
                increment = 1;
                ncount = BOTTOM;
            }
            ncount += increment;
            tickerFinish(this, ncount);
        }
    }

    class Mode_CTC implements Simulator.Event {
        public void fire() {
            int ncount = (int)TCNTn_reg.read() & 0xff;
            tickerStart(ncount);
      //      System.out.println("ctcfire, ncount = " + ncount + " BOTTOM " + BOTTOM + " MAX " + MAX + " OCRn " + OCRn_reg.read());
            if (ncount >= MAX) {
                // OCRn == MAX, then overflow is handled as in normal mode
                overflow();
                ncount = BOTTOM;
            }
            else if (ncount == ((int)OCRn_reg.read() & 0xff)) {
                ncount = BOTTOM;
            }
            else {
                ncount++;
            }
            tickerFinish(this, ncount);
        }
    }

    class Mode_FastPWM implements Simulator.Event {
        public void fire() {
            // TODO: OCn handling
            int ncount = (int)TCNTn_reg.read() & 0xff;
            tickerStart(ncount);
            if (ncount >= MAX) {
                ncount = BOTTOM;
                overflow();
                OCRn_reg.flush(); // pg. 102. update OCRn at TOP
            }
            else {
                ncount++;
            }
            tickerFinish(this, ncount);
        }
    }
    
    private void tickerStart(int count) {
      if (!blockCompareMatch && count == ((int)OCRn_reg.read() & 0xff)) {
          compareMatch();
      }
    }
    
    private void tickerFinish(Simulator.Event ticker, int ncount) {
        TCNTn_reg.write((byte)ncount);
        // previous write sets the compare, so reset it now
        blockCompareMatch = false;

        timerClock.insertEvent(ticker, period);
    }
}
