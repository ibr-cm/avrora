/*
 * Copyright (c) 2012, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

package org.contikios.cooja.avrmote.interfaces;

import org.apache.log4j.Logger;

import org.contikios.cooja.ClassDescription;
import org.contikios.cooja.Mote;
import org.contikios.cooja.avrmote.RFA1Mote;
import avrora.sim.radio.ATmega128RFA1Radio;
import avrora.sim.radio.Radio;

/**
 * Cooja support for Avrora's ATMega128RFA1.
 *
 * @author Joakim Eriksson, David Kopf, Fredrik Osterlind
 */
@ClassDescription("RFA1 Radio")
public class RFA1Radio extends Avrora802154Radio {
  private static Logger logger = Logger.getLogger(RFA1Radio.class);

  /* Radio is idle in state 2, receiver in state 3-5, and a transmitter for > 5 */
  /* See ATmega128RFA1Energy.java for the power consumptions */
  private final static int STATE_POWEROFF = 0;
  private final static int STATE_POWERDOWN = 1;
  private final static int STATE_IDLE = 2;

  private ATmega128RFA1Radio rfa1radio;

  public RFA1Radio(Mote mote) {
    super(mote,
        ((Radio) ((RFA1Mote)mote).getRFA1().getDevice("radio")),
        ((ATmega128RFA1Radio) ((RFA1Mote)mote).getRFA1().getDevice("radio")).getFiniteStateMachine());
    rfa1radio = (ATmega128RFA1Radio) ((RFA1Mote)mote).getRFA1().getDevice("radio");
  }

  public double getFrequency() {
      return rfa1radio.getFrequency();
  }

  protected boolean isRadioOn(int state) {
    if (state == STATE_POWEROFF) {
      return false;
    }
    if (state == STATE_POWERDOWN) {
      return false;
    }
    /* Idle uses about half the power of the rx state. Return on to err on the high side of enery consumption.
    if (state == STATE_IDLE) {
      return false;
    }
    */

    return true;
  }

  public double getCurrentOutputPower() {
    return rfa1radio.getPower();
  }

  public int getCurrentOutputPowerIndicator() {
    return 0x0f - rfa1radio.getPowerIndicator(); /* 0: max power */
  }

  public int getOutputPowerIndicatorMax() {
    return 0x0f;
  }
}
