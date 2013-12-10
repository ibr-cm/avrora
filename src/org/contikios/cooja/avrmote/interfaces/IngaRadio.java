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
//import org.contikios.cooja.avrmote.RavenMote;
import avrora.sim.radio.AT86RF231Radio;
import avrora.sim.radio.Radio;
import org.contikios.cooja.avrmote.IngaMote;

/**
 * Cooja support for Avrora's Atmel AT86RF230 radio (AT86RF231).
 *
 * @author David Kopf, Fredrik Osterlind
 */
@ClassDescription("AT86RF230 Radio")
public class IngaRadio extends Avrora802154Radio {
  private static Logger logger = Logger.getLogger(RavenRadio.class);
  private final static boolean DEBUG = false;

  /* TODO XXX Verify states */
  private final static int STATE_POWEROFF = 0;
  private final static int STATE_POWERDOWN = 1;
  private final static int STATE_IDLE = 2;

  private AT86RF231Radio rf231radio;

  public IngaRadio(Mote mote) {
    super(mote,
        ((Radio) ((IngaMote)mote).getInga().getDevice("radio")),
        ((AT86RF231Radio) ((IngaMote)mote).getInga().getDevice("radio")).getFiniteStateMachine());
    rf231radio = (AT86RF231Radio) ((IngaMote)mote).getInga().getDevice("radio");
  }

  @Override
  public double getFrequency() {
    if (DEBUG) System.out.println("Inga getFrequency " +  rf231radio.getFrequency());
    return rf231radio.getFrequency();
  }

  @Override
  protected boolean isRadioOn(int state) {
    if (state == STATE_POWEROFF) {
      return false;
    }
    else if (state == STATE_POWERDOWN) {
      return false;
    }
    else if (state == STATE_IDLE) {
      return false;
    }

    return true;
  }

  @Override
  public double getCurrentOutputPower() {
    if (DEBUG) System.out.println("Inga getCurrentOutputPower " + rf231radio.getPower());
    return rf231radio.getPower();
  }

  @Override
  public int getCurrentOutputPowerIndicator() {
    if (DEBUG) System.out.println("Inga getOutputPowerIndicator" );
    return 0x0f - (rf231radio.readRegister(AT86RF231Radio.PHY_TX_PWR) & 0x0f); /* 0: max power */
  }

  @Override
  public int getOutputPowerIndicatorMax() {
    if (DEBUG) System.out.println("Inga getOutputPowerIndicatorMax" );
    return 0x0f;
  }
}
