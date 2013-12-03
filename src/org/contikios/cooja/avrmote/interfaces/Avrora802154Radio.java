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
import org.contikios.cooja.avrmote.MicaZMoteType;
import org.contikios.cooja.emulatedmote.Radio802154;
import avrora.sim.FiniteStateMachine;
import avrora.sim.FiniteStateMachine.Probe;
import avrora.sim.radio.Medium;
import avrora.sim.radio.Medium.Receiver;
import avrora.sim.radio.Medium.Transmitter;
import avrora.sim.radio.Radio;

/**
 * Cooja support for Avrora's 802.15.4 radios.
 *
 * @see MicaZMoteType
 * @author Fredrik Osterlind
 */
@ClassDescription("Avrora's 802.15.4")
public abstract class Avrora802154Radio extends Radio802154 {
  private static Logger logger = Logger.getLogger(Avrora802154Radio.class);

  private Medium.Probe txProbe;
  private Probe stateProbe;

  private double signalStrength = -1;

  private Receiver receiver;
  private Transmitter transmitter;
  private FiniteStateMachine fsm;

  public Avrora802154Radio(Mote mote, Radio radio, FiniteStateMachine fsm) {
    super(mote);

    this.receiver = radio.getReceiver();
    this.transmitter = radio.getTransmitter();
    this.fsm = fsm;

    transmitter.insertProbe(txProbe = new Medium.Probe.Empty() {
      public void fireBeforeTransmit(Medium.Transmitter t, byte val) {
        handleTransmit(val);
      }
    });
    fsm.insertProbe(stateProbe = new Probe() {
      public void fireBeforeTransition(int beforeState, int afterState) {
      }
      public void fireAfterTransition(int beforeState, int afterState) {
        /* Note that all other radio events but HW_ON and HW_OFF are triggered
         * when the radio sends bytes */

        if (!isRadioOn(beforeState) && isRadioOn(afterState)) {
          radioOn();
          return;
        }

        if (isRadioOn(beforeState) && !isRadioOn(afterState)) {
          radioOff();
          return;
        }
      }
    });
  }

  public void removed() {
    super.removed();

    transmitter.removeProbe(txProbe);
    fsm.removeProbe(stateProbe);
  }

  public void handleReceive(byte b) {
    receiver.nextByte(true, b);
  }

  protected void handleStartOfReception() {
  }

  protected void handleEndOfReception() {
    /* tell the receiver that the packet is ended */
    receiver.nextByte(false, (byte)0);
  }

  public int getChannel() {
    /* TODO Need to listen of channel changes, and post event */
    return (int) ((getFrequency() - 2405.0)/5) + 11;
  }

  public double getCurrentSignalStrength() {
    return signalStrength;
  }

  public void setCurrentSignalStrength(double signalStrength) {
    this.signalStrength = signalStrength;
    /* XXX Does the receiver take care of averaging this over 8 symbols, or should we? */
    receiver.setRSSI(signalStrength);
  }

  public abstract double getFrequency();

  protected abstract boolean isRadioOn(int state);

  public boolean isRadioOn() {
    int state = fsm.getCurrentState();
    return isRadioOn(state);
  }
}
