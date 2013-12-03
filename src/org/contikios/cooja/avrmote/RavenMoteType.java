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

package org.contikios.cooja.avrmote;

import org.contikios.cooja.AbstractionLevelDescription;
import org.contikios.cooja.ClassDescription;
import org.contikios.cooja.Mote;
import org.contikios.cooja.MoteInterface;
import org.contikios.cooja.Simulation;
import org.contikios.cooja.avrmote.interfaces.AvrDebugger;
import org.contikios.cooja.avrmote.interfaces.AvroraADC;
import org.contikios.cooja.avrmote.interfaces.AvroraClock;
import org.contikios.cooja.avrmote.interfaces.AvroraLED;
import org.contikios.cooja.avrmote.interfaces.AvroraMoteID;
import org.contikios.cooja.avrmote.interfaces.AvroraUsart1;
import org.contikios.cooja.avrmote.interfaces.RavenRadio;
import org.contikios.cooja.interfaces.IPAddress;
import org.contikios.cooja.interfaces.Mote2MoteRelations;
import org.contikios.cooja.interfaces.MoteAttributes;
import org.contikios.cooja.interfaces.Position;
import org.contikios.cooja.interfaces.RimeAddress;

/**
 * AVR-based Raven mote types emulated in Avrora.
 *
 * @author Joakim Eriksson, Fredrik Osterlind, David Kopf
 */
@ClassDescription("Raven Mote Type")
@AbstractionLevelDescription("Emulated level")
public class RavenMoteType extends AvroraMoteType {

  // The returned string is used for mote type name and icon jpg file
  public final String getMoteName() {
    return ("Raven");
  }
  // The returned string is used for firmware file extension
  public final String getMoteContikiTarget() {
    return ("avr-raven");
  }

  public final Mote generateMote(Simulation simulation) {
    RavenMote mote = new RavenMote(simulation, this);
    mote.initMote();
    return mote;
  }

  /* Note the ADC and Debugger interfaces are also an extension of Clock and
   * will get the setDrift/getDrift calls for random startup if included before
   * the Clock interface. The clock would then show zero drift.
   */
  @SuppressWarnings("unchecked")
  public Class<? extends MoteInterface>[] getAllMoteInterfaceClasses() {
    return new Class[] {
        Position.class,
        AvroraMoteID.class,
        AvroraLED.class,
        RavenRadio.class,
        AvroraClock.class,
        AvroraUsart1.class,
        AvrDebugger.class,
        AvroraADC.class,
        Mote2MoteRelations.class,
        MoteAttributes.class,
        RimeAddress.class,
        IPAddress.class
    };
  }

}
