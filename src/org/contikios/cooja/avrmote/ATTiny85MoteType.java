/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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

import java.io.File;
import java.util.Collection;
import javax.swing.JComponent;
import org.contikios.cooja.AbstractionLevelDescription;
import org.contikios.cooja.ClassDescription;
import org.contikios.cooja.Mote;
import org.contikios.cooja.MoteInterface;
import org.contikios.cooja.ProjectConfig;
import org.contikios.cooja.Simulation;
import org.contikios.cooja.avrmote.interfaces.ATTiny85ID;
import org.contikios.cooja.avrmote.interfaces.AvrDebugger;
import org.contikios.cooja.avrmote.interfaces.AvroraADC;
import org.contikios.cooja.avrmote.interfaces.AvroraClock;
import org.contikios.cooja.avrmote.interfaces.AvroraLED;
import org.contikios.cooja.interfaces.IPAddress;
import org.contikios.cooja.interfaces.Mote2MoteRelations;
import org.contikios.cooja.interfaces.MoteAttributes;
import org.contikios.cooja.interfaces.Position;
import org.contikios.cooja.interfaces.RimeAddress;
import org.jdom.Element;

/**
 * AVR-based MicaZ mote types emulated in Avrora.
 *
 * @author Joakim Eriksson, Fredrik Osterlind
 */
@ClassDescription("ATTiny85 Mote Type")
@AbstractionLevelDescription("Emulated level")
public class ATTiny85MoteType extends AvroraMoteType {

  // The returned string is used for mote type name and icon jpg file
  public final String getMoteName() {
    return ("Tiny85");
  }
  // The returned string is used for firmware file extension
  public final String getMoteContikiTarget() {
    return ("tiny85");
  }

  public final Mote generateMote(Simulation simulation) {
    ATTiny85Mote mote = new ATTiny85Mote(simulation, this);
    mote.initMote();
    return mote;
  }

  @SuppressWarnings("unchecked")
  public Class<? extends MoteInterface>[] getAllMoteInterfaceClasses() {
    return new Class[] {
        Position.class,
        ATTiny85ID.class,
        AvroraLED.class,
        AvroraADC.class,
        AvroraClock.class,
        AvrDebugger.class,
        MoteAttributes.class,
        Mote2MoteRelations.class,
        RimeAddress.class,
        IPAddress.class
    };
  }

 

}
