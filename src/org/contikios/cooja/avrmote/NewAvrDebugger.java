/*
 * Copyright (c) 2012, The Contiki OS (www.contiki-os.org).
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
 */

package org.contikios.cooja.avrmote;

import java.io.File;
import java.util.ArrayList;

import javax.swing.JTabbedPane;

import org.apache.log4j.Logger;

import org.contikios.cooja.ClassDescription;
import org.contikios.cooja.Cooja;
import org.contikios.cooja.Mote;
import org.contikios.cooja.PluginType;
import org.contikios.cooja.Simulation;
import org.contikios.cooja.SupportedArguments;
import org.contikios.cooja.WatchpointMote;
import org.contikios.cooja.dialogs.CompileContiki;
import org.contikios.cooja.dialogs.MessageList;
import org.contikios.cooja.dialogs.MessageList.MessageContainer;
import org.contikios.cooja.plugins.Debugger;
import avrora.sim.AtmelInterpreter;
import avrora.sim.mcu.AtmelMicrocontroller;

@ClassDescription("Souce-level debugger")
@PluginType(PluginType.MOTE_PLUGIN)
@SupportedArguments(motes = {AvroraMote.class})
public class NewAvrDebugger extends Debugger {
  private static Logger logger = Logger.getLogger(NewAvrDebugger.class);

  private AvroraMote avroraMote;
  private AtmelInterpreter interpreter;
  private String[] debugSourceFiles;

  /**
   * Source-level debugger for Avrora-based motes.
   *
   * @param mote MSP Mote
   * @param simulationToVisualize Simulation
   * @param gui Simulator
   */
  public NewAvrDebugger(Mote mote, Simulation sim, Cooja gui) {
    super((WatchpointMote)mote, sim, gui, "Avr Debugger - " + mote);
    this.avroraMote = (AvroraMote) mote;

    AtmelMicrocontroller cpu = (AtmelMicrocontroller) avroraMote.getPlatform().getMicrocontroller();
    interpreter = (AtmelInterpreter)cpu.getSimulator().getInterpreter();

    debugSourceFiles = parseSourcefiles();
  }

  /* Parse debugging info:
   * Uses avr-objdump to parse source files from the firmware debugging information.
   * This code is inspired by David Kopf's code in now obsolete AvrDebugger.java.
   *
   * These methods are mote type-specific, and may be moved to AvroraMoteType.java. */
  private String[] parseSourcefiles() {
    ArrayList<String> files = new ArrayList<String>();
    try {
      File firmwareFile = ((AvroraMoteType)avroraMote.getType()).getContikiFirmwareFile();
      MessageList output = new MessageList();
      CompileContiki.compile(
          "avr-objdump -g " + firmwareFile.getName(),
          null, null, firmwareFile.getParentFile(), null, null, output, true);

      for (MessageContainer mc: output.getMessages()) {
        String line = mc.message;
        if (!line.contains("DW_AT_name")) {
          continue;
        }
        line = line.trim();
        int space = line.lastIndexOf(" ");
        if (space < 0) {
          continue;
        }
        String name = line.substring(space+1);
        if (name.endsWith(".c")) {
          files.add(name);
        }
      }
    } catch (Exception e) {
      logger.warn("Error extracting source files: " + e.getMessage(), e);
      return null;
    }
    return files.toArray(new String[0]);
  }

  public void addTabs(JTabbedPane pane, WatchpointMote mote) {
    /* nothing to add */
  }

  public String[] getSourceFiles() {
    return debugSourceFiles;
  }

  public void step(WatchpointMote watchpointMote) {
    interpreter.step();
  }

  public int getCurrentPC() {
    return interpreter.getState().getPC();
  }

  public SourceLocation getSourceLocation(int pc) {
    return avroraMote.getSourceLocation(pc);
  }

}
