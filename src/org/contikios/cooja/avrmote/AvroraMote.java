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

import java.io.File;
import java.util.ArrayList;
import java.util.Collection;

import org.apache.log4j.Logger;
import org.jdom.Element;

import org.contikios.cooja.MoteInterface;
import org.contikios.cooja.MoteInterfaceHandler;
import org.contikios.cooja.MoteMemory;
import org.contikios.cooja.MoteType;
import org.contikios.cooja.Simulation;
import org.contikios.cooja.Watchpoint;
import org.contikios.cooja.WatchpointMote;
import org.contikios.cooja.dialogs.CompileContiki;
import org.contikios.cooja.dialogs.MessageList;
import org.contikios.cooja.dialogs.MessageList.MessageContainer;
import org.contikios.cooja.motes.AbstractEmulatedMote;
import org.contikios.cooja.plugins.Debugger.SourceLocation;
import avrora.arch.avr.AVRProperties;
import avrora.core.LoadableProgram;
import avrora.core.SourceMapping;
import avrora.sim.AtmelInterpreter;
import avrora.sim.Simulator;
import avrora.sim.mcu.AtmelMicrocontroller;
import avrora.sim.mcu.EEPROM;
import avrora.sim.platform.Platform;
import avrora.sim.platform.PlatformFactory;

/**
 * @author Joakim Eriksson, Fredrik Osterlind, David Kopf
 */
public abstract class AvroraMote extends AbstractEmulatedMote implements WatchpointMote {
  public static Logger logger = Logger.getLogger(AvroraMote.class);

  private MoteInterfaceHandler moteInterfaceHandler;
  private MoteType moteType;
  private PlatformFactory factory;

  private Platform platform = null;
  private EEPROM EEPROM = null;
  private AtmelInterpreter interpreter = null;
  private AvrMoteMemory memory = null;

  public Simulator sim;
  public SourceMapping sourceMapping;

  /* Stack monitoring variables */
  private boolean stopNextInstruction = false;

  public AvroraMote(Simulation simulation, MoteType type, PlatformFactory factory) {
    setSimulation(simulation);
    moteType = type;
    this.factory = factory;

    /* Schedule us immediately */
    requestImmediateWakeup();
  }

  protected boolean initEmulator(File fileELF) {
    try {
      LoadableProgram program = new LoadableProgram(fileELF);
      program.load();
      sourceMapping = program.getProgram().getSourceMapping();
      platform = factory.newPlatform(1, program.getProgram());
      AtmelMicrocontroller cpu = (AtmelMicrocontroller) platform.getMicrocontroller();
      EEPROM = (EEPROM) cpu.getDevice("eeprom");
      AVRProperties avrProperties = (AVRProperties) cpu.getProperties();
      sim = cpu.getSimulator();
      interpreter = (AtmelInterpreter) sim.getInterpreter();
      memory = new AvrMoteMemory(program.getProgram().getSourceMapping(), avrProperties, interpreter);
    } catch (Exception e) {
      logger.fatal("Error when initializing Avora mote: " + e.getMessage(), e);
      return false;
    }
    return true;
  }

  public Platform getPlatform() {
    return platform;
  }

  /**
   * Abort mote execution immediately.
   * May for example be called by a breakpoint handler.
   */
  public void stopNextInstruction() {
    /* XXX The current implementation will only stop execution when Avrora
     * returns, up to one millisecond after requested simulation time */
    stopNextInstruction = true;
  }

  private MoteInterfaceHandler createMoteInterfaceHandler() {
    return new MoteInterfaceHandler(this, getType().getMoteInterfaceClasses());
  }

  protected void initMote() {
    initEmulator(moteType.getContikiFirmwareFile());
    moteInterfaceHandler = createMoteInterfaceHandler();
  }

  public void setEEPROM(int address, int i) {
    byte[] eedata = EEPROM.getContent();
    eedata[address] = (byte) i;
  }

  public int getID() {
    return getInterfaces().getMoteID().getMoteID();
  }

  public MoteType getType() {
    return moteType;
  }
  public MoteMemory getMemory() {
    return memory;
  }
  public MoteInterfaceHandler getInterfaces() {
    return moteInterfaceHandler;
  }

  private long cyclesExecuted = 0;
  private long cyclesUntil = 0;
  public void execute(long t) {
    /* Wait until mote boots */
    if (moteInterfaceHandler.getClock().getTime() < 0) {
      scheduleNextWakeup(t - moteInterfaceHandler.getClock().getTime());
      return;
    }

    if (stopNextInstruction) {
      stopNextInstruction = false;
      scheduleNextWakeup(t);
      throw new RuntimeException("Avrora requested simulation stop");
    }

    /* Execute one millisecond */
    cyclesUntil += this.getCPUFrequency()/1000;
    while (cyclesExecuted < cyclesUntil) {
      int nsteps = interpreter.step();
      if (nsteps > 0) {
        cyclesExecuted += nsteps;
      } else {
        /* We end up here when watchpoints (probes) are executed */
        /*logger.warn("Avrora did not execute any instruction, aborting executing temporarily");*/
        break;
      }
    }

    /* Schedule wakeup every millisecond */
    /* TODO Optimize next wakeup time */
    scheduleNextWakeup(t + Simulation.MILLISECOND);
  }

  @SuppressWarnings("unchecked")
  public boolean setConfigXML(Simulation simulation, Collection<Element> configXML, boolean visAvailable) {
    setSimulation(simulation);
    initEmulator(moteType.getContikiFirmwareFile());
    moteInterfaceHandler = createMoteInterfaceHandler();

    for (Element element: configXML) {
      String name = element.getName();

      if (name.equals("motetype_identifier")) {
        /* Ignored: handled by simulation */
      } else if ("breakpoints".equals(element.getName())) {
        setWatchpointConfigXML(element.getChildren(), visAvailable);
      } else if (name.equals("interface_config")) {
        Class<? extends MoteInterface> moteInterfaceClass = simulation.getCooja().tryLoadClass(
            this, MoteInterface.class, element.getText().trim());

        if (moteInterfaceClass == null) {
          logger.fatal("Could not load mote interface class: " + element.getText().trim());
          return false;
        }

        MoteInterface moteInterface = getInterfaces().getInterfaceOfType(moteInterfaceClass);
        moteInterface.setConfigXML(element.getChildren(), visAvailable);
      }
    }

    /* Schedule us immediately */
    requestImmediateWakeup();
    return true;
  }

  public Collection<Element> getConfigXML() {
    ArrayList<Element> config = new ArrayList<Element>();
    Element element;

    /* Breakpoints */
    element = new Element("breakpoints");
    element.addContent(getWatchpointConfigXML());
    config.add(element);

    /* Mote interfaces */
    for (MoteInterface moteInterface: getInterfaces().getInterfaces()) {
      element = new Element("interface_config");
      element.setText(moteInterface.getClass().getName());

      Collection<Element> interfaceXML = moteInterface.getConfigXML();
      if (interfaceXML != null) {
        element.addContent(interfaceXML);
        config.add(element);
      }
    }

    return config;
  }

  /* WatchpointMote */
  public Collection<Element> getWatchpointConfigXML() {
    ArrayList<Element> config = new ArrayList<Element>();
    Element element;

    for (AvrBreakpoint breakpoint: watchpoints) {
      element = new Element("breakpoint");
      element.addContent(breakpoint.getConfigXML());
      config.add(element);
    }

    return config;
  }
  public boolean setWatchpointConfigXML(Collection<Element> configXML, boolean visAvailable) {
    for (Element element : configXML) {
      if (element.getName().equals("breakpoint")) {
        AvrBreakpoint breakpoint = new AvrBreakpoint(this);
        if (!breakpoint.setConfigXML(element.getChildren(), visAvailable)) {
          logger.warn("Could not restore breakpoint: " + breakpoint);
        } else {
          watchpoints.add(breakpoint);
        }
      }
    }
    return true;
  }

  private ArrayList<WatchpointListener> watchpointListeners = new ArrayList<WatchpointListener>();
  private ArrayList<AvrBreakpoint> watchpoints = new ArrayList<AvrBreakpoint>();

  public void addWatchpointListener(WatchpointListener listener) {
    watchpointListeners.add(listener);
  }
  public void removeWatchpointListener(WatchpointListener listener) {
    watchpointListeners.remove(listener);
  }
  public WatchpointListener[] getWatchpointListeners() {
    return watchpointListeners.toArray(new WatchpointListener[0]);
  }
  public Watchpoint addBreakpoint(File codeFile, int lineNr, int address) {
    AvrBreakpoint bp = new AvrBreakpoint(this, address, codeFile, new Integer(lineNr));
    watchpoints.add(bp);

    for (WatchpointListener listener: watchpointListeners) {
      listener.watchpointsChanged();
    }
    return bp;
  }
  public void removeBreakpoint(Watchpoint watchpoint) {
    ((AvrBreakpoint)watchpoint).unregisterBreakpoint();
    watchpoints.remove(watchpoint);

    for (WatchpointListener listener: watchpointListeners) {
      listener.watchpointsChanged();
    }
  }
  public AvrBreakpoint[] getBreakpoints() {
    return watchpoints.toArray(new AvrBreakpoint[0]);
  }
  public boolean breakpointExists(int address) {
    if (address < 0) {
      return false;
    }
    for (AvrBreakpoint watchpoint: watchpoints) {
      if (watchpoint.getExecutableAddress() == address) {
        return true;
      }
    }
    return false;
  }
  public boolean breakpointExists(File file, int lineNr) {
    return breakpointExists(getExecutableAddressOf(file, lineNr));
  }

  /* Parse debugging info:
   * Uses avr-add2line and avr-objdump to parse firmware debugging information.
   * This code is inspired by David Kopf's code in now obsolete AvrDebugger.java.
   *
   * These methods are mote type-specific, and may be moved to AvroraMoteType.java. */
  public SourceLocation getSourceLocation(int pc) {
    /* TODO Cache result? */
    try {
      File firmwareFile = ((AvroraMoteType)getType()).getContikiFirmwareFile();
      MessageList output = new MessageList();
      CompileContiki.compile(
          "avr-addr2line -e " + firmwareFile.getName() + " " + Integer.toHexString(pc),
          null, null, firmwareFile.getParentFile(), null, null, output, true);

      for (MessageContainer mc: output.getMessages()) {
        String line = mc.message;
        int last = line.lastIndexOf(':');
        if (last < 0) {
          continue;
        }
        String file = line.substring(0, last).trim();
        String lineNr = line.substring(last+1).trim();
        if (file.equals("??")) {
          return null;
        }
        return new SourceLocation(new File(file), Integer.parseInt(lineNr));
      }
    } catch (Exception e) {
      logger.warn("Error extracting source location: " + e.getMessage(), e);
    }
    return null;
  }

  public int getExecutableAddressOf(File file, int lineNr) {
    try {
      File firmwareFile = ((AvroraMoteType)getType()).getContikiFirmwareFile();
      MessageList output = new MessageList();
      CompileContiki.compile(
          "avr-objdump -l -d " + firmwareFile.getName(),
          null, null, firmwareFile.getParentFile(), null, null, output, true);

      boolean next = false;
      for (MessageContainer mc: output.getMessages()) {
        String line = mc.message;
        if (line.endsWith(file.getName() + ":" + lineNr)) {
          next = true;
          continue;
        }
        if (next) {
          String[] arr = line.trim().split("[: \t]");
          int address = Integer.parseInt(arr[0], 16);
          return address;
        }
      }
    } catch (Exception e) {
      logger.warn("Error extracting executable address: " + e.getMessage(), e);
    }
    return -1;
  }
  

}
