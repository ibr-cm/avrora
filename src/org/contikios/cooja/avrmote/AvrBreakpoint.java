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
 */

package org.contikios.cooja.avrmote;

import java.io.File;

import org.apache.log4j.Logger;

import org.contikios.cooja.Watchpoint;
import org.contikios.cooja.WatchpointMote.WatchpointListener;
import avrora.sim.Simulator;
import avrora.sim.Simulator.Probe;
import avrora.sim.State;
import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import org.contikios.cooja.util.StringUtils;
import org.jdom.Element;

/**
 * Avrora watchpoint.
 *
 * @author Fredrik Osterlind
 */
public class AvrBreakpoint implements Watchpoint {
  private static Logger logger = Logger.getLogger(AvrBreakpoint.class);

  private Probe probe = null;
  private AvroraMote avroraMote;
  private int address = -1;
  private File codeFile = null;
  private int lineNr = -1;
  private Color color = Color.BLACK;
  private String msg = null;
  private String contikiCode = null;
  private boolean stopsSimulation = true;

  
  public AvrBreakpoint(AvroraMote mote) {
    //super(mote);
    this.avroraMote = mote;
    /* expects setConfigXML(..) */
  }

  public AvrBreakpoint(AvroraMote mote, Integer address, File codeFile, Integer lineNr) {
    this(mote);
    this.address = address;
    this.codeFile = codeFile;
    this.lineNr = lineNr;
    
    createMonitor();
  }
  
  @Override
  public AvroraMote getMote() {
    return avroraMote;
  }

  public void registerBreakpoint() {
    probe = new Simulator.Probe.Empty() {
      public void fireBefore(State s, int pc) {
        /* Avrora will return immediately */

        /* Request Cooja to stop executing */
        if (stopsSimulation()) {
          getMote().stopNextInstruction();
        }

        /* Notify listeners */
        WatchpointListener[] listeners = getMote().getWatchpointListeners();
        for (WatchpointListener listener: listeners) {
          listener.watchpointTriggered(AvrBreakpoint.this);
        }
      }
    };
    getMote().sim.insertProbe(probe, getExecutableAddress());
  }

    public void createMonitor() {
    registerBreakpoint();

    /* Remember Contiki code, to verify it when reloaded */
    if (contikiCode == null) {
      final String code = StringUtils.loadFromFile(codeFile);
      if (code != null) {
        String[] lines = code.split("\n");
        if (getLineNumber()-1 < lines.length) {
          contikiCode = lines[lineNr-1].trim();
        }
      }
    }
  }

  
  public void unregisterBreakpoint() {
    getMote().sim.removeProbe(probe, getExecutableAddress());
  }

  @Override
  public Color getColor() {
    return color;
  }

  @Override
  public void setColor(Color color) {
    this.color = color;
  }

  @Override
  public String getDescription() {
    String desc = "";
    if (codeFile != null) {
      desc += codeFile.getPath() + ":" + lineNr + " (0x" + Integer.toHexString(address) + ")";
    } else if (address >= 0) {
      desc += "0x" + Integer.toHexString(address);
    }
    if (msg != null) {
      desc += "\n\n" + msg;
    }
    return desc;
  }



  @Override
  public void setUserMessage(String msg) {
    this.msg = msg;
  }

  @Override
  public String getUserMessage() {
    return msg;
  }

  @Override
  public File getCodeFile() {
    return codeFile;
  }

  @Override
  public int getLineNumber() {
    return lineNr;
  }

  @Override
  public int getExecutableAddress() {
    return address;
  }

  @Override
  public void setStopsSimulation(boolean stops) {
    stopsSimulation = stops;
  }

  @Override
  public boolean stopsSimulation() {
    return stopsSimulation;
  }
  
  
    public Collection<Element> getConfigXML() {
    ArrayList<Element> config = new ArrayList<Element>();
    Element element;

    element = new Element("stops");
    element.setText("" + stopsSimulation);
    config.add(element);

    element = new Element("codefile");
    File file = avroraMote.getSimulation().getCooja().createPortablePath(codeFile);
    element.setText(file.getPath().replaceAll("\\\\", "/"));
    config.add(element);

    element = new Element("line");
    element.setText("" + lineNr);
    config.add(element);

    if (contikiCode != null) {
      element = new Element("contikicode");
      element.setText(contikiCode);
      config.add(element);
    }

    if (msg != null) {
      element = new Element("msg");
      element.setText(msg);
      config.add(element);
    }

    if (color != null) {
      element = new Element("color");
      element.setText("" + color.getRGB());
      config.add(element);
    }

    return config;
  }

  public boolean setConfigXML(Collection<Element> configXML, boolean visAvailable) {
    /* Already knows mote and breakpoints */

    for (Element element : configXML) {
      if (element.getName().equals("codefile")) {
        File file = new File(element.getText());
        file = avroraMote.getSimulation().getCooja().restorePortablePath(file);

        try {
          codeFile = file.getCanonicalFile();
        } catch (IOException e) {
        }

        if (codeFile == null || !codeFile.exists()) {
          return false;
        }
      } else if (element.getName().equals("line")) {
        lineNr = Integer.parseInt(element.getText());
      } else if (element.getName().equals("contikicode")) {
        String lastContikiCode = element.getText().trim();

        /* Verify that Contiki code did not change */
        final String code = StringUtils.loadFromFile(codeFile);
        if (code != null) {
          String[] lines = code.split("\n");
          if (lineNr-1 < lines.length) {
            contikiCode = lines[lineNr-1].trim();
          }
        }

        if (!lastContikiCode.equals(contikiCode)) {
          logger.warn("Detected modified Contiki code at breakpoint: " + codeFile.getPath() + ":" + lineNr + ".");
          logger.warn("From: '" + lastContikiCode + "'");
          logger.warn("  To: '" + contikiCode + "'");
        }
      } else if (element.getName().equals("msg")) {
        msg = element.getText();
      } else if (element.getName().equals("color")) {
        color = new Color(Integer.parseInt(element.getText()));
      } else if (element.getName().equals("stops")) {
        stopsSimulation = Boolean.parseBoolean(element.getText());
      }
    }

    /* Update executable address */
    address = avroraMote.getExecutableAddressOf(codeFile, lineNr);
    if (address < 0) {
      logger.fatal("Could not restore breakpoint, did source code change?");
      return false;
    }
    createMonitor();

    return true;
  }

  
  @Override
  public String toString() {
    return getMote() + ": " + getDescription();
  }
}
