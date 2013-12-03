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

import java.awt.Container;
import java.awt.Image;
import java.awt.MediaTracker;
import java.awt.Toolkit;
import java.io.File;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collection;

import javax.swing.Icon;
import javax.swing.ImageIcon;
import javax.swing.JComponent;

import org.apache.log4j.Logger;

import org.contikios.cooja.AbstractionLevelDescription;
import org.contikios.cooja.ClassDescription;
import org.contikios.cooja.Cooja;
import org.contikios.cooja.Mote;
import org.contikios.cooja.MoteInterface;
import org.contikios.cooja.MoteInterfaceHandler;
import org.contikios.cooja.MoteMemory;
import org.contikios.cooja.MoteType;
import org.contikios.cooja.ProjectConfig;
import org.contikios.cooja.Simulation;
import org.contikios.cooja.dialogs.CompileContiki;
import org.contikios.cooja.dialogs.MessageList;
import org.contikios.cooja.dialogs.MessageList.MessageContainer;
import org.contikios.cooja.motes.AbstractEmulatedMote;
import org.jdom.Element;

/**
 * AVR-based mote types emulated in Avrora.
 *
 * @author Joakim Eriksson, Fredrik Osterlind, David Kopf
 */
@ClassDescription("Avrora Mote Type")
@AbstractionLevelDescription("Emulated level")
public abstract class AvroraMoteType extends AbstractEmulatedMote implements MoteType {
  public static Logger logger = Logger.getLogger(AvroraMoteType.class);

  private Class<? extends MoteInterface>[] moteInterfaceClasses = null;

  protected Simulation simulation;
  protected String identifier = null;
  protected String description = null;


  /* If source file is defined, the firmware is recompiled when loading simulations */
  private File fileFirmware = null;
  private File fileSource = null;
  private String compileCommands = null;


  public abstract String getMoteName();
  public abstract String getMoteContikiTarget();
  public abstract Class<? extends MoteInterface>[] getAllMoteInterfaceClasses();
  public abstract Mote generateMote(Simulation simulation);

  public boolean configureAndInit(Container parentContainer, Simulation simulation, boolean visAvailable)
  throws MoteTypeCreationException {
    this.simulation = simulation;
    String moteName = this.getMoteName();
    /* If visualized, show compile dialog and let user configure */
    if (visAvailable) {

      /* Create unique identifier */
      if (getIdentifier() == null) {
        int counter = 0;
        boolean identifierOK = false;
        while (!identifierOK) {
          identifierOK = true;

          counter++;
          setIdentifier(moteName + counter);

          for (MoteType existingMoteType : simulation.getMoteTypes()) {
            if (existingMoteType == this) {
              continue;
            }
            if (existingMoteType.getIdentifier().equals(getIdentifier())) {
              identifierOK = false;
              break;
            }
          }
        }
      }

      /* Create initial description */
      if (getDescription() == null) {
        setDescription(moteName +" Mote Type #" + getIdentifier());
      }

      return AvrCompileDialog.showDialog(parentContainer, simulation, this);
    }

    /* Not visualized: Compile Contiki immediately */
    if (getIdentifier() == null) {
      throw new MoteTypeCreationException("No identifier");
    }

    final MessageList compilationOutput = new MessageList();

    if (getCompileCommands() != null) {
      /* Handle multiple compilation commands one by one */
      String[] arr = getCompileCommands().split("\n");
      for (String cmd: arr) {
        if (cmd.trim().isEmpty()) {
          continue;
        }

        try {
          CompileContiki.compile(
              cmd,
              null,
              null /* Do not observe output firmware file */,
              getContikiSourceFile().getParentFile(),
              null,
              null,
              compilationOutput,
              true
          );
        } catch (Exception e) {
          MoteTypeCreationException newException =
            new MoteTypeCreationException("Mote type creation failed: " + e.getMessage());
          newException = (MoteTypeCreationException) newException.initCause(e);
          newException.setCompilationOutput(compilationOutput);

          /* Print last 10 compilation errors to console */
          MessageContainer[] messages = compilationOutput.getMessages();
          for (int i=messages.length-10; i < messages.length; i++) {
            if (i < 0) {
              continue;
            }
            logger.fatal(">> " + messages[i]);
          }

          logger.fatal("Compilation error: " + e.getMessage());
          throw newException;
        }
      }
    }

    if (getContikiFirmwareFile() == null ||
        !getContikiFirmwareFile().exists()) {
      throw new MoteTypeCreationException("Contiki firmware file does not exist: " + getContikiFirmwareFile());
    }
    return true;
  }

  public Icon getMoteTypeIcon() {
    Toolkit toolkit = Toolkit.getDefaultToolkit();
    URL imageURL = this.getClass().getClassLoader().getResource("images/"+this.getMoteName()+".jpg");
    if (imageURL == null) return null;
    Image image = toolkit.getImage(imageURL);
    MediaTracker tracker = new MediaTracker(Cooja.getTopParentContainer());
    tracker.addImage(image, 1);
    try {
      tracker.waitForAll();
    } catch (InterruptedException ex) {
    }
    if (image.getHeight(Cooja.getTopParentContainer()) > 0 && image.getWidth(Cooja.getTopParentContainer()) > 0) {
      image = image.getScaledInstance((200*image.getWidth(Cooja.getTopParentContainer())/image.getHeight(Cooja.getTopParentContainer())), 200, Image.SCALE_DEFAULT);
      return new ImageIcon(image);
    }
    return null;
  }

  public File getExpectedFirmwareFile(File source) {
    File parentDir = source.getParentFile();
    String sourceNoExtension = source.getName().substring(0, source.getName().length()-2);
    return new File(parentDir, sourceNoExtension + "." + getMoteContikiTarget());
  }
  
  

  @Override
  public Collection<Element> getConfigXML(Simulation simulation) {
    ArrayList<Element> config = new ArrayList<Element>();

    Element element;

    // Identifier
    element = new Element("identifier");
    element.setText(getIdentifier());
    config.add(element);

    // Description
    element = new Element("description");
    element.setText(getDescription());
    config.add(element);

    // Source file
    if (fileSource != null) {
      element = new Element("source");
      File file = simulation.getCooja().createPortablePath(fileSource);
      element.setText(file.getPath().replaceAll("\\\\", "/"));
      config.add(element);
      element = new Element("commands");
      element.setText(compileCommands);
      config.add(element);
    }

    // Firmware file
    element = new Element("firmware");
    File file = simulation.getCooja().createPortablePath(fileFirmware);
    element.setText(file.getPath().replaceAll("\\\\", "/"));
    config.add(element);

    // Mote interfaces
    for (Class<? extends MoteInterface> moteInterface : getMoteInterfaceClasses()) {
      element = new Element("moteinterface");
      element.setText(moteInterface.getName());
      config.add(element);
    }

    return config;
  }


  


  @Override
  public String getDescription() {
    return description;
  }

  @Override
  public void setDescription(String description) {
    this.description = description;
  }

  @Override
  public String getIdentifier() {
    return identifier;
  }

  @Override
  public void setIdentifier(String identifier) {
    this.identifier = identifier;
  }

  @Override
  public File getContikiSourceFile() {
    return fileSource;
  }

  @Override
  public void setContikiSourceFile(File file) {
    this.fileSource = file;
  }

  @Override
  public File getContikiFirmwareFile() {
    return fileFirmware;
  }

  @Override
  public void setContikiFirmwareFile(File file) {
    this.fileFirmware = file;
  }

  @Override
  public String getCompileCommands() {
    return compileCommands;
  }

  @Override
  public void setCompileCommands(String commands) {
    this.compileCommands = commands;
  }


  @Override
  public void setMoteInterfaceClasses(Class<? extends MoteInterface>[] classes) {
    this.moteInterfaceClasses = classes;
  }

  

  @Override
  public Class<? extends MoteInterface>[] getMoteInterfaceClasses() {
    return moteInterfaceClasses;
  }

    @Override
  public void execute(long time) {
    throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
  }

  @Override
  public int getID() {
    throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
  }

  @Override
  public MoteInterfaceHandler getInterfaces() {
    throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
  }

  @Override
  public MoteMemory getMemory() {
    throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
  }

  @Override
  public MoteType getType() {
    throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
  }

  @Override
  public Collection<Element> getConfigXML() {
    throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
  }

  @Override
  public boolean setConfigXML(Simulation simulation, Collection<Element> configXML, boolean visAvailable) {
    throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
  }

  @Override
  public JComponent getTypeVisualizer() {
    throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
  }

  @Override
  public ProjectConfig getConfig() {
     logger.warn("MicaZ mote type project config not implemented");
    return null;

  }
}
