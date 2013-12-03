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
import java.io.File;

import org.apache.log4j.Logger;

import org.contikios.cooja.Cooja;
import org.contikios.cooja.MoteInterface;
import org.contikios.cooja.MoteType;
import org.contikios.cooja.Simulation;
import org.contikios.cooja.dialogs.AbstractCompileDialog;

public class AvrCompileDialog extends AbstractCompileDialog {
  private static Logger logger = Logger.getLogger(AvrCompileDialog.class);

  public static boolean showDialog(Container parent, Simulation simulation, MoteType moteType) {

    final AbstractCompileDialog dialog = new AvrCompileDialog(parent, simulation, moteType);

    /* Show dialog and wait for user */
    dialog.setVisible(true); /* BLOCKS */
    if (!dialog.createdOK()) {
      return false;
    }

    /* Assume that if a firmware exists, compilation was ok */
    return true;
  }

  private AvrCompileDialog(Container parent, Simulation simulation, MoteType moteType) {
    super(parent, simulation, moteType);
  }

  public boolean canLoadFirmware(File file) {
    if (file.getName().endsWith(".elf")) {
      return true;
    }
    if (file.getName().endsWith("." + ((AvroraMoteType)moteType).getMoteContikiTarget())) {
      return true;
    }
    return false;
  }

  public String getDefaultCompileCommands(File source) {
    /* TODO Split into String[] */
    return
    /*"make clean TARGET=<target>\n" + */
    Cooja.getExternalToolsSetting("PATH_MAKE") + " " + getExpectedFirmwareFile(source).getName() + " TARGET=" + ((AvroraMoteType)moteType).getMoteContikiTarget();
  }

  public File getExpectedFirmwareFile(File source) {
    return ((AvroraMoteType)moteType).getExpectedFirmwareFile(source);
  }

  public void writeSettingsToMoteType() {
    /* Nothing to do */
  }

  protected String getTargetName() {
  	return ((AvroraMoteType)moteType).getMoteContikiTarget();
  }

  public Class<? extends MoteInterface>[] getDefaultMoteInterfaces() {
    return ((AvroraMoteType)moteType).getMoteInterfaceClasses();
  }

  @Override
  public Class<? extends MoteInterface>[] getAllMoteInterfaces() {
    throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
  }

}
