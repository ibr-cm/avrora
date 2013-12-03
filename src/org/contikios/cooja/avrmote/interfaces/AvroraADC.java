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

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Collection;
import java.util.Random;

import javax.swing.Box;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.JTextField;
import javax.swing.JToggleButton;
import javax.swing.Timer;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import org.apache.log4j.Logger;
import org.jdom.Element;

import org.contikios.cooja.ClassDescription;
import org.contikios.cooja.Mote;
import org.contikios.cooja.MoteInterface;
import org.contikios.cooja.MoteTimeEvent;
import org.contikios.cooja.Simulation;
import org.contikios.cooja.avrmote.AvroraMote;
import avrora.sim.AtmelInterpreter;
import avrora.sim.mcu.ADC;
import avrora.sim.mcu.AtmelMicrocontroller;

/**
 * @author David Kopf
 */
@ClassDescription("ADCs")
public class AvroraADC extends MoteInterface {
  private static Logger logger = Logger.getLogger(AvroraADC.class);

  private AvroraMote myMote;
  private Simulation simulation;
  private AtmelInterpreter interpreter;
  private ADC adcDevice;

  private JPanel jPanel;

  final int NUMCHAN = 5;//One extra for Vcc
  final JTextField[] chanMV={null, null, null, null, null, null, null, null};
  final JSlider[] chanSlider={null, null, null, null, null, null, null, null};

  private long[] chanST={0,0,0,0,0,0,0,0};
  private int[] mv={2500,2500,2500,2500,2500,2500,2500,2500};
  private int[] ns={10,10,10,10,10,10,10,10};
  private int[] dc={2500,2500,2500,2500,2500,2500,2500,2500};
  private int[] ac={1000,1000,1000,1000,1000,1000,1000,1000};
  private float[] hz={1000,1000,1000,1000,1000,1000,1000,1000};
  private int[] fn={0,0,0,0,0,0,0,0};
  private ADC.ADCInput[] chanAI={null, null, null, null, null, null, null, null};

  private boolean hasActiveFunctions = false;
  private MoteTimeEvent executeFunctionsTimeEvent;

  private Timer repaintTimer = new Timer(150, new ActionListener() {
    public void actionPerformed(ActionEvent e) {
      updatePanel();
    }
  });

  public AvroraADC(Mote mote) {
    myMote = (AvroraMote) mote;
    simulation = mote.getSimulation();
    AtmelMicrocontroller cpu = (AtmelMicrocontroller) myMote.getPlatform().getMicrocontroller();
    interpreter = (AtmelInterpreter)cpu.getSimulator().getInterpreter();
    adcDevice = (ADC)cpu.getDevice("adc");

    executeFunctionsTimeEvent = new MoteTimeEvent(mote, 0) {
      public void execute(long t) {
        if (!hasActiveFunctions) {
          return;
        }
        executeFunctions(-1);
        simulation.scheduleEvent(this, t + Simulation.MILLISECOND); /* periodic interval */
      }
    };
  }

  private void checkFunctions() {
    boolean hasActivated = false;
    for (int function: fn) {
      if (function != 0) {
        hasActivated = true;
        break;
      }
    }
    if (!hasActivated) {
      hasActiveFunctions = false;
      return;
    }

    /* Make sure time event is scheduled */
    hasActiveFunctions = true;
    if (!executeFunctionsTimeEvent.isScheduled()) {
      simulation.scheduleEvent(executeFunctionsTimeEvent, simulation.getSimulationTime());
    }
  }

  // calculate the voltage on the given channel
  private Random random = new Random();
  private void getVoltage(int i) {
    //long cycleCount = interpreter.getState().getCycles();
    long cycleCount = simulation.getSimulationTimeMillis();
    int mvolts = dc[i];
    if (ns[i] !=0) mvolts += random.nextInt(ns[i]) - ns[i]/2;
    if (fn[i] > 0) {
      float factor = (myMote).getCPUFrequency();
      int advance = (int)(cycleCount-chanST[i]);
      if (fn[i] == 1) {    //sine
        mvolts += (int)(ac[i]*Math.sin(2*3.14159265*hz[i]*advance/factor));
      } else {
        int cycle = (hz[i] >0) ? (int) (factor/hz[i]) : 1;
        if (cycle == 0) cycle=1;
        int phase = advance%cycle;
        switch (fn[i]) {
        case 2:         //square
          if ((phase) < (cycle/2)) {
            mvolts+=ac[i];
          } else {
            mvolts-=ac[i];
          }
          break;
        case 3:         //triangle
          if (phase < (cycle/2)) {
            mvolts+=4*phase*ac[i]/cycle-ac[i];
          } else {
            mvolts+=4*(cycle-phase)*ac[i]/cycle-ac[i];
          }
          break;
        case 4:         //ramp
          mvolts+=2*phase*ac[i]/cycle-ac[i];
          break;
        }
      }
    }
    if (mvolts < 0 ) {
      mvolts = 0;
    } else if (mvolts > 5000) {
      mvolts = 5000;
    }
    mv[i] = mvolts;
  }

  private void executeFunctions(int adc) {
    if (!hasActiveFunctions) {
      return;
    }

    /* Single channel update */
    if (adc >= 0) {
      getVoltage(adc);
    } else {
      /* Functions */
      for (int i=0; i<=NUMCHAN; i++) {
        if (fn[i] > 0) {
          getVoltage(i);
        }
        /*if (i == NUMCHAN) { adcDevice.VCC_LEVEL = (float)(mvolts)/1000; } */
      }
    }
  }

  public void removed() {
    super.removed();
    repaintTimer.stop();
    hasActiveFunctions = false;
  };

  private void updatePanel() {
    for (int i=0; i<=NUMCHAN; i++) {
      if (fn[i] > 0) {
        chanMV[i].setText(""+mv[i]);
        chanSlider[i].setValue(mv[i]);
      }
    }
  }

  public JPanel getInterfaceVisualizer() {
    jPanel = new JPanel(new BorderLayout());
    final Box boxn = Box.createHorizontalBox();
    final Box boxn1 = Box.createHorizontalBox();
    final Box boxn2 = Box.createHorizontalBox();
    final Box boxn3 = Box.createHorizontalBox();
    final Box boxs = Box.createVerticalBox();

    final JTextField[] chanNS={null, null, null, null, null, null, null, null};
    final JTextField[] chanDC={null, null, null, null, null, null, null, null};
    final JTextField[] chanAC={null, null, null, null, null, null, null, null};
    final JTextField[] chanHz={null, null, null, null, null, null, null, null};
    final JButton[] chanFn={null, null, null, null, null, null, null, null};

    Box box = Box.createVerticalBox();
    box.add(new JLabel(" "));
    JLabel label = new JLabel(" ");
    label.setPreferredSize(new Dimension(20,14));
    box.add(label);
    JTextField text = new JTextField("mV");
    box.add(text);text.setEnabled(false);
    text = new JTextField("N");
    box.add(text);text.setEnabled(false);
    text = new JTextField("DC");
    box.add(text);text.setEnabled(false);
    text = new JTextField("AC");
    box.add(text);text.setEnabled(false);
    text = new JTextField("Hz");
    box.add(text);text.setEnabled(false);
    text = new JTextField("Fn");
    text.setPreferredSize(new Dimension(40,20));
    box.add(text);text.setEnabled(false);
    boxn.add(box);

    for (int i=0; i<=NUMCHAN; i++) {
      ADC.ADCInput adcin = new ADC.ADCInput() {
        public float getVoltage() {
          for (int i=0;i<=NUMCHAN;i++) {
            if (this == chanAI[i]) {
              //      if (i == NUMCHAN) logger.debug("Sample Vcc"); else logger.debug("sample ADC"+i);
              executeFunctions(i);
              return ((float)(mv[i]/1000.0));
            }
          }
          return 0;
        }
      };
      adcDevice.connectADCInput(adcin, i);
      chanAI[i] = adcin; //NB: chanAI[NUMCHAN] overwrites default Avrora Vcc input
      box = Box.createVerticalBox();
      if (i==NUMCHAN) box.add(new JLabel("Vcc")); else box.add(new JLabel("ADC" + i));
      chanSlider[i] = new JSlider(0,5000,2500);
      chanSlider[i].setPreferredSize(new Dimension(50,14));
      box.add(chanSlider[i]);
      chanSlider[i].addChangeListener(new ChangeListener() {
        public void stateChanged(ChangeEvent e) {
          JSlider source = (JSlider)e.getSource();
          for (int i=0; i<=NUMCHAN; i++) {
            if (source == chanSlider[i]) {
              mv[i] = source.getValue();
              chanMV[i].setText(""+mv[i]);
              //  if (i == NUMCHAN) adcDevice.VCC_LEVEL = ((float)mv[i])/1000.0f;
              break;
            }
          }
        }
      });

      chanMV[i] = new JTextField("2500");
      chanMV[i].addActionListener(new ActionListener() {
        public void actionPerformed(ActionEvent e) {
          JTextField source = (JTextField)e.getSource();
          for (int i=0; i<=NUMCHAN; i++ ) {
            if (source == chanMV[i]) {
              int tmp = -1;
              try {tmp = Integer.parseInt(source.getText());} catch (Exception x) {};
              if (tmp >=0 && tmp <=5000) {
                mv[i] = tmp;
                chanSlider[i].setValue(tmp);
              } else {
                tmp = mv[i];
              }
              chanMV[i].setText(""+ tmp);
              break;
            }
          }
        }
      });
      box.add(chanMV[i]);

      chanNS[i] = new JTextField("10");
      chanNS[i].addActionListener(new ActionListener() {
        public void actionPerformed(ActionEvent e) {
          JTextField source = (JTextField)e.getSource();
          for (int i=0; i<=NUMCHAN; i++ ) {
            if (source == chanNS[i]) {
              int tmp = -1;
              try {tmp = Integer.parseInt(source.getText());} catch (Exception x) {};
              if (tmp >=0 && tmp <=5000) {
                ns[i] = tmp;
              } else {
                tmp = ns[i];
              }
              chanNS[i].setText(""+ tmp);
              break;
            }
          }
        }
      });;
      box.add(chanNS[i]);

      chanDC[i] = new JTextField("2500");
      chanDC[i].addActionListener(new ActionListener() {
        public void actionPerformed(ActionEvent e) {
          JTextField source = (JTextField)e.getSource();
          for (int i=0; i<=NUMCHAN; i++ ) {
            if (source == chanDC[i]) {
              int tmp = -1;
              try {tmp = Integer.parseInt(source.getText());} catch (Exception x) {};
              if (tmp >=0 && tmp <=5000) {
                dc[i] = tmp;
              } else {
                tmp = dc[i];
              }
              chanDC[i].setText(""+ tmp);
              break;
            }
          }
        }
      });;
      box.add(chanDC[i]);

      chanAC[i] = new JTextField("1000");
      chanAC[i].addActionListener(new ActionListener() {
        public void actionPerformed(ActionEvent e) {
          JTextField source = (JTextField)e.getSource();
          for (int i=0; i<=NUMCHAN; i++ ) {
            if (source == chanAC[i]) {
              int tmp = -1;
              try {
                tmp = Integer.parseInt(source.getText());
              } catch (Exception x) {
                logger.debug(x);
              }
              if (tmp >=0 && tmp <=50000) {
                ac[i] = tmp;
              } else {
                tmp = ac[i];
              }
              chanAC[i].setText(""+ tmp);
              break;
            }
          }
        }
      });
      box.add(chanAC[i]);

      chanHz[i] = new JTextField("1000.0");
      chanHz[i].addActionListener(new ActionListener() {
        public void actionPerformed(ActionEvent e) {
          JTextField source = (JTextField)e.getSource();
          for (int i=0; i<=NUMCHAN; i++ ) {
            if (source == chanHz[i]) {
              float tmp = -1;
              try {tmp = Float.parseFloat(source.getText());} catch (Exception x) {};
              if (tmp >=0) {
                hz[i] = tmp;
              } else {
                tmp = hz[i];
              }
              chanHz[i].setText(""+ tmp);
              break;
            }
          }
        }
      });
      box.add(chanHz[i]);

      chanNS[i].setEnabled(false);
      chanDC[i].setEnabled(false);
      chanAC[i].setEnabled(false);
      chanHz[i].setEnabled(false);
      chanFn[i] = new JButton("None");
      chanFn[i].setPreferredSize(new Dimension(60,20));
      chanFn[i].addActionListener(new ActionListener() {
        public void actionPerformed(ActionEvent e) {
          JButton source = (JButton)e.getSource();
          int c;
          for (c=0; c<=NUMCHAN; c++) {
            if (source == chanFn[c]) {
              break;
            }
          }
          chanST[c] = interpreter.getState().getCycles();

          String text = source.getText();
          if (text.equals("None")) {
            source.setText("Sine");
            fn[c] = 1;
            chanNS[c].setEnabled(true);
            chanDC[c].setEnabled(true);
            chanAC[c].setEnabled(true);
            chanHz[c].setEnabled(true);
          } else if (text.equals("Sine")) {
            source.setText("Square");
            fn[c] = 2;
          } else if (text.equals("Square")) {
            source.setText("Triangle");
            fn[c] = 3;
          } else if (text.equals("Triangle")) {
            source.setText("Ramp");
            fn[c] = 4;
          } else {
            source.setText("None");
            fn[c] = 0;
            chanNS[c].setEnabled(false);
            chanDC[c].setEnabled(false);
            chanAC[c].setEnabled(false);
            chanHz[c].setEnabled(false);
          }
          checkFunctions();

        }
      });
      box.add(chanFn[i]);

      boxn.add(box);
      updatePanel();
    }

    final JButton updateButton = new JButton("Update");
    final JToggleButton liveButton = new JToggleButton("Live", false);

    boxn1.add(Box.createHorizontalGlue());
    boxn2.add(Box.createHorizontalGlue());
    boxn2.add(updateButton);
    boxn2.add(liveButton);
    boxs.add(boxn1);
    boxs.add(boxn2);
    boxs.add(boxn3);
    jPanel.add(BorderLayout.NORTH, boxn);
    jPanel.add(BorderLayout.SOUTH, boxs);

    executeFunctions(-1);
    updatePanel();

    // one time update
    updateButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        updatePanel();
      }
    });

    liveButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        if (liveButton.isSelected()) {
          repaintTimer.start();
        } else {
          repaintTimer.stop();
        }
      }
    });

    return jPanel;
  }

  public void releaseInterfaceVisualizer(JPanel panel) {
    repaintTimer.stop();
    hasActiveFunctions = false;
  }

  public Collection<Element> getConfigXML() {
    return null;
  }

  public void setConfigXML(Collection<Element> configXML, boolean visAvailable) {
  }
}
