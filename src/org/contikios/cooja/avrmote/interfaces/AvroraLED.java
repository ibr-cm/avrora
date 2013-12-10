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

package org.contikios.cooja.avrmote.interfaces;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.util.Collection;
import java.util.Observable;
import java.util.Observer;

import javax.swing.JPanel;

import org.apache.log4j.Logger;
import org.jdom.Element;

import org.contikios.cooja.ClassDescription;
import org.contikios.cooja.Mote;
import org.contikios.cooja.avrmote.AvroraMote;
import org.contikios.cooja.interfaces.LED;
import avrora.sim.FiniteStateMachine;

/**
 * @author Joakim Eriksson
 * @author David Kopf
 */
@ClassDescription("LEDs")
public class AvroraLED extends LED {
  private static Logger logger = Logger.getLogger(AvroraLED.class);

  private avrora.sim.FiniteStateMachine.Probe[] ledProbes = {null, null, null, null};
  private avrora.sim.platform.LED.LEDGroup leds;

  private boolean probesInserted = false;
  private boolean[] ledOn = {false, false, false, false};
  private int[] ledMap = new int[4];

  private static final int LED_RED = 0;
  private static final int LED_GREEN = 1;
  private static final int LED_BLUE = 2;
  private static final int LED_YELLOW = 3;

  private static final Color DARK_BLUE = new Color(0, 0, 100);
  private static final Color DARK_GREEN = new Color(0, 100, 0);
  private static final Color DARK_RED = new Color(100, 0, 0);
  private static final Color DARK_YELLOW = new Color(130, 100, 0);
  private static final Color BLUE = new Color(0, 0, 255);
  private static final Color GREEN = new Color(0, 255, 0);
  private static final Color RED = new Color(255, 0, 0);
  private static final Color YELLOW = new Color(220, 200, 0);

  public AvroraLED(Mote mote) {
    leds = (avrora.sim.platform.LED.LEDGroup) ((AvroraMote)mote).getPlatform().getDevice("leds");

    int index = 0;
    for ( avrora.sim.platform.LED led : leds.leds) {
      switch (led.color) {
        case "Red":
          ledMap[index++] = LED_RED;
          break;
        case "Green":
          ledMap[index++] = LED_GREEN;
          break;
        case "Blue":
          ledMap[index++] = LED_BLUE;
          break;
        case "Yellow":
          
          ledMap[index++] = LED_YELLOW;
          //ledMap[index++] = LED_BLUE;  //make blue for cooja timeline
          break;
        default:
          logger.debug("LED " + led.color + " not available");
          break;
      }
        if (index > 3) {
            logger.debug(" Ignoring extra LEDs ");
             break;
        }
    }

    if (index-- >=0) {
        ledProbes[0] = new FiniteStateMachine.Probe() {
          @Override
          public void fireAfterTransition(int old, int newstate) {
            ledOn[ledMap[0]] = newstate > 0;
            setChanged();
            notifyObservers();
          }
          @Override
          public void fireBeforeTransition(int arg0, int arg1) {
          }
        };
    }
    if (index-- >=0) {
        ledProbes[1] = new FiniteStateMachine.Probe() {
          @Override
          public void fireAfterTransition(int old, int newstate) {
            ledOn[ledMap[1]] = newstate > 0;
            setChanged();
            notifyObservers();
          }
          @Override
          public void fireBeforeTransition(int arg0, int arg1) {
          }
        };
    }
    if (index-- >=0) {
        ledProbes[2] = new FiniteStateMachine.Probe() {
          @Override
          public void fireAfterTransition(int old, int newstate) {
            ledOn[ledMap[2]] = newstate > 0;
            setChanged();
            notifyObservers();
          }
          @Override
          public void fireBeforeTransition(int arg0, int arg1) {
          }
        };
    }
    if (index-- >=0) {
        ledProbes[3] = new FiniteStateMachine.Probe() {
          @Override
          public void fireAfterTransition(int old, int newstate) {
            ledOn[ledMap[3]] = newstate > 0;
            setChanged();
            notifyObservers();
          }
          @Override
          public void fireBeforeTransition(int arg0, int arg1) {
          }
        };
    }

    //led probe always active for cooja timeline
    for (int i = 0; i < leds.leds.length; i++) {
        leds.leds[i].getFSM().insertProbe(ledProbes[i]);
    }
    probesInserted = true;

  }

  @Override
  public void removed() {
    super.removed();
    /* TODO Remove probes */
  }

  @Override
  public boolean isAnyOn() {
    for (int i=0; i<4; i++) if (ledOn[i]) return true;
    return false;
  }

  public boolean isblueOn() {
   // return ledOn[LED_YELLOW]; //return state of yellow for timeline
    return ledOn[LED_BLUE];
  }

  @Override
  public boolean isGreenOn() {
    return ledOn[LED_GREEN];
  }

  @Override
  public boolean isRedOn() {
    return ledOn[LED_RED];
  }

  @Override
  public boolean isYellowOn()  {
    return ledOn[LED_YELLOW];
  }

  @Override
  public JPanel getInterfaceVisualizer() {
    final JPanel panel = new JPanel() {
      @Override
      public void paintComponent(Graphics g) {
        super.paintComponent(g);
/*
        if (!probesInserted) {
            for (int i=0;i<leds.leds.length;i++) {
                leds.leds[i].getFSM().insertProbe(ledProbes[i]);
                ledOn[ledMap[i]] = leds.leds[i].getFSM().getCurrentState());
            }
            probesInserted = true;
        }
*/
        int x = 20;
        int y = 4;
        int d = 25;
        for (int i = 0; i < leds.leds.length; i++) {
            if (ledOn[ledMap[i]]) {
                switch (ledMap[i]) {
                    case LED_BLUE:
                      g.setColor(BLUE);
                      break;
                    case LED_GREEN:
                      g.setColor(GREEN);
                      break;
                    case LED_RED:
                      g.setColor(RED);
                      break;
                    case LED_YELLOW:
                      g.setColor(YELLOW);
                      break;
                }
                g.fillOval(x, y, d, d);
                g.setColor(Color.BLACK);
                g.drawOval(x, y, d, d);
            } else {
                switch (ledMap[i]) {
                    case LED_BLUE:
                      g.setColor(DARK_BLUE);
                      break;
                    case LED_GREEN:
                      g.setColor(DARK_GREEN);
                      break;
                    case LED_RED:
                      g.setColor(DARK_RED);
                      break;
                    case LED_YELLOW:
                      g.setColor(DARK_YELLOW);
                      break;
                }
                g.fillOval(x + 5, y + 5, d - 10, d - 10);
            }
            x += 40;
        }
      }
    };

    Observer observer;
    this.addObserver(observer = new Observer() {
      @Override
      public void update(Observable obs, Object obj) {
        panel.repaint();
      }
    });

    // Saving observer reference for releaseInterfaceVisualizer
    panel.putClientProperty("intf_obs", observer);

    panel.setPreferredSize(new Dimension(140, 40));

    return panel;
  }

  @Override
  public void releaseInterfaceVisualizer(JPanel panel) {
    Observer observer = (Observer) panel.getClientProperty("intf_obs");
    if (observer == null) {
      logger.fatal("Error when releasing panel, observer is null");
      return;
    }
    //leave probes inserted for cooja timeline
/*
    for (int i=0;i<leds.leds.length;i++) {
        leds.leds[i].getFSM().removeProbe(ledProbes[i]);
    }
    probesInserted = false;
*/
    this.deleteObserver(observer);
  }


  @Override
  public Collection<Element> getConfigXML() {
    return null;
  }

  @Override
  public void setConfigXML(Collection<Element> configXML, boolean visAvailable) {
  }

}

