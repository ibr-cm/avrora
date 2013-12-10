/**
 * Copyright (c) 2007, Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the University of California, Los Angeles nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Created Feb 01, 2010
 */
 
package avrora.sim.radio;

import avrora.Defaults;
import avrora.sim.Simulation;
import avrora.sim.Simulator;
import avrora.sim.clock.MainClock;
import avrora.sim.output.SimPrinter;
import avrora.sim.types.SensorSimulation;

import cck.text.StringUtil;
import cck.util.Option;
import cck.util.Util;

import java.util.*; 

/**
 * A simple mobility model
 *
 * @author Daniel Minder
 */
public class TopologyRWP extends TopologyStatic {
    public final Option.Double MOBILITY_MINX = newOption("mobility-minX", Double.MAX_VALUE, "Minimum X coordinate in m");
    public final Option.Double MOBILITY_MAXX = newOption("mobility-maxX", Double.MIN_VALUE, "Maximum X coordinate in m");
    public final Option.Double MOBILITY_MINY = newOption("mobility-minY", Double.MAX_VALUE, "Minimum Y coordinate in m");
    public final Option.Double MOBILITY_MAXY = newOption("mobility-maxY", Double.MIN_VALUE, "Maximum Y coordinate in m");
    public final Option.Double MOBILITY_MINZ = newOption("mobility-minZ", Double.MAX_VALUE, "Minimum Z coordinate in m");
    public final Option.Double MOBILITY_MAXZ = newOption("mobility-maxZ", Double.MIN_VALUE, "Maximum Z coordinate in m");
    public final Option.Double MOBILITY_MINVEL = newOption("mobility-minvel", 0.0, "Minimum velocity in m/s");
    public final Option.Double MOBILITY_MAXVEL = newOption("mobility-maxvel", 2.777777, "Maximum velocity in m/s");
    public final Option.Double MOBILITY_MAXWAIT = newOption("mobility-maxwait", 0.0, "Maximum wait time between two movements in s");
    public final Option.Double MOBILITY_GRANULARITY = newOption("mobility-granularity", 0.5, "Granularity of the movement in m");
    public final Option.List MOBILE_NODES = newOptionList("mobile-nodes", "", "IDs of the mobile nodes (comma-separated list)");
  
    private int[] mobileNodeIds;
  
    private double minX, maxX, minY, maxY, minZ, maxZ, minVelocity, maxVelocity, maxWait, granularity;

    private static Random rand = Defaults.getSimulation("sensor-network").getRandom();
    
    /**
     * @param o the options representing the known and unknown options from the command line
      */
    public TopologyRWP() {
        super("Random waypoint mobility model. Starting topology can be given as for static topology. "
            +"When doing so, the min/max coordinates are automatically adjusted so that they at least cover all nodes. By default, "
            +"the ranges are negativ which will lead to an error when no start topology and no min/max settings are given. "
            +"All nodes given by the mobile-nodes list will travel with speed between mobility-minvel and mobility-maxvel. "
            +"After reaching the destination they will wait a random time between 0 and mobility-maxwait before they select a new destination."
            +"The positions of the nodes are update with the given granularity. Note that a finer granularity will result in higher computational "
            +"effort and, therefore, higher simulation time.");
    }

    protected class NodeMover implements Simulator.Event {
        private Position nodepos;
        private MainClock clock;
        private SimPrinter printer;
        private double dirX, dirY, dirZ, newX, newY, newZ;
        protected long eventCycles;
        private long allCycles, currStep;
        private boolean wasWaiting;
        
        public NodeMover(Simulation.Node n, Position p) {
            clock = n.getSimulator().getClock();
            nodepos = p;
            wasWaiting = false;
            printer = n.getSimulator().getPrinter("topology");
            setNewPosition();
        }
        
        private void setNewPosition() {
            // calculate the final position and the single steps to reach it
            double distance = 0.0;
            while (distance == 0.0) {
                newX = minX+rand.nextDouble()*(maxX-minX);
                newY = minY+rand.nextDouble()*(maxY-minY);
                newZ = minZ+rand.nextDouble()*(maxZ-minZ);
                dirX= newX - nodepos.x;
                dirY = newY - nodepos.y;
                dirZ = newZ - nodepos.z;
                distance = Math.sqrt(dirX*dirX + dirY*dirY + dirZ*dirZ);
            }
            double steps = distance / granularity;
            dirX /= steps;
            dirY /= steps;
            dirZ /= steps;
            
            double velocity = 0.0;
            while (velocity == 0.0) {
                velocity = minVelocity + rand.nextDouble()*maxVelocity;  // maxVel is actually the delta, see start()
            }
            eventCycles = clock.millisToCycles(1000*granularity/velocity);
            if (eventCycles == 0) {
                eventCycles = 1;  // minimum cycles
            }
            allCycles = clock.millisToCycles(1000*distance/velocity);
            if (allCycles == 0) {
                allCycles = 1;  // minimum cycles
            }
            if (allCycles < eventCycles) {
                eventCycles = allCycles;
            }
            if (printer != null) {
                printer.println("Destination: "+newX+"/"+newY+"/"+newZ+" at v="+velocity);
            }
        }
            
        public void fire() {
            if (wasWaiting) {
                // node was just waiting, no movement in this step
                wasWaiting = false;
                clock.insertEvent(this, eventCycles);
                return;
            }
            
            allCycles -= eventCycles;
            if (allCycles <= 0) {
                // we reached the final destination
                nodepos.x = newX;
                nodepos.y = newY;
                nodepos.z = newZ;
                // set new position
                setNewPosition();
                // check if we may wait
                if (maxWait > 0.0) {
                    double waitTime = rand.nextDouble() * maxWait;
                    if (printer != null) {
                        printer.println("Waiting for "+waitTime);
                    }
                    wasWaiting = true;
                    clock.insertEvent(this, clock.millisToCycles(1000 * waitTime));
                    return;
                }
                clock.insertEvent(this, eventCycles);
            }
            else {
                // make one step
                nodepos.x += dirX;
                nodepos.y += dirY;
                nodepos.z += dirZ;
                if (allCycles < eventCycles) {
                    eventCycles = allCycles;
                }
                clock.insertEvent(this, eventCycles);
                if (printer != null) {
                    printer.println("New position: "+nodepos.x+"/"+nodepos.y+"/"+nodepos.z);
                }
            }
        }
    }
    
    public void addNode(Simulation.Node node) {
        super.addNode(node);
        
        // add a position if it is not known
        if (nodes.size() > positions.size()) {
            positions.add(new Position(minX+rand.nextDouble()*(maxX-minX), minY+rand.nextDouble()*(maxY-minY), minZ+rand.nextDouble()*(maxZ-minZ), 0.0));
        }

        // check if the current node is a mobile one
        int nodenr = nodes.size()-1;  // nr of current node
        for (int i=0; i<mobileNodeIds.length; i++) {
            if (mobileNodeIds[i] == nodenr) {
                NodeMover nm = new NodeMover(node, (Position)positions.get(nodenr));
                node.getSimulator().getClock().insertEvent(nm, nm.eventCycles);
                break;
            }
        }
    }

    public void start() {
        minX = MOBILITY_MINX.get();
        maxX = MOBILITY_MAXX.get();
        minY = MOBILITY_MINY.get();
        maxY= MOBILITY_MAXY.get();
        minZ = MOBILITY_MINZ.get();
        maxZ = MOBILITY_MAXZ.get();
        minVelocity = MOBILITY_MINVEL.get();
        maxVelocity = MOBILITY_MAXVEL.get();
        maxWait = MOBILITY_MAXWAIT.get();
        granularity = MOBILITY_GRANULARITY.get();
        
        if (minVelocity < 0) {
            Util.userError("Minimum velocity must be >= 0!");
        }
        if (maxVelocity <= 0) {
            Util.userError("Maximum velocity must be > 0!");
        }
        if (maxVelocity < minVelocity) {
            Util.userError("Maximum velocity must be >= minimum velocity!");
        }
        maxVelocity -= minVelocity;  // calculate delta to ease later calculations
          
        if (!TOPOLOGY_FILE.isBlank()) {
            parseFile();
            // adjust the minimum and maximum coordinates
            Iterator i = positions.iterator();
            while (i.hasNext()) {
                Position p = (Position)i.next();
                if (p.x < minX) {
                    minX = p.x;
                }
                if (p.x > maxX) {
                    maxX = p.x;
                }
                if (p.y < minY) {
                    minY = p.y;
                }
                if (p.y > maxY) {
                    maxY = p.y;
                }
                if (p.z < minZ) {
                    minZ = p.z;
                }
                if (p.z > maxZ) {
                    maxZ = p.z;
                }
            }
        }
        
        // check the min/max values
        if (minX > maxX) {
            Util.userError("X coordinate range wrong (topology file given and/or MINX/MAXX set?)");
        }
        if (minY > maxY) {
            Util.userError("Y coordinate range wrong (topology file given and/or MINY/MAXY set?)");
        }
        if (minZ > maxZ) {
            Util.userError("Z coordinate range wrong (topology file given and/or MINZ/MAXZ set?)");
        }
        if (granularity == 0.0) {
            Util.userError("Granularity cannot be 0");
        }
        
        // we can use this model also for just a random start topology, but no mobility...
        mobileNodeIds = new int[MOBILE_NODES.get().size()];
        int cnt = 0;
        Iterator iter = MOBILE_NODES.get().iterator();
        while (iter.hasNext()) {
            String currentNode = (String) iter.next();
            mobileNodeIds[cnt] = StringUtil.evaluateIntegerLiteral(currentNode.trim());
            cnt++;
        }
    }

}
