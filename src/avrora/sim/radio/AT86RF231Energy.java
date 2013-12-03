
package avrora.sim.radio;

import cck.text.StringUtil;
/**
 * Constants for Atmel AT86RF231 radio energy consumption
 * 
 * @author David Kopf
 */
public abstract class AT86RF231Energy {  
   
    /**
     * <code>startMode</code> the default operating mode after turning on / reset
     */
    public static final int startMode = 0;

    /**
     * <code>modeName</code> names of the operating modes
     */
    public static final String[] modeName = {
        "Power Off:            ",
        "Power Down:           ",
        "Idle:              ",
        "Receive (Rx):         ",
        "Receive HL (Rx):         ",
        "Receive LS (Rx):         ",
        "Transmit (Tx):        "
    };


    /**
     * <code>modeAmpere</code> power consumption of the operating modes
     */
    //3.3 volts @ 25C
    public static final double[] modeAmpere = {                                             
        0.00000001,//Sleep 10nA
        0.000420,  //TRX_OFF
        0.000550,  //P_ON, PLL_ON
        0.0124,   //RX_ON High sensitivity
        0.0119,   //RX_ON High input level
        0.0103,   //RX_ON Low sensitivity
        0.0075,   // (TX_BUSY, TX_PWR=15 (-17 dBm)
        0.0068,   // (TX_PWR=14 (-12 dBm)
        0.0068,   // (TX_PWR=13 (- 9 dBm)
        0.0068,   // (TX_PWR=12 (- 7 dBm)
        0.0068,   // (TX_PWR=11 (- 5 dBm)
        0.0068,   // (TX_PWR=10 (- 4 dBm)
        0.0068,   // (TX_PWR= 9 (- 3 dBm)
        0.0068,   // (TX_PWR= 8 (- 2 dBm)
        0.0068,   // (TX_PWR= 7 (- 1 dBm)
        0.0118,   // (TX_PWR= 6 (  0 dBm)
        0.0068,   // (TX_PWR= 5 (0.7 dBm)
        0.0068,   // (TX_PWR= 4 (1.3 dBm)
        0.0068,   // (TX_PWR= 3 (1.8 dBm)
        0.0068,   // (TX_PWR= 2 (2.3 dBm)
        0.0068,   // (TX_PWR= 1 (2.8 dBm)
        0.0141    // (TX_PWR= 0 (3.0 dBm)      
    };

    public static String[] allModeNames() {
        String[] modeName = new String[16+6];

        System.arraycopy(AT86RF231Energy.modeName, 0, modeName, 0, 6);
        for (int i = 0; i < 16; i++) {
            modeName[i + 4] = AT86RF231Energy.modeName[4] + StringUtil.leftJustify(i+":  ", 2);
        }
        return modeName;
    }
}
