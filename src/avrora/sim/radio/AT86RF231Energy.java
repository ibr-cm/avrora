
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
    
    public static final int SLEEP     = 0;
    public static final int TRX_OFF   = 1;
    public static final int PLL_ON    = 2;
    public static final int RX_ON_HS  = 3; // RX_ON High sensitivity
    public static final int RX_ON_HI  = 4; // RX_ON High input level
    public static final int RX_ON_LS  = 5; // RX_ON Low sensitivity
    public static final int TX_PWR_15 = 6; // TX_PWR = 15

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
        0.00000002, // SLEEP: 0.02µA
        0.0004,     // TRX_OFF: 0.4mA
        0.0056,     // PLL_ON: 5.6mA
        0.0123,     // RX_ON (High sensitivity): 12.3mA
        0.0103,     // RX_ON (High input level): 10.3mA
        0.0118,     // RX_ON (Low sensitivity):  11.8mA
        0.0074,     // (TX_PWR=15 (-17 dBm) TX_BUSY...
        0.0068,     // (TX_PWR=14 (-12 dBm)
        0.0068,     // (TX_PWR=13 (- 9 dBm)
        0.0068,     // (TX_PWR=12 (- 7 dBm)
        0.0068,     // (TX_PWR=11 (- 5 dBm)
        0.0068,     // (TX_PWR=10 (- 4 dBm)
        0.0068,     // (TX_PWR= 9 (- 3 dBm)
        0.0068,     // (TX_PWR= 8 (- 2 dBm)
        0.0068,     // (TX_PWR= 7 (- 1 dBm)
        0.0116,     // (TX_PWR= 6 (  0 dBm)
        0.0068,     // (TX_PWR= 5 (0.7 dBm)
        0.0068,     // (TX_PWR= 4 (1.3 dBm)
        0.0068,     // (TX_PWR= 3 (1.8 dBm)
        0.0068,     // (TX_PWR= 2 (2.3 dBm)
        0.0068,     // (TX_PWR= 1 (2.8 dBm)
        0.0140      // (TX_PWR= 0 (3.0 dBm)      
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
