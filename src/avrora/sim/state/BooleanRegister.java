/**
 * Copyright (c) 2007, Ben L. Titzer
 * See the file "license.txt" for details.
 *
 * Created Nov 4, 2007
 */
package avrora.sim.state;

/**
 * The <code>BooleanRegister</code> definition.
 *
 * @author Ben L. Titzer
 */
public class BooleanRegister implements BooleanView {

    protected ValueSetListener listener;
    protected boolean value;


    public boolean getValue() {
        return value;
    }

    public void setValue(boolean v) {
        value = v;
        if (listener != null) {
            listener.onValueSet(this, v);
        }
    }

    public void setValueSetListener(ValueSetListener listener) {
        this.listener = listener;
    }
}
