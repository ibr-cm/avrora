package avrora.sim.state;

import java.util.LinkedList;

/** 
 * Abstract implementation of the {@link RegisterView} which implements event handling.
 */
public abstract class AbstractRegisterView implements RegisterView {
    private LinkedList<RegisterValueSetListener> listeners;

    /** Notifies all registered listeners of a changed value. */
    protected void notify(int oldValue, int newValue) {
        if (listeners != null) {
            for (RegisterValueSetListener l : listeners) {
                l.onValueSet(this, oldValue, newValue);
            }
        }
    }

    public void registerValueSetListener(RegisterValueSetListener listener) {
        if (listeners == null) {
            listeners = new LinkedList<RegisterValueSetListener>();
        }
        listeners.add(listener);
    }

}
