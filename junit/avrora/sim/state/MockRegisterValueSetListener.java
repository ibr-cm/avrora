package avrora.sim.state;

/**
 * Mocked RegisterValueSetListener
 * 
 * @author Matthias Linder
 */
public class MockRegisterValueSetListener implements RegisterView.RegisterValueSetListener {
    
    public int called;
    public int oldValue;
    public int newValue;
    
    public void onValueSet(RegisterView view, int oldValue, int newValue) {
        called++;
        this.oldValue = oldValue;
        this.newValue = newValue;
    }
}
