/*
Code for controller
 */

package frc.team8051.services;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;

public class OI {
    private final int DRIVER_JOYSTICK = 0;
    private final int OPERATOR_JOYSTICK = 1;
    private final Joystick[] joysticks = new Joystick[2];

    public class JoystickMap {
        public static final int BUTTON_A = 1;
        public static final int BUTTON_B = 2;
        public static final int BUTTON_X = 3;
        public static final int BUTTON_Y = 4;
        public static final int BUTTON_LB = 5;
        public static final int BUTTON_RB = 6;
        public static final int BUTTON_BACK = 7;
        public static final int BUTTON_START = 8;
    }

    public OI() {
        joysticks[DRIVER_JOYSTICK] = new Joystick(DRIVER_JOYSTICK);
        joysticks[OPERATOR_JOYSTICK] = new Joystick(OPERATOR_JOYSTICK);
    }

    public void initializeBind() {

    }

    public double getRightXAxis(Joystick joystick) { 
        return joystick.getRawAxis(4); 
    }

    public double getRightYAxis(Joystick joystick) {
        return joystick.getRawAxis(5);
    }
    
    public double getLeftXAxis(Joystick joystick) {
        return joystick.getRawAxis(0);
    }

    public double getLeftYAxis(Joystick joystick) {
        return joystick.getRawAxis(1);
    }

    public JoystickButton getJoystickButton(Joystick joystick, int buttonNumber) {
        return new JoystickButton(joystick, buttonNumber);
    }

    public Joystick getOpertorJoystick() {
        return joysticks[OPERATOR_JOYSTICK];
    }

    public Joystick getDriverJoystick() {
        return joysticks[DRIVER_JOYSTICK];
    }
}
