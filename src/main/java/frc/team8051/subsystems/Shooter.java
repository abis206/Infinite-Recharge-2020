package frc.team8051.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.team8051.Robot;

public class Shooter {
    public static void periodic(){
        final WPI_VictorSPX shooterMotor = new WPI_VictorSPX(0);
        if(Robot.getInstance().getOI().joystick.getRawButton(1)){
            shooterMotor.set(0);
        }
    }
}
