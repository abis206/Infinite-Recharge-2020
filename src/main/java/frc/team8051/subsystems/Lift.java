package frc.team8051.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.team8051.Robot;

public class Lift {
    public static void periodic(){
        double maximumOverDrive = 70;
        final WPI_VictorSPX shooterMotor = new WPI_VictorSPX(0);
        if(Robot.getInstance().getOI().joystick.getRawAxis(5) > 0){
            shooterMotor.set(maximumOverDrive);
            maximumOverDrive = maximumOverDrive*0.75;
        }
        if(Robot.getInstance().getOI().joystick.getRawAxis(5) < 0) {
            shooterMotor.set(maximumOverDrive);
            maximumOverDrive = maximumOverDrive;
        }
    }
}
