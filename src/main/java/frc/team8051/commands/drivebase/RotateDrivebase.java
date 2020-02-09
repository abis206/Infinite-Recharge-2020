package frc.team8051.commands.drivebase;

import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import frc.team8051.Robot;
import frc.team8051.sensors.Gyro;


public class RotateDrivebase extends PIDCommand {
    private DifferentialDrive drivebase;
    private double desiredAngle;
    private Gyro gyro;
    private boolean waitFinished;
    private double lastTime;
    private double waitDuration = .350; //seconds 

    public RotateDrivebase(DifferentialDrive drivebase, Gyro gyro, double angle) {
        super(0.05, 0, 0.1);

        this.drivebase = drivebase;
        this.desiredAngle = angle;
        this.gyro = gyro;
    }
 
    public RotateDrivebase(double angle) {
        this(Robot.getInstance().getDifferentialDrive(), Robot.getInstance().getGyro(), angle);
    }
    
    public RotateDrivebase() {
        this(0);
    }

    @Override
    protected void initialize() {
        lastTime = 0;
        waitFinished = false;

        gyro.reset();
        getPIDController().setInputRange(-180, 180);
        getPIDController().setOutputRange(-1, 1);
        getPIDController().setSetpoint(desiredAngle);
        getPIDController().setAbsoluteTolerance(3);
        getPIDController().setContinuous(true);
    }


    @Override
    protected double returnPIDInput() {
        double pidInput = gyro.getHeading();
        // SmartDashboard.putNumber("PID Input", pidInput);
        return pidInput;
    }

    @Override
    protected void usePIDOutput(double output) {
        // System.out.println("<RotateDrivebase> pid output " + output);
        // SmartDashboard.putNumber("PID Output", output);
        drivebase.tankDrive(-output, output);
    }

    @Override
    protected boolean isFinished() {
    
        if(getPIDController().onTarget() && !waitFinished) {
            waitFinished = true;
            lastTime = 	Timer.getFPGATimestamp();
        }

        if(waitFinished && (Timer.getFPGATimestamp() - lastTime) >= waitDuration) {
            return true;
        } 
        return false;
    }

}
