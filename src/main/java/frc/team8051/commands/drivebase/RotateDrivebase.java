package frc.team8051.commands.drivebase;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.team8051.Robot;
import frc.team8051.sensors.Gyro;


public class RotateDrivebase extends PIDCommand {
    private DifferentialDrive drivebase;
    private double desiredAngle;
    private Gyro gyro;
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
        gyro.reset();
        
        getPIDController().setPIDSourceType(PIDSourceType.kDisplacement);
        getPIDController().setInputRange(-360, 360);
        getPIDController().setOutputRange(-1, 1);
        getPIDController().setSetpoint(desiredAngle);
        getPIDController().setAbsoluteTolerance(2);
        getPIDController().setContinuous(true);
    }


    @Override
    protected double returnPIDInput() {
        // what if gyro reads beyond 180 ? or 360 ?
        
        // System.out.println("<RotateDrivebase> gyro pid value " + gyroValue);
        // System.out.println("<RotateDrivebase> gyro angle value" + gyro.getAngle());
        // System.out.println("<RotateDrivebase> set point value " + getPIDController().getSetpoint());
        
        return gyro.pidGet();
    }

    @Override
    protected void usePIDOutput(double output) {
        // System.out.println("<RotateDrivebase> pid output " + output);
         drivebase.tankDrive(output, -output);
    }

    @Override
    protected boolean isFinished() {
        return getPIDController().onTarget();
    }

}
