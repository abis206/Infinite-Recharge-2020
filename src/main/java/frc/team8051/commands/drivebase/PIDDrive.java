package frc.team8051.commands.drivebase;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team8051.Robot;
import frc.team8051.sensors.DrivebaseEncoder;
import frc.team8051.sensors.Gyro;

public class PIDDrive extends PIDCommand {
    // use gyro pidcontroller to drive straight until setpoint - 1
    // use encoder pidcontroller for remaining setpoint
    private DifferentialDrive drivebase;
    private double distance;
    private DrivebaseEncoder drivebaseEncoder;
    private boolean waitFinished;
    private double lastTime;
    private double waitDuration = .350; //seconds 
    private Gyro gyro;
    private PIDController gyroPID;

    public PIDDrive(DifferentialDrive drivebase, DrivebaseEncoder drivebaseEncoder, double distance) {
        super(3.0, 0, 5);
        this.drivebase = drivebase;
        this.distance = distance;
        this.gyro = Robot.getInstance().getGyro();
        this.gyroPID = new PIDController(0, 0, 0);
        this.drivebaseEncoder = drivebaseEncoder;
        setSetpoint(this.distance);
        getPIDController().setAbsoluteTolerance(0.1);
        getPIDController().setOutputRange(-1, 1);
        SmartDashboard.putData("PIDDrive Gyro PID", this.gyroPID);
    }

    public PIDDrive(double distance) {
        this(Robot.getInstance().getDifferentialDrive(),
             Robot.getInstance().getDrivebaseEncoder(),
             distance);
    }

    public PIDDrive() {
        this(0);
    }

    @Override
    protected void initialize() {
        this.gyro.reset();
        this.gyroPID.setPID(0.08, 0, 0.10);
        this.gyroPID.setSetpoint(0);
        this.gyroPID.enableContinuousInput(-180, 180);
        this.gyroPID.setTolerance(3);

        drivebaseEncoder.zeroEncoder();
        this.waitFinished = false;
        this.lastTime = 0;
    }

    @Override
    protected double returnPIDInput() {
        double pidInput = (drivebaseEncoder.getLeftSensorReading() + drivebaseEncoder.getRightSensorReading())/2;
        return pidInput;
    }

    @Override
    protected void usePIDOutput(double output) {
        System.out.println("zRotation" + gyroPID.calculate(gyro.getHeading()));
        // rivebase.arcadeDrive(-Robot.getInstance().getOI().getRightYAxis(), 
        // -gyroPID.calculate(gyro.getHeading()));
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
