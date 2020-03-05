/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.team8051;

import frc.team8051.sensors.DrivebaseEncoder;
import frc.team8051.sensors.Gyro;
import frc.team8051.subsystems.DifferentialDriveBase;
import frc.team8051.subsystems.Drivebase;
import frc.team8051.services.OI;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private static Robot robot;
  private Drivebase drivebase;
  private OI oi;
  private DifferentialDriveBase differentialDriveBase;
  private DrivebaseEncoder drivebaseEncoder;
  private Gyro gyro;
  private boolean JoystickUser = false;

  Robot() {
    robot = this;
    oi = new OI();
    differentialDriveBase = new DifferentialDriveBase();
    drivebaseEncoder = new DrivebaseEncoder();

    gyro = new Gyro();
    gyro.calibrate();
  }

  @Override
  public void robotInit() {
<<<<<<< HEAD
=======
    oi.initializeBind();
    testDrive = new TestDrive();
    colorSensor.initializeColor();
    colorSensor.setToZero();
>>>>>>> 51ed52066497c06d5104713e0e9e046ee73a536a

    // SmartDashboard.putData("Rotate Drivebase Command", new RotateDrivebase());
    SmartDashboard.putData("Analog Gyro", gyro);
    SmartDashboard.putBoolean("Reset Gyro", false);
    SmartDashboard.putBoolean("Reset Encoders", false);

    // SmartDashboard.putData("Drivebase Subsystem", differentialDriveBase);
    // SmartDashboard.putData("PIDDrive Command", new PIDDrive());
  }
  
  @Override
  public void robotPeriodic() {

    // using below lines for tuning the pid constants
    if(SmartDashboard.getBoolean("Reset Gyro", false)) {
      gyro.reset();
      SmartDashboard.putBoolean("Reset Gyro", false);
    }
    if(SmartDashboard.getBoolean("Reset Encoders", false)) {
      drivebaseEncoder.zeroEncoder();
      SmartDashboard.putBoolean("Reset Encoders", false);
    }

    SmartDashboard.putNumber("Current Heading", gyro.getHeading());
<<<<<<< HEAD
    Scheduler.getInstance().run();
=======
    colorSensor.colorMatcherr();
>>>>>>> 51ed52066497c06d5104713e0e9e046ee73a536a
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    System.out.println("Running teleopInit()");
  }

  @Override
  public void teleopPeriodic() {
    if(oi.getOpertorJoystick().getRawButtonPressed(OI.JoystickMap.BUTTON_A)) {
      JoystickUser = !JoystickUser;
    }

    double r = JoystickUser? oi.getRightYAxis(oi.getOpertorJoystick()) : oi.getRightYAxis(oi.getDriverJoystick());
    double l = JoystickUser? oi.getLeftYAxis(oi.getOpertorJoystick()) : oi.getLeftYAxis(oi.getDriverJoystick());
    differentialDriveBase.getDifferentialDrive().tankDrive(-r * 0.5, -l * 0.5);
 
  }

  @Override
  public void testPeriodic() {

  }

  public static Robot getInstance() {
    if(robot == null)
      robot = new Robot();
    return robot;
  }

  public OI getOI() {
    return oi;
  }

  public Drivebase getDrivebase() {
    return drivebase;
  }

  public DifferentialDrive getDifferentialDrive() {
    return differentialDriveBase.getDifferentialDrive();
  }

  public DrivebaseEncoder getDrivebaseEncoder() {
    return drivebaseEncoder;
  }

  public Gyro getGyro() {
    return gyro;
  }
}
