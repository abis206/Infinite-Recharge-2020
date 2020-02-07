/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.team8051;

import frc.team8051.commands.drivebase.TestDrive;
import frc.team8051.commands.drivebase.RotateDrivebase;
import frc.team8051.sensors.DrivebaseEncoder;
import frc.team8051.sensors.Gyro;
import frc.team8051.subsystems.DifferentialDriveBase;
import frc.team8051.subsystems.Drivebase;
import frc.team8051.services.OI;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private static Robot robot;
  private Drivebase drivebase;
  private OI oi;
  private TestDrive testDrive;
  private DifferentialDriveBase differentialDriveBase;
  private DrivebaseEncoder drivebaseEncoder;
  private Gyro gyro;

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
    oi.initializeBind();
    testDrive = new TestDrive();

    SmartDashboard.putData("Rotate Drivebase Command", new RotateDrivebase());
    SmartDashboard.putData("Analog Gyro", gyro);
    SmartDashboard.putBoolean("Reset Gyro", false);
  }
  
  @Override
  public void robotPeriodic() {
    // for tuning the pid rotatedrivebase command
    if(SmartDashboard.getBoolean("Reset Gyro", false)) {
      gyro.reset();
      SmartDashboard.putBoolean("Reset Gyro", false);
    }
    SmartDashboard.putNumber("Current Heading", gyro.getAngle());
    // System.out.println("gryo angle: " + gyro.getAngle());
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
    // Scheduler.getInstance().add(testDrive);
  }

  @Override
  public void teleopPeriodic() {
//    System.out.println("encoder left " + drivebaseEncoder.getLeftSensorReading() +
//                    " encoder right " + drivebaseEncoder.getRightSensorReading());
    Scheduler.getInstance().run();
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
