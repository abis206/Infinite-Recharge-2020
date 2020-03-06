package frc.team8051.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


///// Constants for different surface floor
///// constants from drivebase is commented out
final class Ceramics {
  final class FeedForwardConstants {
    public static final double Ks = 0; // 0.808;
    public static final double Kv = 0; // 0.952;
    public static final double Ka = 0; // 0.144;
  }
  final class PIDConstants {
    public static final double Kp = 0; // 6.43;
    public static final double Ki = 0; // 0.00;
    public static final double Kd = 0; // 0.00;
  }
}

final class Carpet {
  final class FeedForwardConstants {
    public static final double Ks = 0; // 0.924;
    public static final double Kv = 0; // 0.961; 
    public static final double Ka = 0; // 0.215;
  }
  final class PIDConstants {
    public static final double Kp = 0; // 9.38;
    public static final double Ki = 0; // 0.00;
    public static final double Kd = 0; // 0.00;
  }
}

public class Drivebase extends SubsystemBase {
    // maths :)
    private final double WheelDiameter = Units.feetToMeters(6.0/12.0);// wheel diameter is 6 inch
    private final double PulsePerRevolution = 20.0;
    private final double GearRatio = 10.75;
    private final double DistancePerPulse = (WheelDiameter*Math.PI)/PulsePerRevolution/GearRatio;

    private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(
        new WPI_VictorSPX(12),
        new WPI_VictorSPX(13)
    );

    private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(
        new WPI_VictorSPX(14),
        new WPI_VictorSPX(15)
    );

    private final DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
    private final Encoder encoderR = new Encoder(1, 2, false);
    private final Encoder encoderL = new Encoder(3, 4, false);

    private final ADIS16448_IMU imu = new ADIS16448_IMU();

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.685);

    private Pose2d pose = new Pose2d(0.0, 0.0, getHeading());

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), pose);
  
    private final SimpleMotorFeedforward feedforward = 
    new SimpleMotorFeedforward(
      Carpet.FeedForwardConstants.Ks,
      Carpet.FeedForwardConstants.Kv,
      Carpet.FeedForwardConstants.Ka
    );

    private final PIDController leftPIDController = new PIDController(Carpet.PIDConstants.Kp, 0, 0);
    private final PIDController rightPIDController = new PIDController(Carpet.PIDConstants.Kp, 0, 0);
    
    NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("position").getEntry("x");
    NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("position").getEntry("y");

    public Drivebase() {
        encoderL.setDistancePerPulse(DistancePerPulse);
        encoderR.setDistancePerPulse(DistancePerPulse);

        SmartDashboard.putData("Left PID Controller", leftPIDController);
        SmartDashboard.putData("Right PID Controller", rightPIDController);
    }

    public DifferentialDrive getDifferentialDrive() {
        return differentialDrive;
    }

    public void zeroHeading() {
        imu.reset();
    }

    public void zeroDistance() {
        encoderL.reset();
        encoderR.reset();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(-imu.getAngle(), 360));
    }

    public double getRawHeading() {
        return -imu.getAngle();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
          encoderL.getRate(), 
          encoderR.getRate()
        );
      }
    
      public DifferentialDriveKinematics getKinematics() {
        return kinematics;
      }
    
      public Pose2d getPose() {
        return pose;
      }
    
      public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
      }
    
      public PIDController getLeftPIDController() {
        return leftPIDController;
      }
    
      public PIDController getRightPIDController() {
        return rightPIDController;
      }
    
      public void setVolts(double leftVolts, double rightVolts) {
        leftGroup.setVoltage(leftVolts);
        rightGroup.setVoltage(-rightVolts);
        differentialDrive.feed();
      }
    
      public void reset() {
        zeroDistance();
        odometry.resetPosition(new Pose2d(), getHeading());
      }
    
      @Override
      public void periodic() {
        pose = odometry.update(
          getHeading(), 
          encoderL.getDistance(), 
          encoderR.getDistance()
        );
        
        m_xEntry.setDouble(pose.getTranslation().getX());
        m_yEntry.setDouble(pose.getTranslation().getY());
        // System.out.println(pose);
        // SmartDashboard.putNumber("Gyro_Heading", getRawHeading());
      }
    
}
