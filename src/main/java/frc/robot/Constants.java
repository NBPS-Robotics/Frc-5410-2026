package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity; 

public final class Constants {

  public static final SparkBaseConfig kBrakeConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);
  public static final SparkBaseConfig kCoastConfig = new SparkMaxConfig().idleMode(IdleMode.kCoast);
  public static final SparkBaseConfig kBrakeInvertedConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(true);

  static final double ratio = 1.0;

  

  public static final class DriveConstants {
    public static final double MaxErrorFromBot = 0.5;
    //controls speed for telop
    public static final double speedFull = 1.0;
    public static final double speedSlow = 0.5;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speedss
    public static  double kMaxSpeedMetersPerSecond = 5.6;
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.6); // Used in vision recognition
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(32.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));


    // Angular offsets of the modules relative to the chassis in radians
    /* public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2; */

    // Used for auto turning
    public static final double kTurningP = 0.02;
    public static final double kTurningI = 0.001;
    public static final double kTurningD = 0.005;
    public static final double kTurningIZone = 30.0;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1; // CAN OK ("swerve" canbus)
    public static final int kFrontLeftTurningCanId = 21; // CAN OK ("rio" canbus)
    public static final int kFrontLeftTurnEncoderCanId = 30; // CAN OK ("rio" canbus)

    public static final int kRearLeftDrivingCanId = 3; // CAN OK ("swerve" canbus)
    public static final int kRearLeftTurningCanId = 8; // CAN OK ("rio" canbus)
    public static final int kRearLeftTurnEncoderCanId = 33; // CAN OK ("rio" canbus)

    public static final int kFrontRightDrivingCanId = 2; // CAN OK ("swerve" canbus)
    public static final int kFrontRightTurningCanId = 20; // CAN OK ("rio" canbus)
    public static final int kFrontRightTurnEncoderCanId = 32; // CAN OK ("rio" canbus)

    public static final int kRearRightDrivingCanId = 4; // CAN OK ("swerve" canbus)
    public static final int kRearRightTurningCanId = 10; // CAN OK ("rio" canbus)
    public static final int kRearRightTurnEncoderCanId = 31; // CAN OK ("rio" canbus)

    public static final int kPigeonGyroCanId = 5; // CAN OK ("swerve" canbus)
    //public static final boolean kGyroReversed = false;
  }

  public static final class ShooterConstants {
    public static final int p = 0;
    public static final int i = 0;
    public static final int d = 0;
    public static final int f = 0;

    public static final int canIDL1 = 0;
    public static final int canIDL2 = 0;
    public static final int canIDR1 = 0;
    public static final int canIDR2 = 0;
  }

  public static final class TransferConstants{
    public static final int LCanId=0;
    public static final int RCanId=0;
    
    public static final double p=0;
    public static final double i=0;
    public static final double d=0;
    public static final double f=0;

    public static final int motorSpeed = 5000;
  }

  public static final class ShooterConstants{
    public static final int LCanId=0;
    public static final int RCanId=0;
    public static final int L2CanId=0;
    public static final int R2CanId=0;
    
    public static final double p=0;
    public static final double i=0;
    public static final double d=0;
    public static final double f=0;

    public static final int feedSpeed = 3000;
    public static final int idleSpeed = 1000;
  }

  public static final class FloorConstants {
    public static final int canID = 0;

<<<<<<< HEAD
    public static final double motorSpeed = 1;
  }

  public static final class IntakeConstants {
    public static final int canID = 0;
    public static final int canID2 = 0;
    public static final int pivotID = 0;

    public static final double p=0;
    public static final double i=0;
    public static final double d=0;

    public static final double tolerance=0;

    public static final double deploy=0;
    public static final double stow=0;

    public static final double iZone=0;

    public static final double motorSpeed = 1;
=======
    public static final double motorSpeed = 5000;
>>>>>>> 53e1d893a4015e8f05168be14278cab9e39231eb
  }

  public static final class OIConstants {
    /* public static final int kDriverGamepadPort = 0;
    public static final int kCoDriverGamepadPort = 1;
    public static final int kButtonPanelPort = 2; */

    // This deadband is too narrow for most joysticks, including stock PS5 controllers. For future years, you may need to make this bigger.
    public static final double kDriveDeadband = 0.05;
    public static final double kDriveLargeDeadband = 0.1; // This deadband is typical.
  }

  
  public static final double MAX_SPEED  = Units.feetToMeters(18.42);
}