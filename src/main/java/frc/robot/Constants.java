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
  public static final SparkBaseConfig kBrakeInvertedConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(true);

  static final double ratio = 1.0;

  public static final class OpConstantsForBall {
    public static final double Ball1Lift = 0.3 * ratio; //GROUND BALL
    public static final double Ball1Pivot = 0.05;

    public static final double Ball2Lift = 0.3 * ratio; //PROCESSOR BALL
    public static final double Ball2Pivot = 0.18;

    public static final double Ball3Lift = 1.75 * ratio; //BALL LEVEL 1
    public static final double Ball3Pivot = 0.18;

    public static final double Ball4Lift = 4.75 * ratio; //BALL LEVEL 2
    public static final double Ball4Pivot = 0.17;

    public static final double Ball5Lift = 10.45 * ratio; //Barge
    public static final double Ball5Pivot = 0.32; 
  }


  public static final class OpConstantsForPipe {
    public static final double Pipe1Lift = 0.65 * ratio; //TROUGH (LEVEL 1)
    public static final double Pipe1Pivot = 0.35;

    public static final double Pipe2Lift = 3.57 * ratio; //LEVEL 2
    public static final double Pipe2Pivot = 0.21;

    public static final double Pipe3Lift = 5.3 * ratio; //LEVEL 3
    public static final double Pipe3Pivot = 0.31424;

    public static final double Pipe4Lift = 10.45 * ratio; //LEVEL 4
    public static final double Pipe4Pivot = 0.24;
    public static final double Pipe4PivotOut = 0.4; //Applied while outtaking

    public static final double PipeRetLift = 2.8 * ratio;
    public static final double PipeRetPivot = 0.345;

    public static final double PipeIntakeLift = 2.8 * ratio; //REDUNDENT WITH PipeRetLift
    public static final double PipeIntakePivot = 0.345; //REDUNDENT WITH PipeRetPivot
  }

  public static final class IntakeConstants {
    // Parameters related to the Ball Intake and Pipe Intake

    
    // Pipe Intake:

    public static final double kPipeIntakeSpeed = -0.75;
    public static final double kPipeOuttakeSpeed = 0.4;
    public static final double kPipeOuttakeL1Speed = 0.3;
    public static final double kPipeOuttakeL4Speed = 0.5;

    public static final int kPipeMotorId = 4; // CAN OK
    public static final int kPipeSensorChannel = 1; // CAN OK


    // Ball Intake:

    public static final double kBallIntakeSpeed = 1;
    public static final double kBallOuttakeSpeed = -0.5;
    public static final double kBallOuttakeBargeSpeed = -1.0;

    public static final int kBallMotorId1 = 12; // CAN OK
    public static final int kBallMotorId2 = 13; // CAN OK
    public static final int kBallSensorChannel = 0; // CAN OK

  }

  public static final class IntakePositionConstants {
    public static final int kMaxLiftMotorTemp=75;
    public static final int kLiftMotor1Id = 5; // CAN OK (NO ENCODER, FOLLOWER MOTOR)
    public static final int kLiftMotor2Id = 11; // CAN OK (HAS ALTERNATE ENCODER, LEADING MOTOR)

    public static final double kLiftPosP = 0.50; // Used when the lift is going up (error is positive)
    public static final double kLiftNegP = 0.40; // Used when the lift is going down (error is negative)
    public static final double kLiftI = 0.005;//0.0015;
    public static final double kLiftPosD = 0;//0.0001;
    public static final double kLiftNegD = 0;//0.003;
    public static final double kLiftIZone = 0.5;
    public static final double kLiftLoopTolerance = 0;//0.05; // Tolerance for the closed loop
    public static final double kLiftTolerance = 0.15; // Tolerance for checking whether at setpoint (i.e. checking for when to move on to next command)
    public static final double kLiftAntigrav = 0;//0.1775; // Antigrav constant: an amount of power added to the PID output to counteract gravity

    public static final int kPivotMotor1Id = 27; // CAN OK (HAS ABSOLUTE ENCODER)

    public static final double kPivotP = 2.75;
    public static final double kPivotI = 0.005;
    public static final double kPivotD = 0.0;
    public static final double kPivotIZone = 0.03;
    public static final double kPivotLoopTolerance = 0;//0.01; // Tolerance for the closed loop
    public static final double kPivotTolerance = 0.01; // Tolerance for checking whether at setpoint (i.e. checking for when to move on to next command)

    public static final double stowPivot = 0.30;
    public static final double stowLift = 0.9 * ratio;

    public static final double bargePivotTravel = 0.30; // pivot moves to this first before lift rises
    public static final double bargePivot = 0.4;
    public static final double bargePivotShove = 0.3; // applied while depositing barge
    public static final double bargeLift = 10.45 * ratio;
  }

  public static final class HangConstants {
    // Parameters related to the Hang Subsystem

    public static final double kHangUnlockPos = 1.0; // UNTUNED
    public static final double kHangTwistPower = 1.0; // UNTUNED
    public static final double kHangGripPower = 1.0; // UNTUNED

    public static final double kHangLiftPos = 4.75 * ratio;
    public static final double kHangPivotPos = 0.30;

    public static final int kHangTwistMotorId = 6; // CAN NG
    public static final int kHangGripMotorId = 7; // CAN NG
    public static final int kServo1Channel = 0; // CAN OK
    public static final int kServo2Channel = 1; // CAN OK
  }

  public static final class TelePathingConstants {
    public static final PathConstraints kDefaultConstraints = new PathConstraints(2.0, 1.0, 2 * Math.PI, 4 * Math.PI);
    //public static final PathConstraints kDefaultConstraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
  }

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