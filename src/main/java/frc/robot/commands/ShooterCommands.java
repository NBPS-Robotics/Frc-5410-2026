package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class ShooterCommands {


    //velocity should be in units per second
    //getRelativeGoalPose will return the position of a hypothetical goal that the robot should aim for in order to hit the actual goal...
    //...accounting for the time it will take for the ball to reach the goal and the movement of the robot during that time
    public static Translation2d getRelativeGoalPose(Pose2d robotPose, ChassisSpeeds velocity, Translation2d goalPose){
        Translation2d vel = new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
        Translation2d G = goalPose;
        for (int i = 0; i < 10; i++) {
            double distance = robotPose.getTranslation().getDistance(goalPose);
            G = goalPose.minus(vel.times(timeToGoal(distance)));
        }
        return G;
    }

    //should return seconds to reach goal
    //used in getRelativeGoalPose
    public static double timeToGoal(double distance) {
        return 0;
    }

    public static double hoodAngle(double distance) {
        return 0;
    }



    public static Command getTurnAndDriveCommand(SwerveSubsystem drivebase, CommandPS5Controller gamepad, Translation2d goalPose) {

        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
            () -> -gamepad.getLeftY(),
            () -> -gamepad.getLeftX(),
            () -> getTurnSpeed(drivebase, gamepad, goalPose),
            OIConstants.kDriveDeadband, OIConstants.kDriveDeadband);

        return driveFieldOrientedAnglularVelocity;

    }

    public static double getTurnSpeed(SwerveSubsystem drivebase, CommandPS5Controller gamepad, Translation2d goalPose) {
        
        double dx = goalPose.getX() - drivebase.getPose().getX();
        double dy = goalPose.getY() - drivebase.getPose().getY();

        double desiredAngle = Math.atan2(dy, dx);
        double currentAngle = drivebase.getPose().getRotation().getRadians();
        double angleError = MathUtil.clamp(desiredAngle - currentAngle, -Math.PI, Math.PI);

        return angleError * 1.0;
    }

    

    public static Command getTurnDriveShootWhileAtRestCommand(SwerveSubsystem drivebase, CommandPS5Controller gamepad, Translation2d goalPose) {

        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
            () -> 0,
            () -> 0,
            () -> getTurnSpeed(drivebase, gamepad, goalPose),
            OIConstants.kDriveDeadband, OIConstants.kDriveDeadband);
        
        ShooterSubsystem shooter = ShooterSubsystem.getInstance();
        double distance = drivebase.getPose().getTranslation().getDistance(goalPose);
        Command shootCommand = shooter.shootCommand().alongWith(shooter.hoodCommand(hoodAngle(distance)));

        IntakeSubsystem intake = IntakeSubsystem.getInstance();
        FloorSubsystem floor = FloorSubsystem.getInstance();
        TransferSubsystem transfer = TransferSubsystem.getInstance();
        Command loadBallsCommand = intake.intakeCommand().alongWith(floor.intakeCommand()).alongWith(new InstantCommand(transfer::setRunFull));

        if (Math.abs(getTurnSpeed(drivebase, gamepad, goalPose)) < 0.5) {
            return driveFieldOrientedAnglularVelocity.alongWith(shootCommand).alongWith(loadBallsCommand);
        } else {
            return driveFieldOrientedAnglularVelocity.alongWith(shootCommand);
        }
    }



    public static Command getTurnDriveShootWhileMovingCommand(SwerveSubsystem drivebase, CommandPS5Controller gamepad, Translation2d goalPose) {

        Translation2d relativeGoalPose = getRelativeGoalPose(drivebase.getPose(), drivebase.getFieldVelocity(), goalPose);

        Command driveFieldOrientedAnglularVelocity = getTurnAndDriveCommand(drivebase, gamepad, relativeGoalPose);
        
        ShooterSubsystem shooter = ShooterSubsystem.getInstance();
        double distance = drivebase.getPose().getTranslation().getDistance(relativeGoalPose);
        Command shootCommand = shooter.shootCommand().alongWith(shooter.hoodCommand(hoodAngle(distance)));

        IntakeSubsystem intake = IntakeSubsystem.getInstance();
        FloorSubsystem floor = FloorSubsystem.getInstance();
        TransferSubsystem transfer = TransferSubsystem.getInstance();
        Command loadBallsCommand = intake.intakeCommand().alongWith(floor.intakeCommand()).alongWith(new InstantCommand(transfer::setRunFull));

        if (Math.abs(getTurnSpeed(drivebase, gamepad, relativeGoalPose)) < 0.5) {
            return driveFieldOrientedAnglularVelocity.alongWith(shootCommand).alongWith(loadBallsCommand);
        } else {
            return driveFieldOrientedAnglularVelocity.alongWith(shootCommand);
        }
    }
}
