package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
            double distance = robotPose.getTranslation().getDistance(G);
            G = goalPose.minus(vel.times(timeToGoal(distance)));
        }
        return G;
    }

    //should return seconds to reach goal
    //used in getRelativeGoalPose
    public static double timeToGoal(double distance) {
        return Interpolator.interpolate(distance, Interpolator.DataType.SHOT_TIME);
    }

    public static double hoodAngle(double distance) {
        return Interpolator.interpolate(distance, Interpolator.DataType.HOOD);
    }



    public static Command getTurnAndDriveCommand(SwerveSubsystem drivebase, CommandPS5Controller gamepad, Translation2d goalPose) {

        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
            () -> -gamepad.getLeftY(),
            () -> -gamepad.getLeftX(),
            () -> Math.cos(getTurnAngle(drivebase, gamepad, goalPose)),
            () -> Math.sin(getTurnAngle(drivebase, gamepad, goalPose))
        );

        return driveFieldOrientedAnglularVelocity;

    }

    public static double getTurnAngle(SwerveSubsystem drivebase, CommandPS5Controller gamepad, Translation2d goalPose) {
        
        double dx = goalPose.getX() - drivebase.getPose().getX();
        double dy = goalPose.getY() - drivebase.getPose().getY();

        double desiredAngle = Math.atan2(dy, dx);
        
        return desiredAngle;
    }

    

    public static Command getTurnDriveShootWhileAtRestCommand(SwerveSubsystem drivebase, CommandPS5Controller gamepad, Translation2d goalPose) {

        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
            () -> -gamepad.getLeftY(),
            () -> -gamepad.getLeftX(),
            () -> Math.cos(getTurnAngle(drivebase, gamepad, goalPose)),
            () -> Math.sin(getTurnAngle(drivebase, gamepad, goalPose))
        );
        
        ShooterSubsystem shooter = ShooterSubsystem.getInstance();
        double distance = drivebase.getPose().getTranslation().getDistance(goalPose);
        Command shootCommand = shooter.shootCommand().andThen(shooter.hoodCommand(hoodAngle(distance)));

        return new ParallelCommandGroup(driveFieldOrientedAnglularVelocity, shootCommand);
    }



    public static Command getTurnDriveShootWhileMovingCommand(SwerveSubsystem drivebase, CommandPS5Controller gamepad, Translation2d goalPose) {

        Translation2d relativeGoalPose = getRelativeGoalPose(drivebase.getPose(), drivebase.getFieldVelocity(), goalPose);

        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
            () -> -gamepad.getLeftY(),
            () -> -gamepad.getLeftX(),
            () -> Math.cos(getTurnAngle(drivebase, gamepad, relativeGoalPose)),
            () -> Math.sin(getTurnAngle(drivebase, gamepad, relativeGoalPose))
        );
        
        ShooterSubsystem shooter = ShooterSubsystem.getInstance();
        double distance = drivebase.getPose().getTranslation().getDistance(relativeGoalPose);
        Command shootCommand = shooter.shootCommand().andThen(shooter.hoodCommand(hoodAngle(distance)));

        return new ParallelCommandGroup(driveFieldOrientedAnglularVelocity, shootCommand);
    }
}
