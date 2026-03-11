package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class ShooterUtils {


    //velocity should be in units per second
    //getRelativeGoalPose will return the position of a hypothetical goal that the robot should aim for in order to hit the actual goal...
    //...accounting for the time it will take for the ball to reach the goal and the movement of the robot during that time
    public static Translation2d getRelativeGoalPose(Translation2d robotPose, Translation2d velocity, Translation2d goalPose){
        Translation2d G = goalPose;
        for (int i = 0; i < 10; i++) {
            double distance = robotPose.getDistance(goalPose);
            G = goalPose.minus(velocity.times(timeToGoal(distance)));
        }
        return G;
    }

    //should return seconds to reach goal
    //used in getRelativeGoalPose
    public static double timeToGoal(double distance) {
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

}
