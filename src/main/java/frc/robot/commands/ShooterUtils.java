package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShooterUtils {


    //velocity should be in units per second
    //getRelativeGoalPose will return the position of a hypothetical goal that the robot should aim for in order to hit the actual goal...
    //...accounting for the time it will take for the ball to reach the goal and the movement of the robot during that time
    public Translation2d getRelativeGoalPose(Translation2d robotPose, Translation2d velocity, Translation2d goalPose){
        Translation2d G = goalPose;
        for (int i = 0; i < 10; i++) {
            double distance = robotPose.getDistance(goalPose);
            G = goalPose.minus(velocity.times(timeToGoal(distance)));
        }
        return G;
    }

    //should return seconds to reach goal
    //used in getRelativeGoalPose
    public double timeToGoal(double distance) {
        return 0;
    }

}
