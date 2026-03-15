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
import swervelib.math.SwerveMath;
import frc.robot.Constants;

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
        return 0.02607*distance + 0.46286;
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
            () -> -gamepad.getLeftY(),
            () -> -gamepad.getLeftX(),
            () -> getTurnSpeed(drivebase, gamepad, goalPose),
            OIConstants.kDriveDeadband, OIConstants.kDriveDeadband);
        
        ShooterSubsystem shooter = ShooterSubsystem.getInstance();
        double distance = drivebase.getPose().getTranslation().getDistance(goalPose);
        Command shootCommand = shooter.runOnce(()->{shooter.setSpeed(Constants.ShooterConstants.shootSpeed); shooter.setHood(hoodAngle(distance));});

        return driveFieldOrientedAnglularVelocity.alongWith(shootCommand);
    
    }



    public static Command getTurnDriveShootWhileMovingCommand(SwerveSubsystem drivebase, CommandPS5Controller gamepad, Translation2d goalPose) {

        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
            () -> -gamepad.getLeftY(),
            () -> -gamepad.getLeftX(),
            () -> Math.cos(getTurnAngle(drivebase, gamepad, getRelativeGoalPose(drivebase.getPose(), drivebase.getFieldVelocity(), goalPose))),
            () -> Math.sin(getTurnAngle(drivebase, gamepad, getRelativeGoalPose(drivebase.getPose(), drivebase.getFieldVelocity(), goalPose)))
        );
        
        ShooterSubsystem shooter = ShooterSubsystem.getInstance();
        Command shootCommand = new RunCommand(()-> {
            double distance = drivebase.getPose().getTranslation().getDistance(getRelativeGoalPose(drivebase.getPose(), drivebase.getFieldVelocity(), goalPose));
            shooter.shootCommand().andThen(shooter.hoodCommand(hoodAngle(distance)));
        }, shooter);

        return new ParallelCommandGroup(driveFieldOrientedAnglularVelocity, shootCommand);
    }



    private static boolean autoTurnEnabled = true;
    public static void toggleAutoTurn() {
        autoTurnEnabled = !autoTurnEnabled;
    }

    public static class TurnAndShootCommand extends Command {

        FloorSubsystem floor;
        TransferSubsystem transfer;
        ShooterSubsystem shooter;
        SwerveSubsystem drivebase;
        CommandPS5Controller gamepad;

        boolean doLoading;
        Translation2d goalPose = new Translation2d(4.6, 4);

        public TurnAndShootCommand(SwerveSubsystem p_drivebase, CommandPS5Controller p_gamepad) {
            floor = FloorSubsystem.getInstance();
            transfer = TransferSubsystem.getInstance();
            shooter = ShooterSubsystem.getInstance();
            drivebase = p_drivebase;
            gamepad = p_gamepad;
            addRequirements(floor, transfer, shooter, drivebase);
            doLoading = false;
        }

        @Override
        public void initialize() {
            shooter.setSpeed(Constants.ShooterConstants.shootSpeed);
            floor.stopFloor();
            transfer.setStop();

            doLoading = false;
        }

        @Override
        public void execute() {
            double[] transV = SwerveSubsystem.deadband2d(-gamepad.getLeftY(), -gamepad.getLeftX(), Constants.OIConstants.kDriveDeadband);
            double angRot = autoTurnEnabled ? getTurnSpeed(drivebase, gamepad, goalPose) : MathUtil.applyDeadband(-gamepad.getRightX(), Constants.OIConstants.kDriveDeadband);
            drivebase.swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                                transV[0] * drivebase.swerveDrive.getMaximumChassisVelocity() * drivebase.driveMultiplier,
                                transV[1] * drivebase.swerveDrive.getMaximumChassisVelocity() * drivebase.driveMultiplier), 0.8),
                                angRot * drivebase.swerveDrive.getMaximumChassisAngularVelocity() * drivebase.driveMultiplier,
                                true,
                                true
            );
            
            double distance = drivebase.getPose().getTranslation().getDistance(goalPose);
            shooter.setHood(hoodAngle(distance));
            
            if (Math.abs(angRot) < 0.2 && shooter.atSpeed()) {
                doLoading = true;
            }
            if (doLoading) {
                floor.doFloorIntake();
                transfer.setRun();
            }
        }

        @Override
        public void end(boolean interrupted) {
            floor.stopFloor();
            transfer.setStop();
            shooter.setIdle();
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }
}
