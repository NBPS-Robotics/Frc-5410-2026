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

    public static double hoodAngle(double distance, boolean normalSpeed) {
        return normalSpeed ? 0.02607*distance + 0.46286 : 0.02607*distance + 0.46286;
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

    public static double getTurnSpeed(SwerveSubsystem drivebase, CommandPS5Controller gamepad, double angle) {
        


        double desiredAngle = angle;
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
        Command shootCommand = shooter.runOnce(()->{shooter.setSpeed(Constants.ShooterConstants.shootSpeed); shooter.setHood(hoodAngle(distance, true));});

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
            shooter.shootCommand().andThen(shooter.hoodCommand(hoodAngle(distance, true)));
        }, shooter);

        return new ParallelCommandGroup(driveFieldOrientedAnglularVelocity, shootCommand);
    }



    private static boolean autoTurnEnabled = true;
    public static void toggleAutoTurn() {
        autoTurnEnabled = !autoTurnEnabled;
    }

    // public Command aimHoodCommand(){
    //     ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    //     SwerveSubsystem drivebase = drivebase
    //     Translation2d goalPose = new Translation2d(4.6, 4);
    //     double distance = drivebase.getPose().getTranslation().getDistance(goalPose);
    //     return this.runOnce(()->shooter.setHood(hoodAngle(distance)));
    // }

    public static class TurnAndShootCommand extends Command {

        FloorSubsystem floor;
        TransferSubsystem transfer;
        ShooterSubsystem shooter;
        SwerveSubsystem drivebase=SwerveSubsystem.getInstance();
        CommandPS5Controller gamepad;
        Pose2d botPose=drivebase.getPose();

        boolean doLoading;
        Translation2d goalPose;
        
        public TurnAndShootCommand(SwerveSubsystem p_drivebase, CommandPS5Controller p_gamepad) {
            floor = FloorSubsystem.getInstance();
            transfer = TransferSubsystem.getInstance();
            shooter = ShooterSubsystem.getInstance();
            drivebase = p_drivebase;
            gamepad = p_gamepad;

            if (drivebase.isRedAlliance())
                    goalPose = new Translation2d(11.920, 4.04);
            else
                    goalPose = new Translation2d(4.6, 4.04);

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

            botPose = drivebase.getPose();

            double[] transV = SwerveSubsystem.deadband2d(-gamepad.getLeftY(), -gamepad.getLeftX(), Constants.OIConstants.kDriveDeadband);
            double angRot = autoTurnEnabled ? getTurnSpeed(drivebase, gamepad, goalPose) : MathUtil.applyDeadband(-gamepad.getRightX(), Constants.OIConstants.kDriveDeadband);
            drivebase.swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                                transV[0] * drivebase.swerveDrive.getMaximumChassisVelocity() * drivebase.driveMultiplier,
                                transV[1] * drivebase.swerveDrive.getMaximumChassisVelocity() * drivebase.driveMultiplier), 0.8),
                                angRot * drivebase.swerveDrive.getMaximumChassisAngularVelocity() * drivebase.driveMultiplier,
                                true,
                                true
            );

            double distance = botPose.getTranslation().getDistance(goalPose);
            if (distance < 2.4 && Constants.ShooterConstants.shootSpeed != Constants.ShooterConstants.shootSpeedConstLow) {
                Constants.ShooterConstants.shootSpeed = Constants.ShooterConstants.shootSpeedConstLow;
                Constants.ShooterConstants.fl = Constants.ShooterConstants.flLow;
                Constants.ShooterConstants.fr = Constants.ShooterConstants.frLow;
                shooter.setSpeed(Constants.ShooterConstants.shootSpeed);
            } else if (distance > 2.4 && Constants.ShooterConstants.shootSpeed != Constants.ShooterConstants.shootSpeedConst) {
                Constants.ShooterConstants.shootSpeed = Constants.ShooterConstants.shootSpeedConst;
                Constants.ShooterConstants.fl = Constants.ShooterConstants.flHigh;
                Constants.ShooterConstants.fr = Constants.ShooterConstants.frHigh;
                shooter.setSpeed(Constants.ShooterConstants.shootSpeed);
            }
            shooter.setHood(hoodAngle(distance, Constants.ShooterConstants.shootSpeed!=Constants.ShooterConstants.shootSpeedConstLow));

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
            shooter.setHood(0.49);
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    public static class FullFeedCommand extends Command {

        FloorSubsystem floor;
        TransferSubsystem transfer;
        ShooterSubsystem shooter;
        SwerveSubsystem drivebase=SwerveSubsystem.getInstance();
        CommandPS5Controller gamepad;
        Pose2d botPose=drivebase.getPose();

        boolean doLoading;
        Translation2d goalPose;
        
        public FullFeedCommand(SwerveSubsystem p_drivebase, CommandPS5Controller p_gamepad) {
            floor = FloorSubsystem.getInstance();
            transfer = TransferSubsystem.getInstance();
            shooter = ShooterSubsystem.getInstance();
            drivebase = p_drivebase;
            gamepad = p_gamepad;

            if (drivebase.isRedAlliance())
                    goalPose = new Translation2d(15, 4.04);
            else
                    goalPose = new Translation2d(1.5, 4.04);

            addRequirements(floor, transfer, shooter, drivebase);
            doLoading = false;
        }

        @Override
        public void initialize() {
            shooter.setSpeed(Constants.ShooterConstants.shootSpeed);
            floor.stopFloor();
            transfer.setStop();

            doLoading = false;

            Constants.ShooterConstants.fl = Constants.ShooterConstants.flHigh;
            Constants.ShooterConstants.fr = Constants.ShooterConstants.frHigh;
            shooter.setSpeed(Constants.ShooterConstants.feedSpeed);
            shooter.setHood(0.7);
        }

        @Override
        public void execute() {

            botPose = drivebase.getPose();

            double[] transV = SwerveSubsystem.deadband2d(-gamepad.getLeftY(), -gamepad.getLeftX(), Constants.OIConstants.kDriveDeadband);
            double angRot;
            if(drivebase.isRedAlliance())angRot = autoTurnEnabled ? getTurnSpeed(drivebase, gamepad, 0) : MathUtil.applyDeadband(-gamepad.getRightX(), Constants.OIConstants.kDriveDeadband);
            else angRot = autoTurnEnabled ? getTurnSpeed(drivebase, gamepad, Math.PI) : MathUtil.applyDeadband(-gamepad.getRightX(), Constants.OIConstants.kDriveDeadband);
            drivebase.swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                                transV[0] * drivebase.swerveDrive.getMaximumChassisVelocity() * drivebase.driveMultiplier,
                                transV[1] * drivebase.swerveDrive.getMaximumChassisVelocity() * drivebase.driveMultiplier), 0.8),
                                angRot * drivebase.swerveDrive.getMaximumChassisAngularVelocity() * drivebase.driveMultiplier,
                                true,
                                true
            );

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
            shooter.setHood(0.49);
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }






    public static class TurnAndShootInAutoCommand extends Command {

        FloorSubsystem floor;
        TransferSubsystem transfer;
        ShooterSubsystem shooter;
        SwerveSubsystem drivebase=SwerveSubsystem.getInstance();
        CommandPS5Controller gamepad;
        Pose2d botPose=drivebase.getPose();

        boolean doLoading;
        Translation2d goalPose;
        
        public TurnAndShootInAutoCommand(SwerveSubsystem p_drivebase) {
            floor = FloorSubsystem.getInstance();
            transfer = TransferSubsystem.getInstance();
            shooter = ShooterSubsystem.getInstance();
            drivebase = p_drivebase;
            
            if (drivebase.isRedAlliance())
                    goalPose = new Translation2d(11.920, 4.04);
            else
                    goalPose = new Translation2d(4.6, 4);

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
            botPose = drivebase.getPose();

            double[] transV = SwerveSubsystem.deadband2d(0, 0, Constants.OIConstants.kDriveDeadband);
            double angRot = getTurnSpeed(drivebase, null, goalPose);
            drivebase.swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                                transV[0] * drivebase.swerveDrive.getMaximumChassisVelocity() * drivebase.driveMultiplier,
                                transV[1] * drivebase.swerveDrive.getMaximumChassisVelocity() * drivebase.driveMultiplier), 0.8),
                                angRot * drivebase.swerveDrive.getMaximumChassisAngularVelocity() * drivebase.driveMultiplier,
                                true,
                                true
            );
            
            

            double distance = botPose.getTranslation().getDistance(goalPose);
            Constants.ShooterConstants.shootSpeed = Constants.ShooterConstants.shootSpeedConst;
            Constants.ShooterConstants.fl = Constants.ShooterConstants.flHigh;
            Constants.ShooterConstants.fr = Constants.ShooterConstants.frHigh;
            shooter.setSpeed(Constants.ShooterConstants.shootSpeed);
            shooter.setHood(hoodAngle(distance, Constants.ShooterConstants.shootSpeed!=Constants.ShooterConstants.shootSpeedConstLow));

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
            shooter.setHood(0.49);
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }





    public static class JustDoShootCommand extends Command {

        FloorSubsystem floor;
        TransferSubsystem transfer;
        ShooterSubsystem shooter;
        SwerveSubsystem drivebase=SwerveSubsystem.getInstance();
        Pose2d botPose=drivebase.getPose();

        boolean doLoading;
        boolean doneLoading;
        Translation2d goalPose;
        
        public JustDoShootCommand(SwerveSubsystem p_drivebase) {
            floor = FloorSubsystem.getInstance();
            transfer = TransferSubsystem.getInstance();
            shooter = ShooterSubsystem.getInstance();
            drivebase = p_drivebase;

            if (drivebase.isRedAlliance())
                    goalPose = new Translation2d(11.920, 4.04);
            else
                    goalPose = new Translation2d(4.6, 4.04);

            addRequirements(floor, transfer, shooter);
            doLoading = false;
            doneLoading = false;
        }

        @Override
        public void initialize() {
            shooter.setSpeed(Constants.ShooterConstants.shootSpeed);
            floor.stopFloor();
            transfer.setStop();

            doLoading = false;
            doneLoading = false;
        }

        @Override
        public void execute() {

            botPose = drivebase.getPose();

            //double angRot = getTurnSpeed(drivebase, null, goalPose);

            double distance = botPose.getTranslation().getDistance(goalPose);
            if (autoTurnEnabled && Constants.ShooterConstants.shootSpeed != Constants.ShooterConstants.shootSpeedConst) {
                Constants.ShooterConstants.shootSpeed = Constants.ShooterConstants.shootSpeedConst;
                Constants.ShooterConstants.fl = Constants.ShooterConstants.flHigh;
                Constants.ShooterConstants.fr = Constants.ShooterConstants.frHigh;
                shooter.setSpeed(Constants.ShooterConstants.shootSpeed);
            } else if (!autoTurnEnabled && Constants.ShooterConstants.shootSpeed != Constants.ShooterConstants.shootSpeedConstLow) {
                Constants.ShooterConstants.shootSpeed = Constants.ShooterConstants.shootSpeedConstLow;
                Constants.ShooterConstants.fl = Constants.ShooterConstants.flLow;
                Constants.ShooterConstants.fr = Constants.ShooterConstants.frLow;
                shooter.setSpeed(Constants.ShooterConstants.shootSpeed);
            }
            shooter.setHood(hoodAngle(distance, Constants.ShooterConstants.shootSpeed!=Constants.ShooterConstants.shootSpeedConstLow));

            if (!doLoading && shooter.atSpeed()) {
                doLoading = true;
            }
            if (!doneLoading && doLoading) {
                floor.doFloorIntake();
                transfer.setRun();
                doneLoading = true;
            }
        }

        @Override
        public void end(boolean interrupted) {
            floor.stopFloor();
            transfer.setStop();
            shooter.setIdle();
            shooter.setHood(0.49);
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

}
