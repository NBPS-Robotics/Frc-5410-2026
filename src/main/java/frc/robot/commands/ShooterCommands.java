package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import swervelib.math.SwerveMath;
import frc.robot.Constants;

public class ShooterCommands {

    public static Translation2d getRelativeGoalPose(Pose2d robotPose, ChassisSpeeds velocity, Translation2d goalPose){
        Translation2d vel = new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
        Translation2d G = goalPose;
        for (int i = 0; i < 10; i++) {
            double distance = robotPose.getTranslation().getDistance(G);
            G = goalPose.minus(vel.times(timeToGoal(distance)));
        }
        return G;
    }

    public static double timeToGoal(double distance) {
        return Interpolator.interpolate(distance, Interpolator.DataType.SHOT_TIME);
    }

    public static double hoodAngle(double distance, boolean normalSpeed) {
        return normalSpeed ? Interpolator.interpolate(distance, Interpolator.DataType.HOOD) : Interpolator.interpolate(distance, Interpolator.DataType.HOOD_CLOSE);
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
        double angleError = MathUtil.angleModulus(desiredAngle - currentAngle);

        return angleError * 1.0;
    }

    public static double getTurnSpeed(SwerveSubsystem drivebase, CommandPS5Controller gamepad, double angle) {
        
        double desiredAngle = angle;
        double currentAngle = drivebase.getPose().getRotation().getRadians();
        double angleError = MathUtil.angleModulus(desiredAngle - currentAngle);

        return angleError * 1.0;
    }

    public enum ShootSpeed {
        LOW, HIGH, FEED
    }
    public static void changeSpeed(ShootSpeed speed, ShooterSubsystem shooter) {
        if (speed == ShootSpeed.LOW) {
            Constants.ShooterConstants.shootSpeed = Constants.ShooterConstants.shootSpeedConstLow;
            Constants.ShooterConstants.fl = Constants.ShooterConstants.flLow;
            Constants.ShooterConstants.fr = Constants.ShooterConstants.frLow;
            shooter.setSpeed(Constants.ShooterConstants.shootSpeed);
        } else if (speed == ShootSpeed.HIGH) {
            Constants.ShooterConstants.shootSpeed = Constants.ShooterConstants.shootSpeedConst;
            Constants.ShooterConstants.fl = Constants.ShooterConstants.flHigh;
            Constants.ShooterConstants.fr = Constants.ShooterConstants.frHigh;
            shooter.setSpeed(Constants.ShooterConstants.shootSpeed);
        } else if (speed == ShootSpeed.FEED) {
            Constants.ShooterConstants.shootSpeed = Constants.ShooterConstants.feedSpeed;
            Constants.ShooterConstants.fl = Constants.ShooterConstants.flHigh;
            Constants.ShooterConstants.fr = Constants.ShooterConstants.frHigh;
            shooter.setSpeed(Constants.ShooterConstants.shootSpeed);
        }
    }





    private static boolean autoTurnEnabled = true;
    public static void toggleAutoTurn() {
        autoTurnEnabled = !autoTurnEnabled;
    }

    public static class TurnAndShootCommand extends Command {
        FloorSubsystem floor;
        TransferSubsystem transfer;
        ShooterSubsystem shooter;
        SwerveSubsystem drivebase=SwerveSubsystem.getInstance();
        CommandPS5Controller gamepad;
        Pose2d botPose=drivebase.getPose();

        boolean doLoading;
        Translation2d goalPose = new Translation2d(11.920, 4.04);

        int updater = 0;
        double startTime;
        double beginTime;
        
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
            startTime = 0;
            beginTime = Timer.getFPGATimestamp();

            floor.stopFloor();
            transfer.setStop();

            changeSpeed(ShootSpeed.LOW, shooter);

            doLoading = false;
            updater = 0;
            if (drivebase.isRedAlliance()) goalPose = new Translation2d(11.920, 4.04);
            else goalPose = new Translation2d(4.6, 4.04);
        }

        @Override
        public void execute() {

            botPose = drivebase.getPose();

            updater++;
            if (updater%7==5) {
                SmartDashboard.putNumber("getTurnSpeed", getTurnSpeed(drivebase, gamepad, goalPose));
                SmartDashboard.putBoolean("autoTurnEnabled", autoTurnEnabled);
                SmartDashboard.putNumber("true_distToGoal", botPose.getTranslation().getDistance(goalPose));
                SmartDashboard.putNumber("goal.x", goalPose.getX());
                SmartDashboard.putNumber("goal.y", goalPose.getY());
            }

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
            if (distance < 2.975 && Constants.ShooterConstants.shootSpeed != Constants.ShooterConstants.shootSpeedConstLow) {
                changeSpeed(ShootSpeed.LOW, shooter);
            } else if (distance > 3.025 && Constants.ShooterConstants.shootSpeed != Constants.ShooterConstants.shootSpeedConst) {
                changeSpeed(ShootSpeed.HIGH, shooter);
            }
            shooter.setHood(hoodAngle(distance, Constants.ShooterConstants.shootSpeed!=Constants.ShooterConstants.shootSpeedConstLow));
            SmartDashboard.putNumber("hoodAngle return", hoodAngle(distance, Constants.ShooterConstants.shootSpeed!=Constants.ShooterConstants.shootSpeedConstLow));

            
            if (startTime == 0 && Math.abs(angRot) < 0.2 && shooter.atSpeed()) {
                startTime = Timer.getFPGATimestamp();
                floor.doFloorOuttake();
            }
            if (!doLoading && startTime != 0 && Timer.getFPGATimestamp()-startTime>0.25) doLoading = true;
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
        double startTime;
        
        public FullFeedCommand(SwerveSubsystem p_drivebase, CommandPS5Controller p_gamepad) {
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
            floor.stopFloor();
            transfer.setStop();

            doLoading = false;
            startTime = 0;

            changeSpeed(ShootSpeed.FEED, shooter);
            shooter.setHood(0.67);
        }

        @Override
        public void execute() {

            botPose = drivebase.getPose();

            double[] transV = SwerveSubsystem.deadband2d(-gamepad.getLeftY(), -gamepad.getLeftX(), Constants.OIConstants.kDriveDeadband);
            double angRot;
            if(drivebase.isRedAlliance())angRot = MathUtil.applyDeadband(-gamepad.getRightX(), Constants.OIConstants.kDriveDeadband);
            else angRot = autoTurnEnabled ? getTurnSpeed(drivebase, gamepad, Math.PI) : MathUtil.applyDeadband(-gamepad.getRightX(), Constants.OIConstants.kDriveDeadband);
            drivebase.swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                                transV[0] * drivebase.swerveDrive.getMaximumChassisVelocity() * drivebase.driveMultiplier,
                                transV[1] * drivebase.swerveDrive.getMaximumChassisVelocity() * drivebase.driveMultiplier), 0.8),
                                angRot * drivebase.swerveDrive.getMaximumChassisAngularVelocity() * drivebase.driveMultiplier,
                                true,
                                true
            );

            if (startTime == 0 && Math.abs(angRot) < 0.2 && shooter.atSpeed()) {
                startTime = Timer.getFPGATimestamp();
                floor.doFloorOuttake();
            }
            if (!doLoading && startTime != 0 && Timer.getFPGATimestamp()-startTime>0.25) doLoading = true;
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
        double startTime;
        Translation2d goalPose;
        
        public TurnAndShootInAutoCommand(SwerveSubsystem p_drivebase) {
            floor = FloorSubsystem.getInstance();
            transfer = TransferSubsystem.getInstance();
            shooter = ShooterSubsystem.getInstance();
            drivebase = p_drivebase;
            
            if (drivebase.isRedAlliance()) goalPose = new Translation2d(11.920, 4.04);
            else goalPose = new Translation2d(4.6, 4);

            addRequirements(floor, transfer, shooter, drivebase);
            doLoading = false;
        }

        @Override
        public void initialize() {
            if (drivebase.isRedAlliance()) goalPose = new Translation2d(11.920, 4.04);
            else goalPose = new Translation2d(4.6, 4);

            floor.stopFloor();
            transfer.setStop();

            changeSpeed(ShootSpeed.LOW, shooter);

            doLoading = false;
            startTime = 0;
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
            shooter.setHood(hoodAngle(distance, Constants.ShooterConstants.shootSpeed!=Constants.ShooterConstants.shootSpeedConstLow));

            if (startTime == 0 && Math.abs(angRot) < 0.2 && shooter.atSpeed()) {
                startTime = Timer.getFPGATimestamp();
                floor.doFloorOuttake();
            }
            if (!doLoading && startTime != 0 && Timer.getFPGATimestamp()-startTime>0.25) doLoading = true;
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
            floor.stopFloor();
            transfer.setStop();

            doLoading = false;
            doneLoading = false;
        }

        @Override
        public void execute() {

            botPose = drivebase.getPose();

            double distance = botPose.getTranslation().getDistance(goalPose);
            if (autoTurnEnabled && Constants.ShooterConstants.shootSpeed != Constants.ShooterConstants.shootSpeedConst) {
                changeSpeed(ShootSpeed.HIGH, shooter);
            } else if (!autoTurnEnabled && Constants.ShooterConstants.shootSpeed != Constants.ShooterConstants.shootSpeedConstLow) {
                changeSpeed(ShootSpeed.LOW, shooter);
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
