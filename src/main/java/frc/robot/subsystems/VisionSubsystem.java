package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.UtilCommands.DriveCommand;
import frc.utils.LimelightHelpers;
import frc.utils.LimelightHelpers.PoseEstimate;

/**
 * Massive credit to team 4253 Raid Zero.
 * Code modified from their repository:
 * https://github.com/TASRobotics/RaidZero-FRC-2025/blob/main/src/main/java/raidzero/robot/subsystems/drivetrain/Limelight.java
 */
public class VisionSubsystem extends SubsystemBase{
    public enum LED_MODE {
        PIPELINE, OFF, BLINK, ON
    }

    public enum STREAM_MODE {
        STANDARD, PIP_MAIN, PIP_SECOND
    }

    /** If greater than current time, the next vision scan will also reset swerve drive odometry to the vision results. Probably don't use this in a match. */
    public double resetOdomAt = -10;

    private static final String fLimeName = "limelight-limef";
    private static final String bLimeName = "limelight";

    private static final String fPoseName = "FLpose";
    private static final String bPoseName = "BLpose";

    private boolean ignoreFlLime = false;
    private boolean ignoreBlLime = false;
    private boolean ignoreAllLimes = false;

    private StructPublisher<Pose2d> flNT = NetworkTableInstance.getDefault().getStructTopic("flNT", Pose2d.struct).publish();
    private StructPublisher<Pose2d> blNT = NetworkTableInstance.getDefault().getStructTopic("blNT", Pose2d.struct).publish();

    private LimelightHelpers.PoseEstimate limeF, limeB;
    private LimelightHelpers.PoseEstimate limeFPrev, limeBPrev;

    private Notifier notifier;

    private SwerveSubsystem swerve;

    /**
     * Constructs a {@link VisionSubsystem} instance
     */
    public VisionSubsystem(SwerveSubsystem swerve) {
        this.swerve = swerve;
        this.startThread();
    }

    /**
     * Sets the stream mode of the limelight
     *
     * @param limelightName The name of the limelight
     * @param mode {@link STREAM_MODE} of the limelight
     */
    public void setStreamMode(String limelightName, STREAM_MODE mode) {
        if (mode == STREAM_MODE.STANDARD) {
            LimelightHelpers.setStreamMode_Standard(limelightName);
        } else if (mode == STREAM_MODE.PIP_MAIN) {
            LimelightHelpers.setStreamMode_PiPMain(limelightName);
        } else if (mode == STREAM_MODE.PIP_SECOND) {
            LimelightHelpers.setStreamMode_PiPSecondary(limelightName);
        }
    }
    @Override
    public void periodic(){
        loop();
    }
    /**
     * Sets the pipeline of the limelight
     *
     * @param limelightName The name of the limelight
     * @param pipeline The pipeline index
     */
    public void setPipeline(String limelightName, int pipeline) {
        LimelightHelpers.setPipelineIndex(limelightName, pipeline);
    }

    /**
     * Sets the LED mode of the limelight
     *
     * @param limelightName The name of the limelight
     * @param mode The LED mode
     */
    public void setLedMode(String limelightName, LED_MODE mode) {
        if (mode == LED_MODE.PIPELINE) {
            LimelightHelpers.setLEDMode_PipelineControl(limelightName);
        } else if (mode == LED_MODE.OFF) {
            LimelightHelpers.setLEDMode_ForceOff(limelightName);
        } else if (mode == LED_MODE.BLINK) {
            LimelightHelpers.setLEDMode_ForceBlink(limelightName);
        } else if (mode == LED_MODE.ON) {
            LimelightHelpers.setLEDMode_ForceOn(limelightName);
        }
    }

    /**
     * Flag the next vision scan to directly reset the swerve pose estimator's pose to the vision results.
     * <p>This is useful for when localization gets messed up due to too much desync between odometry and vision results.</p>
     * <p>Times out if no vision scan occurs within 0.4 seconds of call.</p>
     */
    public void resetOdometry() {
        resetOdomAt = Timer.getFPGATimestamp() + 0.4;
    }

    /**
     * If currently holding out for a vision scan to reset odometry due to {@link #resetOdometry()}, don't.
     */
    public void cancelResetOdometry() {
        resetOdomAt = -10;
    }

    private boolean doResetOdom() {
        return resetOdomAt > Timer.getFPGATimestamp();
    }

    /**
     * Starts the Limelight odometry thread
     */
    private void startThread() {
        notifier = new Notifier(this::loop);
        notifier.startPeriodic(0.02);
    }

    /**
     * The main loop of the Limelight odometry thread
     */
    private void loop() {
        if (swerve.pigeon.getAngularVelocityZWorld().getValueAsDouble() > 360) {
            ignoreAllLimes = true;
        } else {
            ignoreAllLimes = false;
        }

        //updateFrontLime();
        //updateBackLime();
        updateAllLimes();
    }

    /**
     * Updates the odometry for the front limelight
     */
    private void updateFrontLime() {
        LimelightHelpers.SetRobotOrientation(
            fLimeName,
            swerve.getPose().getRotation().getDegrees(),
            swerve.pigeon.getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
        limeF = LimelightHelpers.getBotPoseEstimate_wpiBlue(fLimeName);

        if (limeF != null && limeF.pose != null) {
            SmartDashboard.putString("LimeF pose", limeF.pose.toString());
            if (doResetOdom()) {
                swerve.swerveDrive.resetOdometry(limeF.pose);
                resetOdomAt = -10;
            }

            ignoreFlLime = !poseInField(limeF.pose) ||
                (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue(fLimeName).getZ()) > 0.4) ||
                (LimelightHelpers.getTA(fLimeName) < 0.1) ||
                (limeFPrev != null && (limeF.pose.getTranslation().getDistance(limeFPrev.pose.getTranslation()) /
                    (limeF.timestampSeconds - limeFPrev.timestampSeconds)) > DriveConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeFPrev != null && (limeF.pose.getTranslation()
                    .getDistance(limeFPrev.pose.getTranslation()) > DriveConstants.MaxErrorFromBot)/*DriveConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.02)*/) ||
                (limeF.rawFiducials.length > 0 && limeF.rawFiducials[0].ambiguity > 0.5 &&
                    limeF.rawFiducials[0].distToCamera > 4.0) ||
                limeF.pose.equals(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

            if (!ignoreAllLimes && !ignoreFlLime) {
                SmartDashboard.putBoolean(fPoseName, true);
                flNT.set(limeF.pose);

                swerve.swerveDrive.addVisionMeasurement(
                    limeF.pose,
                    Utils.fpgaToCurrentTime(limeF.timestampSeconds),
                    VecBuilder.fill(0.5, 0.5, 5).div(LimelightHelpers.getTA(fLimeName))
                );
            } else {
                SmartDashboard.putBoolean(fPoseName, false);
            }

            limeFPrev = limeF;
        }
    }

    /**
     * Updates the odometry for the back limelight
     */
    private void updateBackLime() {
        LimelightHelpers.SetRobotOrientation(
            bLimeName,
            swerve.getPose().getRotation().getDegrees(),
            swerve.pigeon.getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
        limeB = LimelightHelpers.getBotPoseEstimate_wpiBlue(bLimeName);

        if (limeB != null && limeB.pose != null) {
            SmartDashboard.putString("LimeB pose", limeB.pose.toString());
            if (doResetOdom()) {
                swerve.swerveDrive.resetOdometry(limeB.pose);
                resetOdomAt = -10;
            }

            ignoreBlLime = !poseInField(limeB.pose) ||
                (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue(bLimeName).getZ()) > 0.4) ||
                (LimelightHelpers.getTA(bLimeName) < 0.1) ||
                (limeBPrev != null && (limeB.pose.getTranslation().getDistance(limeBPrev.pose.getTranslation()) /
                    (limeB.timestampSeconds - limeBPrev.timestampSeconds)) > DriveConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeBPrev != null && (limeB.pose.getTranslation()
                    .getDistance(limeBPrev.pose.getTranslation()) > DriveConstants.MaxErrorFromBot)/*DriveConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.02)*/) ||
                (limeB.rawFiducials.length > 0 && limeB.rawFiducials[0].ambiguity > 0.5 &&
                    limeB.rawFiducials[0].distToCamera > 4.0) ||
                limeB.pose.equals(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

            if (!ignoreAllLimes && !ignoreBlLime) {
                SmartDashboard.putBoolean(bPoseName, true);
                blNT.set(limeB.pose);

                swerve.swerveDrive.addVisionMeasurement(
                    limeB.pose,
                    Utils.fpgaToCurrentTime(limeB.timestampSeconds),
                    VecBuilder.fill(0.75, 0.75, 5).div(LimelightHelpers.getTA(bLimeName))
                );
            } else {
                SmartDashboard.putBoolean(bPoseName, false);
            }

            limeBPrev = limeB;
        }
    }
     /**
     * Updates the odometry for the back limelight,using another method
     */
    private void updateAllLimes() {//new test vision method
        double r=swerve.getPose().getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(
            bLimeName,
            r,
            swerve.pigeon.getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
        LimelightHelpers.SetRobotOrientation(
            fLimeName,
            r,
            swerve.pigeon.getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
       
        limeB = LimelightHelpers.getBotPoseEstimate_wpiBlue(bLimeName);
        limeF = LimelightHelpers.getBotPoseEstimate_wpiBlue(fLimeName);
            if (!ignoreAllLimes) {
                SmartDashboard.putBoolean(bPoseName, true);
                limeB = validatePoseEstimate(limeB);
                SmartDashboard.putBoolean(fPoseName, true);
                limeF = validatePoseEstimate(limeF);//makes sure pose is valid

                PoseEstimate bestPose;
            if (limeF!= null && limeB != null) {
                bestPose = (limeF.avgTagArea >= limeB.avgTagArea) ? limeF : limeB;
         } else if (limeF != null) {//tries to get the best pose
              bestPose = limeF;
            } else {
            bestPose = limeB;
            }
                
            if (bestPose != null) {
                swerve.swerveDrive.addVisionMeasurement(
                    bestPose.pose,
                    bestPose.timestampSeconds,
                    VecBuilder.fill(0.75, 0.75, 45));
            } 
        }
    
    }
    


    /**
     * Checks if a pose is inside the field dimensions
     *
     * @param pose The {@link Pose2d} to check
     * @return True if the pose is inside the field dimensions, false otherwise
     */
    private boolean poseInField(Pose2d pose) {
        return pose.getTranslation().getX() > 0 &&
            pose.getTranslation().getX() < 17.55 &&
            pose.getTranslation().getY() > 0 &&
            pose.getTranslation().getY() < 8.05;
    }

    public PoseEstimate validatePoseEstimate(PoseEstimate poseEstimate) {
        if (poseEstimate == null) return null;
            
            double tagMin = 1;
            double tagMax = 2;
            double minArea = 0.08;
            if (poseEstimate.tagCount == 1) minArea = 0.18;
            if (poseEstimate.tagCount > tagMax || poseEstimate.tagCount < tagMin) return null;
            if (poseEstimate.avgTagArea < minArea) return null;
            if (poseEstimate.avgTagDist > 3.25) return null;
            if(poseEstimate.rawFiducials[0].ambiguity > .65)return null;
            return poseEstimate;
        
}

}