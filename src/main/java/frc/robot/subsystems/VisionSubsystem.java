package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private boolean ignoreFlLime = false;
    private boolean ignoreBlLime = false;
    private boolean ignoreAllLimes = false;

    private LimelightHelpers.PoseEstimate limeF, limeB;

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
        //Look back at the github commit history if you want to see the old methods for updating each limelight separately
        updateAllLimes();
    }

     /**
     * Updates the odometry for the back limelight,using another method
     */
    private void updateAllLimes() {
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
            limeB = validatePoseEstimate(limeB);
            limeF = validatePoseEstimate(limeF);

            PoseEstimate bestPose = null;
            boolean canUseB = limeB != null && !ignoreBlLime;
            boolean canUseF = limeF != null && !ignoreFlLime;
            if (canUseB && canUseF) {
                bestPose = (limeF.avgTagArea >= limeB.avgTagArea) ? limeF : limeB;
            } else if (canUseF) {
                bestPose = limeF;
            } else if (canUseB) {
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
    public boolean poseInField(Pose2d pose) {
        return pose.getTranslation().getX() > 0 &&
            pose.getTranslation().getX() < 17.55 &&
            pose.getTranslation().getY() > 0 &&
            pose.getTranslation().getY() < 8.05;
    }

    public PoseEstimate validatePoseEstimate(PoseEstimate poseEstimate) {
        if (poseEstimate == null) return null;
            
            double tagMin = 1;
            double tagMax = 5;
            double minArea = 0.08;
            if (poseEstimate.tagCount == 1) minArea = 0.18;
            if (poseEstimate.tagCount > tagMax || poseEstimate.tagCount < tagMin) return null;
            if (poseEstimate.avgTagArea < minArea) return null;
            if (poseEstimate.avgTagDist > 8) return null;
            if(poseEstimate.rawFiducials[0].ambiguity > .65)return null;
            return poseEstimate;
        
}

}