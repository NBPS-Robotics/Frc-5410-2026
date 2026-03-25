package frc.robot.commands.UtilCommands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class OpCommands {
    public final TransferSubsystem transfer=TransferSubsystem.getInstance();
    public final ShooterSubsystem shooter=ShooterSubsystem.getInstance();
    public final FloorSubsystem floor = FloorSubsystem.getInstance();
    public final IntakeSubsystem intake = IntakeSubsystem.getInstance();

    // STATIC

    public static Command getDriveCommand(SwerveSubsystem drivebase, CommandPS5Controller gamepad) {
        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the desired angle NOT angular rotation
        /* Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
            () -> MathUtil.applyDeadband(driverGamepad.getLeftY(), Constants.OIConstants.kDriveDeadband),
            () -> MathUtil.applyDeadband(driverGamepad.getLeftX(), Constants.OIConstants.kDriveDeadband),
            () -> driverGamepad.getRightX(),
            () -> driverGamepad.getRightY()); */

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the angular velocity of the robot
        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
            () -> -gamepad.getLeftY()*(drivebase.isRedAlliance() ? -1 : 1),
            () -> -gamepad.getLeftX()*(drivebase.isRedAlliance() ? -1 : 1),
            () -> -gamepad.getRightX(),
            OIConstants.kDriveDeadband, OIConstants.kDriveDeadband);

        return driveFieldOrientedAnglularVelocity;


    }
}