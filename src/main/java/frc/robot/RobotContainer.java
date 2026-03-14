// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.UtilCommands.DriveCommand;
import frc.robot.commands.UtilCommands.OpCommands;
import frc.robot.commands.UtilCommands.WaitCommand;
import frc.robot.commands.Interpolator;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.TestCommand;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveModule;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  public final VisionSubsystem vision = new VisionSubsystem(drivebase);
  public final TransferSubsystem transfer=TransferSubsystem.getInstance();
  public final ShooterSubsystem shooter=ShooterSubsystem.getInstance();
  public final TestCommand test=new TestCommand(drivebase);
  public final FloorSubsystem floor = FloorSubsystem.getInstance();
  public final IntakeSubsystem intake = IntakeSubsystem.getInstance();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS5Controller driverGamepad = new CommandPS5Controller(0);
  final CommandPS5Controller coDriverGamepad = new CommandPS5Controller(1);
  final CommandGenericHID buttonPanel = new CommandGenericHID(2);
  /*
   * The button IDs on the button panel follow this layout:
   * 
   * Stick panel (LED pointing forward):
   * 1 2
   * 3 4
   * Left stick: axis 0 (horizontal), axis 1 (vertical)
   * Right stick: axis 4 (horizontal), axis 5 (vertical)
   * 
   * Button panel (3 LEDs pointing forward, 1 LED pointing right):
   * 13  9  5
   * 14 10  6
   * 15 11  7
   * 16 12  8
   */

  SendableChooser<Command> autoChooser = new SendableChooser<>();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Register commands for PathPlanner
    registerNamedCommands();

    // Input bindings
    //configureBindingsPanel(); // testing code
    configureBindingsShootOnTheMoveTest(); // testing code

    // Load interpolator files (if present)
    try {
      Interpolator.loadFiles(
        new File(Filesystem.getDeployDirectory(), "HoodData.txt"),
        new File(Filesystem.getDeployDirectory(), "ShootTimeData.txt")
      );
    } catch (IOException e) {
      e.printStackTrace();
      Interpolator.failed();
    }

    setAutoCommands();
    SmartDashboard.putData("Autos", autoChooser);
  }

  //testing mode
  private void configureBindingsPanel()
  {

    // DRIVER CONTROLS:

    //Joysticks (Default) - Drive the robot
    Command driveCommand = OpCommands.getDriveCommand(drivebase, driverGamepad);
    drivebase.setDefaultCommand(driveCommand);
    //driverGamepad.R2().and(driverGamepad.options())
    //      .onTrue(ShooterCommandss.getTurnAndDriveCommand(drivebase, driverGamepad, new Translation2d(4, 4)))
    //      .onFalse(driveCommand);//TODO: change to actual goal pose

    //Options - Zeros the robot heading
    driverGamepad.options().onTrue(Commands.runOnce(drivebase::zeroGyro));
    
    driverGamepad.R2().onTrue(shooter.shootCommand()).onFalse(shooter.idleCommand());
    driverGamepad.L1().onTrue(shooter.hoodCommand(0.7));
    driverGamepad.R1().onTrue(shooter.hoodCommand(0.77));

    driverGamepad.povDown().onTrue(transfer.stopCommand());
    driverGamepad.povUp().onTrue(transfer.runCommand());
    driverGamepad.povLeft().onTrue(intake.deployCommand());
    driverGamepad.povRight().onTrue(intake.stowCommand());

    driverGamepad.cross().onTrue(intake.zeroCommand());
    driverGamepad.triangle().onTrue(new InstantCommand(shooter::setIncreasePR));
    driverGamepad.square().onTrue(new InstantCommand(shooter::setDecreaseFR));
    driverGamepad.circle().onTrue(new InstantCommand(shooter::setIncreaseFR));
    
    //driverGamepad.square().onTrue(new InstantCommand(transfer::setRunFull)).onFalse(new InstantCommand(transfer::setStop));
    //driverGamepad.cross().onTrue(new ParallelCommandGroup(new InstantCommand(transfer::setRunFull),floor.intakeCommand())).onFalse(new ParallelCommandGroup(transfer.stopCommand(),floor.stopCommand()));
    driverGamepad.L2().onTrue(new ParallelCommandGroup(transfer.runCommand(),floor.intakeCommand(),intake.intakeCommand())).onFalse(new ParallelCommandGroup(transfer.stopCommand(),floor.stopCommand(),intake.stopCommand()));
    //driverGamepad.circle().onTrue(new InstantCommand(transfer::setRunFullB)).onFalse(new InstantCommand(transfer::setStop));
  }

  //comp bindings
  private void configureBindingsCOMPETITION()
  {

    // DRIVER CONTROLS:

    //Joysticks (Default) - Drive the robot
    Command driveCommand = OpCommands.getDriveCommand(drivebase, driverGamepad);
    drivebase.setDefaultCommand(driveCommand);

    //Options - Zeros the robot heading
    driverGamepad.options().onTrue(Commands.runOnce(drivebase::zeroGyro));
    
    
    driverGamepad.L2().onTrue(intake.defer(()->intake.smartIntakeCommand()));
    driverGamepad.L2().onFalse(intake.stopCommand());
    driverGamepad.cross().onTrue(intake.zeroCommand());

    //stow button
    driverGamepad.L1().onTrue(new SequentialCommandGroup(
      intake.stowCommand(),
      intake.intakeCommand()
    )).onFalse(intake.stopCommand());

    driverGamepad.R2().and(()->shooter.atSpeed()).onTrue(new ParallelCommandGroup(transfer.runCommand(),floor.intakeCommand()));//TODO: shoot code here next, should only shoot when in shoot mode, if in feed mode it should feed, and otherwise stay stowed
    driverGamepad.R2().and(()->!shooter.atSpeed()).onTrue(shooter.shootCommand());
    driverGamepad.R2().onFalse(new ParallelCommandGroup(transfer.stopCommand(),floor.stopCommand(),shooter.idleCommand()));

    //driverGamepad.R1().onTrue(shooter.adjustHoodCommand(0.005));
    //driverGamepad.povUp().onTrue(shooter.adjustHoodCommand(-0.005));
    
    driverGamepad.R1().and(()->shooter.isShootMode()).onTrue(shooter.hoodCommand(0.49));
    driverGamepad.R1().and(()->!shooter.isShootMode()).onTrue(shooter.hoodCommand(0.6));//TODO:toggle shoot mode, when not in shoot mode should be fully stowed to protect hood
    driverGamepad.povUp().and(()->shooter.isShootMode()).onTrue(shooter.hoodCommand(0.49));
    driverGamepad.povUp().and(()->!shooter.isShootMode()).onTrue(shooter.hoodCommand(0.77));//TODO: Toggle feed mode, shooter modes should be a state machine
    //modes should function as such:
    //shoot mode: always aim the hood so its ready when r2 is pressed
    //feed mode: same but for feed aiming
    //stow mode: keep hood down for protection
    //on r2 press: if in shoot or feed aim robot yaw for shots and spin up shooter, when both ready shoot

    //also add shoot on the move toggle to the button panel along with overrides we may find we need later


  }

  private void configureBindingsHoodTuning() {

    // DRIVER CONTROLS:

    //Joysticks (Default) - Drive the robot
    Command driveCommand = OpCommands.getDriveCommand(drivebase, driverGamepad);
    drivebase.setDefaultCommand(driveCommand);

    //Options - Zeros the robot heading
    driverGamepad.options().onTrue(Commands.runOnce(drivebase::zeroGyro));
    
    
    driverGamepad.L2().onTrue(intake.defer(()->intake.smartIntakeCommand()));
    driverGamepad.L2().onFalse(intake.stopCommand());
    driverGamepad.cross().onTrue(intake.zeroCommand());

    //stow button
    driverGamepad.L1().onTrue(new SequentialCommandGroup(
      intake.stowCommand(),
      intake.intakeCommand()
    )).onFalse(intake.stopCommand());

    driverGamepad.R2().and(()->shooter.atSpeed()).onTrue(new ParallelCommandGroup(transfer.runCommand(),floor.intakeCommand()));//TODO: shoot code here next, should only shoot when in shoot mode, if in feed mode it should feed, and otherwise stay stowed
    driverGamepad.R2().and(()->!shooter.atSpeed()).onTrue(shooter.shootCommand());
    driverGamepad.R2().onFalse(new ParallelCommandGroup(transfer.stopCommand(),floor.stopCommand(),shooter.idleCommand()));

    driverGamepad.R1().onTrue(shooter.adjustHoodCommand(0.005));
    driverGamepad.povUp().onTrue(shooter.adjustHoodCommand(-0.005));
  }

  private void configureBindingsPanelTest()
  {

    // DRIVER CONTROLS:

    //Joysticks (Default) - Drive the robot
    Command driveCommand = OpCommands.getDriveCommand(drivebase, driverGamepad);
    drivebase.setDefaultCommand(driveCommand);
    shooter.runPidCommand();
    //shooter.setDefaultCommand(shooter.runPidCommand());

    //Options - Zeros the robot heading
    driverGamepad.options().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverGamepad.L1().onTrue(shooter.hoodCommand(0.6));
    driverGamepad.R1().onTrue(shooter.hoodCommand(0.77));
    driverGamepad.R2().onTrue(shooter.shootCommand()).onFalse(shooter.idleCommand());

    driverGamepad.povDown().onTrue(new InstantCommand(shooter::setDecreasePL));
    driverGamepad.povUp().onTrue(new InstantCommand(shooter::setIncreasePL));
    driverGamepad.povLeft().onTrue(new InstantCommand(shooter::setDecreaseFL));
    driverGamepad.povRight().onTrue(new InstantCommand(shooter::setIncreaseFL));

    driverGamepad.cross().onTrue(new InstantCommand(shooter::setDecreasePR));
    driverGamepad.triangle().onTrue(new InstantCommand(shooter::setIncreasePR));
    driverGamepad.square().onTrue(new InstantCommand(shooter::setDecreaseFR));
    driverGamepad.circle().onTrue(new InstantCommand(shooter::setIncreaseFR));

    //driverGamepad.square().onTrue(new InstantCommand(transfer::setRunFull)).onFalse(new InstantCommand(transfer::setStop));
    //driverGamepad.cross().onTrue(new ParallelCommandGroup(new InstantCommand(transfer::setRunFull),floor.intakeCommand())).onFalse(new ParallelCommandGroup(transfer.stopCommand(),floor.stopCommand()));
    driverGamepad.L2().onTrue(new ParallelCommandGroup(new InstantCommand(transfer::setRun),floor.intakeCommand(),intake.intakeCommand())).onFalse(new ParallelCommandGroup(transfer.stopCommand(),floor.stopCommand(),intake.stopCommand()));
    //driverGamepad.circle().onTrue(new InstantCommand(transfer::setRunFullB)).onFalse(new InstantCommand(transfer::setStop));
  }

  public void configureBindingsShootOnTheMoveTest() {

    Command driveCommand = OpCommands.getDriveCommand(drivebase, driverGamepad);
    drivebase.setDefaultCommand(driveCommand);
    driverGamepad.options().onTrue(Commands.runOnce(drivebase::zeroGyro));

    Trigger autoTurning = driverGamepad.triangle().or(driverGamepad.cross());

    driverGamepad.L2().onTrue(intake.defer(()->intake.smartIntakeCommand()));
    driverGamepad.L2().onFalse(intake.stopCommand());
    driverGamepad.cross().onTrue(intake.zeroCommand());

    //stow button
    driverGamepad.L1().onTrue(new SequentialCommandGroup(
      intake.stowCommand(),
      intake.intakeCommand()
    )).onFalse(intake.stopCommand());

    driverGamepad.R1().onTrue(shooter.adjustHoodCommand(0.005));
    driverGamepad.povUp().onTrue(shooter.adjustHoodCommand(-0.005));

    driverGamepad.R2().whileTrue(new ParallelCommandGroup(new RepeatCommand(ShooterCommands.getTurnDriveShootWhileAtRestCommand(drivebase, driverGamepad, new Translation2d(4.6, 4)))))
          .onFalse(OpCommands.getDriveCommand(drivebase, driverGamepad).alongWith(shooter.idleCommand()));
          
    driverGamepad.triangle().onTrue(new ParallelCommandGroup(transfer.runCommand(),floor.intakeCommand()))
          .onFalse(new ParallelCommandGroup(transfer.stopCommand(),floor.stopCommand()));//TODO: change to actual goal pose
    
    //driverGamepad.cross().whileTrue(new RepeatCommand(ShooterCommands.getTurnDriveShootWhileMovingCommand(drivebase, driverGamepad, new Translation2d(4, 4))))
    //      .onFalse(driveCommand.alongWith(shooter.idleCommand()).alongWith(new ParallelCommandGroup(transfer.stopCommand(),floor.stopCommand(),intake.stopCommand())));//TODO: change to actual goal pose

  }






  public Trigger sticksInUseTrigger(CommandPS5Controller gamepad) {
    return new Trigger(() -> Math.abs(gamepad.getLeftX()) > Constants.OIConstants.kDriveDeadband
                          || Math.abs(gamepad.getLeftY()) > Constants.OIConstants.kDriveDeadband
                          || Math.abs(gamepad.getRightX()) > Constants.OIConstants.kDriveDeadband);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected(); 
  }

  private static double normalDegrees(double deg) {
    double mod = deg % 360.0;
    if (mod < 0) mod += 360;
    return mod;
  }
  
  public void setAutoCommands(){//TODO:Autos
   // autoChooser = AutoBuilder.buildAutoChooser();

    PIDController turnController = new PIDController(DriveConstants.kTurningP, DriveConstants.kTurningI, DriveConstants.kTurningD);
    turnController.setIZone(DriveConstants.kTurningIZone);
    turnController.enableContinuousInput(0, 360);
    turnController.setSetpoint(0);
  }

  public void registerNamedCommands() {//TODO:autos here
    
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  public void resetOdometryFromVision() {
    vision.resetOdometry();
  }

  public void cancelResetOdometryFromVision() {
    vision.cancelResetOdometry();
  }


  public void updateSmartDashboard() {
    

    SmartDashboard.putNumber("Pigeon Oritentation", drivebase.pigeon.getAccumGyroZ().getValueAsDouble() % 360.0);

    Pose2d fieldPos = drivebase.getPose();
    SmartDashboard.putNumber("Field X Position", fieldPos.getX());
    SmartDashboard.putNumber("Field Y Position", fieldPos.getY());
    SmartDashboard.putNumber("Field Heading", fieldPos.getRotation().getDegrees());

    int i=1;
    for (SwerveModule module : drivebase.swerveDrive.swerveDriveConfiguration.modules) {
      SmartDashboard.putNumber("module absolute "+i, module.getAbsoluteEncoder().getAbsolutePosition());
      SmartDashboard.putNumber("module angle "+i, module.getState().angle.getDegrees());
      SmartDashboard.putNumber("module offset "+i,Math.abs(module.getAbsoluteEncoder().getAbsolutePosition()-module.getState().angle.getDegrees()));
      i++;
    }

    SmartDashboard.putBoolean("Interpolator Failed Load?", Interpolator.hasFailed());
    SmartDashboard.putNumber("Distance to Goal", drivebase.getPose().getTranslation().getDistance(new Translation2d(4.6,4)));
    
    /* double goToStow = SmartDashboard.getNumber("Go to stow", 0);
    if (goToStow > 0.001) dashboardStowCommand.schedule();
    SmartDashboard.putNumber("Go to stow", 0);
    SmartDashboard.putBoolean("Will go stow?", dashboardStowCommand.isScheduled()); */

    SmartDashboard.updateValues();
  }

  //private Command dashboardStowCommand = opCommands.getStowParallelCommand();
}