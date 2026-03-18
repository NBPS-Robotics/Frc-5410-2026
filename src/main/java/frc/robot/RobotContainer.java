// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.util.Set;
import java.util.function.Supplier;
import java.util.jar.Attributes.Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  SendableChooser<Command> AutoChooser = new SendableChooser<>();
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = SwerveSubsystem.getInstance();
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
    configureBindingsCOMPETITION(); // testing code

    // Load interpolator files (if present)
    try {
      Interpolator.loadFiles(
        new File(Filesystem.getDeployDirectory(), "HoodData.txt"),
        new File(Filesystem.getDeployDirectory(), "HoodDataClose.txt"),
        new File(Filesystem.getDeployDirectory(), "ShootTimeData.txt")
      );
    } catch (IOException e) {
      e.printStackTrace();
      Interpolator.failed();
    }

    setAutoCommands();
    SmartDashboard.putData("Autos", autoChooser);
  }



  //comp bindings
  private void configureBindingsCOMPETITION()
  {
    Command driveCommand = OpCommands.getDriveCommand(drivebase, driverGamepad); //Joysticks - Drive the robot
    drivebase.setDefaultCommand(driveCommand);
    driverGamepad.options().onTrue(Commands.runOnce(drivebase::zeroGyro)); //Options - Zeros the robot heading
    
    driverGamepad.L2().onTrue(intake.defer(()->intake.smartIntakeCommand())); //L2 - Deploy intake and start intaking
    driverGamepad.L2().onFalse(intake.stopCommand());
    driverGamepad.cross().onTrue(intake.zeroCommand());

    driverGamepad.L1().onTrue(new SequentialCommandGroup( //L1 - Stow the intake
      intake.stowCommand(),
      intake.intakeCommand()
    )).onFalse(intake.stopCommand());
    driverGamepad.povDown().onTrue(new SequentialCommandGroup(intake.outtakeCommand(),transfer.outtakeCommand(),floor.outtakeCommand())).onFalse(new SequentialCommandGroup(intake.stopCommand(),transfer.stopCommand(),floor.stopCommand()));

    // driverGamepad.R2().and(()->shooter.atSpeed()).onTrue(new ParallelCommandGroup(transfer.runCommand(),floor.intakeCommand()));//TODO: shoot code here next, should only shoot when in shoot mode, if in feed mode it should feed, and otherwise stay stowed
    // driverGamepad.R2().and(()->!shooter.atSpeed()).onTrue(shooter.shootCommand());
    // driverGamepad.R2().onFalse(new ParallelCommandGroup(transfer.stopCommand(),floor.stopCommand(),shooter.idleCommand()));
    driverGamepad.R2().whileTrue(new ParallelCommandGroup(new ShooterCommands.TurnAndShootCommand(drivebase, driverGamepad),new InstantCommand(()->shooter.shootMode=true)));
    driverGamepad.R1().whileTrue(new ParallelCommandGroup(new ShooterCommands.FullFeedCommand(drivebase, driverGamepad),new InstantCommand(()->shooter.shootMode=true)));


    // driverGamepad.povUp().and(()->shooter.shootMode()).onTrue(shooter.hoodCommand(0.49));
    // driverGamepad.povUp().and(()->!shooter.shootMode()).onTrue(shooter.hoodCommand(0.77));//TODO: Toggle feed mode, shooter modes should be a state machine
    //modes should function as such:
    //shoot mode: always aim the hood so its ready when r2 is pressed
    //feed mode: same but for feed aiming
    //stow mode: keep hood down for protection
    //on r2 press: if in shoot or feed aim robot yaw for shots and spin up shooter, when both ready shoot

    //also add shoot on the move toggle to the button panel along with overrides we may find we need later
  }

  
  
  private void configureBindingsShooterTuning()
  {

    // DRIVER CONTROLS:

    //Joysticks (Default) - Drive the robot
    Command driveCommand = OpCommands.getDriveCommand(drivebase, driverGamepad);
    drivebase.setDefaultCommand(driveCommand);
    shooter.runPidCommand();
    //shooter.setDefaultCommand(shooter.runPidCommand());

    //Options - Zeros the robot heading
    //driverGamepad.options().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverGamepad.options().onTrue(new InstantCommand(()->ShooterCommands.toggleAutoTurn()));
    driverGamepad.R2().whileTrue(new ShooterCommands.JustDoShootCommand(drivebase));

    driverGamepad.povUp().onTrue(shooter.runOnce(()->shooter.setIncreasePR()));
    driverGamepad.povDown().onTrue(shooter.runOnce(()->shooter.setDecreasePR()));
    driverGamepad.povRight().onTrue(shooter.runOnce(()->shooter.setIncreaseDR()));
    driverGamepad.povLeft().onTrue(shooter.runOnce(()->shooter.setDecreaseDR()));

    driverGamepad.triangle().onTrue(shooter.runOnce(()->shooter.setIncreaseFRHigh()));
    driverGamepad.cross().onTrue(shooter.runOnce(()->shooter.setDecreaseFRHigh()));
    driverGamepad.circle().onTrue(shooter.runOnce(()->shooter.setIncreaseFRLow()));
    driverGamepad.square().onTrue(shooter.runOnce(()->shooter.setDecreaseFRLow()));

    //driverGamepad.square().onTrue(new InstantCommand(transfer::setRunFull)).onFalse(new InstantCommand(transfer::setStop));
    //driverGamepad.cross().onTrue(new ParallelCommandGroup(new InstantCommand(transfer::setRunFull),floor.intakeCommand())).onFalse(new ParallelCommandGroup(transfer.stopCommand(),floor.stopCommand()));
    driverGamepad.L2().onTrue(new ParallelCommandGroup(new InstantCommand(transfer::setRun),floor.intakeCommand(),intake.intakeCommand())).onFalse(new ParallelCommandGroup(transfer.stopCommand(),floor.stopCommand(),intake.stopCommand()));
    //driverGamepad.circle().onTrue(new InstantCommand(transfer::setRunFullB)).onFalse(new InstantCommand(transfer::setStop));
  }



  public void configureBindingsShootOnTheMoveTest() {

    Command driveCommand = OpCommands.getDriveCommand(drivebase, driverGamepad);
    drivebase.setDefaultCommand(driveCommand);
    driverGamepad.options().onTrue(Commands.runOnce(drivebase::zeroGyro));

    driverGamepad.L2().onTrue(intake.defer(()->intake.smartIntakeCommand()));
    driverGamepad.L2().onFalse(intake.stopCommand());
    driverGamepad.cross().onTrue(intake.zeroCommand());

    driverGamepad.L1().onTrue(new SequentialCommandGroup(
      intake.stowCommand(),
      intake.intakeCommand()
    )).onFalse(intake.stopCommand());

    driverGamepad.R2().whileTrue(new ShooterCommands.TurnAndShootCommand(drivebase, driverGamepad));
    driverGamepad.triangle().onTrue(new InstantCommand(()->ShooterCommands.toggleAutoTurn()));
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
  

  private static double normalDegrees(double deg) {
    double mod = deg % 360.0;
    if (mod < 0) mod += 360;
    return mod;
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected(); 
  }

  public void setAutoCommands(){
    autoChooser.addOption("BlueHumanPlayer", new PathPlannerAuto("BlueHumanPlayer"));
    autoChooser.addOption("BlueHumanPlayerSimple", new PathPlannerAuto("BlueHumanPlayerSimple"));
    autoChooser.addOption("BlueTrough", new PathPlannerAuto("BlueTrough"));
    autoChooser.addOption("BlueTroughSimple", new PathPlannerAuto("BlueTroughSimple"));
    autoChooser.addOption("CenterAuto", new PathPlannerAuto("CenterAuto"));
    autoChooser.addOption("CenterAutoSimple", new PathPlannerAuto("CenterAutoSimple"));
  }

  public void registerNamedCommands() {//TODO:autos here
    NamedCommands.registerCommand("Idle Shooter", shooter.idleHighCommand());
    NamedCommands.registerCommand("Deploy Intake", intake.deployCommand());
    NamedCommands.registerCommand("Start Intake", intake.intakeCommand());
    NamedCommands.registerCommand("Stow Intake", intake.stowCommand());
    NamedCommands.registerCommand("Turn And Shoot", new ShooterCommands.TurnAndShootInAutoCommand(drivebase));
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
    


    Pose2d fieldPos = drivebase.getPose();
    SmartDashboard.putNumber("Field X Position", fieldPos.getX());
    SmartDashboard.putNumber("Field Y Position", fieldPos.getY());
    SmartDashboard.putNumber("Field Heading", fieldPos.getRotation().getDegrees());

    /*int i=1;
    for (SwerveModule module : drivebase.swerveDrive.swerveDriveConfiguration.modules) {
      SmartDashboard.putNumber("module absolute "+i, module.getAbsoluteEncoder().getAbsolutePosition());
      SmartDashboard.putNumber("module angle "+i, module.getState().angle.getDegrees());
      SmartDashboard.putNumber("module offset "+i,Math.abs(module.getAbsoluteEncoder().getAbsolutePosition()-module.getState().angle.getDegrees()));
      i++;
    }*/

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