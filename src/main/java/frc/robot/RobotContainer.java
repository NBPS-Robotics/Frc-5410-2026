// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.UtilCommands.OpCommands;
import frc.robot.commands.Interpolator;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.TestCommand;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.VisionSubsystem;

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
    driverGamepad.povLeft().whileTrue(intake.new IntakeShakeCommand());
    driverGamepad.povRight().whileTrue(new InstantCommand(drivebase::lock));

    driverGamepad.L1().onTrue(new SequentialCommandGroup( //L1 - Stow the intake
      intake.stowCommand(),
      intake.intakeCommand()
    )).onFalse(intake.stopCommand());
    driverGamepad.povDown().onTrue(new SequentialCommandGroup(intake.outtakeCommand(),transfer.outtakeCommand(),floor.outtakeCommand())).onFalse(new SequentialCommandGroup(intake.stopCommand(),transfer.stopCommand(),floor.stopCommand()));

   
    driverGamepad.R2().whileTrue(new ShooterCommands.TurnAndShootCommand(drivebase, driverGamepad));
    driverGamepad.R1().whileTrue(new ShooterCommands.FullFeedCommand(drivebase, driverGamepad));
    driverGamepad.triangle().onTrue(new InstantCommand(()->ShooterCommands.toggleAutoTurn()));

    
  }

  
  
  public void configureBindingsShooterTuning()
  {

    // DRIVER CONTROLS:

    //Joysticks (Default) - Drive the robot
    Command driveCommand = OpCommands.getDriveCommand(drivebase, driverGamepad);
    drivebase.setDefaultCommand(driveCommand);
    //shooter.setDefaultCommand(shooter.runPidCommand());

    //Options - Zeros the robot heading
    //driverGamepad.options().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverGamepad.options().onTrue(new InstantCommand(()->ShooterCommands.toggleAutoTurn()));
    driverGamepad.R2().whileTrue(new ShooterCommands.JustDoShootCommand(drivebase));

   

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


  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected(); 
  }

  public void setAutoCommands(){
    autoChooser.addOption("HumanPlayer", new PathPlannerAuto("BlueHumanPlayer"));
    autoChooser.addOption("HumanPlayerSimple", new PathPlannerAuto("BlueHumanPlayerSimple"));
    autoChooser.addOption("HumanPlayerCloseShot", new PathPlannerAuto("BlueHumanPlayerCloseShoot"));
    autoChooser.addOption("Trough", new PathPlannerAuto("BlueTrough"));
    autoChooser.addOption("TroughSimple", new PathPlannerAuto("BlueTroughSimple"));
    autoChooser.addOption("TroughCloseShot", new PathPlannerAuto("BlueTroughCloseShoot"));
    autoChooser.addOption("CenterAuto", new PathPlannerAuto("CenterAuto"));
    autoChooser.addOption("CenterAutoSimple", new PathPlannerAuto("CenterAutoSimple"));
    autoChooser.addOption("TroughFastFar GetTrough", new PathPlannerAuto("FastFarTrough_GetTrough"));
    autoChooser.addOption("TroughFastFar GetCenter", new PathPlannerAuto("FastFarTrough_GetCenter"));
    autoChooser.addOption("HPFastFar GetCenter", new PathPlannerAuto("FastFarHP_GetCenter"));
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("Idle Shooter", shooter.idleHighCommand());
    NamedCommands.registerCommand("Deploy Intake", intake.deployCommand());
    NamedCommands.registerCommand("Start Intake", intake.intakeCommand());
    NamedCommands.registerCommand("Stow Intake", intake.stowCommand());
    NamedCommands.registerCommand("Shake", intake.new IntakeShakeCommand());
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