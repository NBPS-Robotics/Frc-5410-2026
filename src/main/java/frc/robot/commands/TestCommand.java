package frc.robot.commands;

import java.io.File;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;


public class TestCommand extends SequentialCommandGroup{
  private final SwerveSubsystem drive;

  
  

  public TestCommand(SwerveSubsystem drive){
    this.drive=drive;
    
    addRequirements(drive);
    addCommands(group0());
  }

  public Command[] group0(){
    Command[] commands={
    
    };
    return commands;
  }
}