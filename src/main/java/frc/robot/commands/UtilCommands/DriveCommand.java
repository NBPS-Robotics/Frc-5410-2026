package frc.robot.commands.UtilCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;


public class DriveCommand extends Command{

    private final SwerveSubsystem swerveDriveSubsystem;
    private final double time; 
    double startTime; 
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(1, 0,0); 
    

    public DriveCommand(SwerveSubsystem swerveDriveSubsystem, double time){
       this.swerveDriveSubsystem = swerveDriveSubsystem;
       this.time = time; 
        addRequirements(swerveDriveSubsystem);
    }

    @Override 
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
      swerveDriveSubsystem.drive(chassisSpeeds);
    }

    @Override 
    public boolean isFinished(){ 
        return Timer.getFPGATimestamp() - startTime > time;
    }

    @Override
    public void end(boolean interrupted){}
}