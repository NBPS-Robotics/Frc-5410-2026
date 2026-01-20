package frc.robot.commands.UtilCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitCommand extends Command{

    double startTime;
    double time; 


    public WaitCommand(double time){
        this.time = time; 
        addRequirements();
    }

    @Override 
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
    }

    @Override 
    public void execute(){    
    }

    @Override 
    public boolean isFinished(){
       return Timer.getFPGATimestamp() - startTime > time;
    }
    
}