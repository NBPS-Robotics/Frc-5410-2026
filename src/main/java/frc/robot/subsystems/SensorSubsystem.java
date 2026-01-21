package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SensorSubsystem extends SubsystemBase{
    //btw i cant spell
    //note: does not contain the gyro, but this can be added if needed
    //this is a singleton subsystem, used to store all sensors and their outputs
    //please put all sensors here, with the possible exeption
    //of motor controllers, but it would be beneficial to have them here too
    
    //all sensors output values should be stored with either of 2 options:
    //1. public variable
    //2. private variable with getter metheod

    //all values from the sensors should be updated in the "updateAll" metheod
    
    private static SensorSubsystem instance;

    public static SensorSubsystem getInstance(){
        if (instance == null) {
            instance = new SensorSubsystem();
        }
        return instance;
    }


    public void init(){//initialize sensors and subsystems they might need for procsesing

    }

    public void updateAll(){//update all sensors and values
     
    }

}