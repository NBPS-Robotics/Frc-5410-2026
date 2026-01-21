package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FloorSubsystem extends SubsystemBase{

    private SparkMax motor = new SparkMax(Constants.FloorConstants.canID, MotorType.kBrushless);

    private static FloorSubsystem floorSingleton;
    public static FloorSubsystem getInstance() {
        if (floorSingleton==null) floorSingleton = new FloorSubsystem();
        return floorSingleton;
    }

    public FloorSubsystem() {
        SparkBaseConfig sharedConfig = new SparkMaxConfig().apply(Constants.kBrakeConfig).smartCurrentLimit(40, 40);
        SparkBaseConfig motorConfig = new SparkMaxConfig().apply(sharedConfig);

        motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }



    public void doFloorIntake() {
        motor.set(Constants.FloorConstants.motorSpeed);
    }

    public void doFloorOuttake() {
        motor.set(-Constants.FloorConstants.motorSpeed);
    }

    public void stopFloor() {
        motor.set(-0.0);
    }



    public Command intakeCommand(){
        return this.runOnce(()->doFloorIntake());
    }

    public Command outtakeCommand(){
        return this.runOnce(()->doFloorOuttake());
    }

    public Command stopCommand(){
        return this.runOnce(()->stopFloor());
    }
}
