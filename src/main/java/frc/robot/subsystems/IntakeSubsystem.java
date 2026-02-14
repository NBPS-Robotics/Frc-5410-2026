package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

    private SparkMax motor = new SparkMax(Constants.IntakeConstants.canID, MotorType.kBrushless);
    private SparkMax motor2 = new SparkMax(Constants.IntakeConstants.canID2, MotorType.kBrushless);
    private SparkMax pivot = new SparkMax(Constants.IntakeConstants.pivotID, MotorType.kBrushless);

    private static IntakeSubsystem intakeSingleton;
    public static IntakeSubsystem getInstance() {
        if (intakeSingleton==null) intakeSingleton = new IntakeSubsystem();
        return intakeSingleton;
    }

    @SuppressWarnings({"removal"})
    public IntakeSubsystem() {
        SparkBaseConfig sharedConfig = new SparkMaxConfig().apply(Constants.kBrakeConfig).smartCurrentLimit(40, 40);
        SparkBaseConfig motorConfig = new SparkMaxConfig().apply(sharedConfig);
        SparkBaseConfig motor2Config = new SparkMaxConfig().apply(sharedConfig).inverted(true);
        SparkBaseConfig pivotConfig = new SparkMaxConfig().apply(sharedConfig);
        pivotConfig.closedLoop.outputRange(-1, 1)
                                    .pid(IntakeConstants.p, IntakeConstants.i, IntakeConstants.d)
                                    .iZone(IntakeConstants.iZone)
                                    .maxMotion.allowedClosedLoopError(IntakeConstants.tolerance);

        for (int i = 0; i<=5; i++) {
            motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            motor2.configure(motor2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            pivot.configure(pivotConfig,ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
    }


    public void zero() {
        pivot.getEncoder().setPosition(0);
    }

    public void deploy() {
        pivot.getClosedLoopController().setSetpoint(IntakeConstants.deploy, ControlType.kMAXMotionPositionControl);
    }

     public void stow() {
        pivot.getClosedLoopController().setSetpoint(IntakeConstants.stow, ControlType.kMAXMotionPositionControl);
    }

    public void doIntake() {
        motor.set(Constants.FloorConstants.motorSpeed);
    }

    public void doOuttake() {
        motor.set(-Constants.FloorConstants.motorSpeed);
    }

    public void stopIntake() {
        motor.set(0.0);
    }



    public Command intakeCommand(){
        return this.runOnce(()->doIntake());
    }

    public Command outtakeCommand(){
        return this.runOnce(()->doOuttake());
    }

    public Command stopCommand(){
        return this.runOnce(()->stopIntake());
    }

    public Command deployCommand(){
        return this.runOnce(()->deploy());
    }
    public Command stowCommand(){
        return this.runOnce(()->stow());
    }
}
