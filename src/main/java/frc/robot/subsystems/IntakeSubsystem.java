package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeSubsystem extends SubsystemBase{

    private SparkMax motor = new SparkMax(Constants.IntakeConstants.canID, MotorType.kBrushless);
    private SparkMax motor2 = new SparkMax(Constants.IntakeConstants.canID2, MotorType.kBrushless);
    public SparkMax pivot = new SparkMax(Constants.IntakeConstants.pivotID, MotorType.kBrushless);
    SparkBaseConfig sharedConfig = new SparkMaxConfig().apply(Constants.kBrakeConfig).smartCurrentLimit(20, 20);
    SparkBaseConfig motorConfig = new SparkMaxConfig().apply(sharedConfig);
    SparkBaseConfig motor2Config = new SparkMaxConfig().apply(sharedConfig);
    SparkBaseConfig pivotConfig = new SparkMaxConfig().apply(sharedConfig).inverted(true);
    private static IntakeSubsystem intakeSingleton;
    public static IntakeSubsystem getInstance() {
        if (intakeSingleton==null) intakeSingleton = new IntakeSubsystem();
        return intakeSingleton;
    }

    @SuppressWarnings({"removal"})
    public IntakeSubsystem() {
       
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


    public void zero() {//
        pivot.getEncoder().setPosition(0);
    }

    public void setBrake() {//
        pivotConfig.idleMode(IdleMode.kBrake);
        pivot.configure(pivotConfig,ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setCoast() {//
        pivotConfig.idleMode(IdleMode.kCoast);
        pivot.configure(pivotConfig,ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void deploy() {
        pivot.getClosedLoopController().setSetpoint(IntakeConstants.deploy, ControlType.kPosition);
    }

     public void stow() {
        pivot.getClosedLoopController().setSetpoint(IntakeConstants.stow, ControlType.kPosition);
    }

    public boolean getAtPosistion(){
        return Math.abs(pivot.getClosedLoopController().getSetpoint()-pivot.getEncoder().getPosition())<=1;
    }

    public Trigger atStow(){
        return new Trigger(()->getAtPosistion()&&pivot.getClosedLoopController().getSetpoint()==IntakeConstants.stow);
    }
    public Trigger atDeploy(){
        return new Trigger(()->getAtPosistion()&&pivot.getClosedLoopController().getSetpoint()==IntakeConstants.deploy);
    }

    public void doIntake() {
        motor.set(-1);
        motor2.set(-1);
    }

    public void doOuttake() {
        motor.set(1);
        motor2.set(1);
    }

    public void stopIntake() {
        motor.set(0.0);
        motor2.set(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Pos", pivot.getEncoder().getPosition());
        
    }



    public Command intakeCommand(){//
        return this.runOnce(()->doIntake());
    }

    public Command zeroCommand(){//
        return this.runOnce(()->pivot.getEncoder().setPosition(0));
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
