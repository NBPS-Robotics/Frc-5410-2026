package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TransferSubsystem extends SubsystemBase{

    private final SparkMax leftTransfer = new SparkMax(Constants.TransferConstants.LCanId, MotorType.kBrushless);
    private final SparkMax rightTransfer = new SparkMax(Constants.TransferConstants.RCanId, MotorType.kBrushless);

    private final PIDController transferPid=new PIDController(Constants.TransferConstants.p, Constants.TransferConstants.i, Constants.TransferConstants.d);

    private final SparkBaseConfig rightConfig;
    private final SparkBaseConfig leftConfig;

    private static TransferSubsystem instance;

    public static TransferSubsystem getInstance(){
        if (instance == null) {
            instance = new TransferSubsystem();
        }
        return instance;
    }

    public TransferSubsystem(){
        SparkBaseConfig sharedConfig = new SparkMaxConfig().apply(Constants.kBrakeConfig).smartCurrentLimit(40, 40);
        rightConfig=new SparkMaxConfig().apply(sharedConfig).inverted(true);
        leftConfig=new SparkMaxConfig().apply(sharedConfig).follow(leftTransfer);
        transferPid.setSetpoint(0);
        for(int i=0; i<=5; i++){
            rightTransfer.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            leftTransfer.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }

    }

    public void setRun(){
        transferPid.setSetpoint(5000);
    }

    public void setStop(){
        transferPid.setSetpoint(0);
    }

    public void setOuttake(){
        transferPid.setSetpoint(-5000);
    }

    public void runPid(){
        rightTransfer.set((transferPid.getSetpoint()*Constants.TransferConstants.f)+transferPid.calculate(rightTransfer.getEncoder().getVelocity()));
    }

    //commands
    public Command runCommand(){
        return this.runOnce(()->setRun());
    }
    public Command stopCommand(){
        return this.runOnce(()->setStop());
    }
    public Command outtakeCommand(){
        return this.runOnce(()->setOuttake());
    }

}
