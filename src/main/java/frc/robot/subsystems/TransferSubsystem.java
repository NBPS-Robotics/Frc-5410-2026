package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TransferSubsystem extends SubsystemBase{

    private static TransferSubsystem instance;
    public static TransferSubsystem getInstance(){
        if (instance == null) {
            instance = new TransferSubsystem();
        }
        return instance;
    }

    private final SparkMax rightTransfer = new SparkMax(Constants.TransferConstants.RCanId, MotorType.kBrushless);

    private final PIDController transferPid=new PIDController(Constants.TransferConstants.p, Constants.TransferConstants.i, Constants.TransferConstants.d);
    private final SparkBaseConfig rightConfig;

    public TransferSubsystem(){
        SparkBaseConfig sharedConfig = new SparkMaxConfig().apply(Constants.kCoastConfig).smartCurrentLimit(40, 40).inverted(true);
        rightConfig=new SparkMaxConfig().apply(sharedConfig).inverted(false);
        transferPid.setSetpoint(0);
        for(int i=0; i<=5; i++){
            rightTransfer.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
    }



    public void setRun(){
        transferPid.setSetpoint(Constants.TransferConstants.motorSpeed);
    }

    public void setRunFull(){
        rightTransfer.set(1);
    }
    public void setRunFullB(){
        rightTransfer.set(-1);
    }

    public void setStop(){
        transferPid.setSetpoint(0);
        rightTransfer.set(0);
    }

    public void setOuttake(){
        transferPid.setSetpoint(-Constants.TransferConstants.motorSpeed);
    }

    public void runPid(){
        rightTransfer.set((transferPid.getSetpoint()*Constants.TransferConstants.f)+transferPid.calculate(rightTransfer.getEncoder().getVelocity()));
        if(transferPid.getSetpoint()==0)rightTransfer.set(0);
    }



    @Override
    public void periodic(){
        SmartDashboard.putNumber("transfer speed",rightTransfer.getEncoder().getVelocity());
    }



    public Command runCommand(){
        return this.runOnce(()->setRun());
    }
    public Command stopCommand(){
        return this.runOnce(()->setStop());
    }
    public Command outtakeCommand(){
        return new SequentialCommandGroup(this.runOnce(()->setOuttake()), this.runOnce(()->transferPid.reset()));
    }
}
