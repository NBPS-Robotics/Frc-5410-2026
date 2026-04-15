package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TransferConstants;
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
    private final SparkBaseConfig rightConfig;

    public TransferSubsystem(){
        SparkBaseConfig sharedConfig = new SparkMaxConfig().apply(Constants.kCoastConfig).smartCurrentLimit(30, 30).inverted(true);
        rightConfig=new SparkMaxConfig().apply(sharedConfig).inverted(false);
        rightConfig.closedLoop.outputRange(-1, 1)
                                    .pid(TransferConstants.p, TransferConstants.i, TransferConstants.d)
                                    .feedForward.kV(TransferConstants.f);
        for(int i=0; i<=5; i++){
            rightTransfer.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
         rightTransfer.getClosedLoopController().setSetpoint(0,SparkBase.ControlType.kVelocity);
    }

 

    public void setRun(){
        rightTransfer.getClosedLoopController().setSetpoint(TransferConstants.motorSpeed,SparkBase.ControlType.kVelocity);
    }

    public void setRunFull(){
        rightTransfer.getClosedLoopController().setSetpoint(TransferConstants.motorSpeed,SparkBase.ControlType.kVelocity);
    }
    public void setRunFullB(){
        rightTransfer.getClosedLoopController().setSetpoint(-TransferConstants.motorSpeed,SparkBase.ControlType.kVelocity);
    }

    public void setStop(){
        rightTransfer.getClosedLoopController().setSetpoint(0,SparkBase.ControlType.kVelocity);
        rightTransfer.set(0);
    }

    public void setOuttake(){
        rightTransfer.getClosedLoopController().setSetpoint(-TransferConstants.motorSpeed,SparkBase.ControlType.kVelocity);
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
        return new SequentialCommandGroup(this.runOnce(()->setOuttake()));
    }
}
