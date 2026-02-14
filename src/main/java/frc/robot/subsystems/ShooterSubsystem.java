package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem shooterSingleton;
    public static ShooterSubsystem getInstance() {
        if (shooterSingleton==null) shooterSingleton = new ShooterSubsystem();
        return shooterSingleton;
    }

    private SparkMax motorL1 = new SparkMax(Constants.ShooterConstants.canIDL1, MotorType.kBrushless);
    private SparkMax motorL2 = new SparkMax(Constants.ShooterConstants.canIDL2, MotorType.kBrushless);
    private SparkMax motorR1 = new SparkMax(Constants.ShooterConstants.canIDR1, MotorType.kBrushless);
    private SparkMax motorR2 = new SparkMax(Constants.ShooterConstants.canIDR2, MotorType.kBrushless);

    private final SparkBaseConfig rightConfig;
    private final SparkBaseConfig leftConfig;

    private final PIDController PIDLeft = new PIDController(Constants.ShooterConstants.p, Constants.ShooterConstants.i, Constants.ShooterConstants.d);
    private final PIDController PIDRight = new PIDController(Constants.ShooterConstants.p, Constants.ShooterConstants.i, Constants.ShooterConstants.d);


    public ShooterSubsystem() {

        SparkBaseConfig sharedConfig = new SparkMaxConfig().apply(Constants.kBrakeConfig).smartCurrentLimit(40, 40);
        rightConfig=new SparkMaxConfig().apply(sharedConfig).inverted(true);
        leftConfig=new SparkMaxConfig().apply(sharedConfig).follow(motorL1);
        PIDLeft.setSetpoint(0);
        PIDRight.setSetpoint(0);

        for(int i=0; i<=5; i++){
            motorL1.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            motorL2.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            motorR1.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            motorR2.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }

    }



    public void setSpeed(int speed) {
        PIDLeft.setSetpoint(speed);
        PIDRight.setSetpoint(speed);
    }

    public void doStop() {
        PIDLeft.setSetpoint(0);
        PIDRight.setSetpoint(0);
    }

    public void runPid(){
        motorL1.set((PIDLeft.getSetpoint()*Constants.ShooterConstants.f)+PIDLeft.calculate(motorL1.getEncoder().getVelocity()));
        motorR1.set((PIDRight.getSetpoint()*Constants.ShooterConstants.f)+PIDRight.calculate(motorR1.getEncoder().getVelocity()));
    }



    public Command setSpeedCommand(int speed){
        return this.runOnce(()->setSpeed(speed));
    }
    public Command stopCommand(){
        return this.runOnce(()->doStop());
    }

}
