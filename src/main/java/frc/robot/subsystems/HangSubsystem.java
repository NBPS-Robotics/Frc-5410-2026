package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HangConstants;;

public class HangSubsystem extends SubsystemBase {

    private static SparkMax motor = new SparkMax(HangConstants.canID, MotorType.kBrushless);

    private static Servo servo = new Servo(HangConstants.ServoPWMID);

    private static HangSubsystem instance;
    public static HangSubsystem getInstance() {
        if (instance == null) instance = new HangSubsystem();
        return instance;
    }

    @SuppressWarnings({"removal"})
    public HangSubsystem() {
        SparkBaseConfig sharedConfig = new SparkMaxConfig().apply(Constants.kBrakeConfig).smartCurrentLimit(40, 40);
        SparkBaseConfig motorConfig = new SparkMaxConfig().apply(sharedConfig);
        motorConfig.closedLoop.outputRange(-1, 1)
                                    .pid(HangConstants.p, HangConstants.i, HangConstants.d)
                                    .iZone(HangConstants.iZone)
                                    .maxMotion.allowedClosedLoopError(HangConstants.tolerance);

        for(int i=0; i<=5; i++){
            motor.configure(motorConfig,ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
    }


    public void zero() {
        motor.getEncoder().setPosition(0);
    }
    public void setHang(double position) {
        motor.getClosedLoopController().setSetpoint(position, ControlType.kMAXMotionPositionControl);
    }
    public void setServo(double position) {
        servo.set(position);
    }


    public Command hangCommand(double position) {
        return this.runOnce(()->setHang(position));
    }
    public Command servoCommand(double position) {
        return this.runOnce(()->setServo(position));
    }


}
