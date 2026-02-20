package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class HangSubsystem extends SubsystemBase {

    private static SparkMax motor = new SparkMax(Constants.HangConstants.canID, MotorType.kBrushless);

    private static Servo servo = new Servo(Constants.HangConstants.ServoPWMID);

    private final SparkBaseConfig motorConfig;

    private static HangSubsystem instance;
    public static HangSubsystem getInstance() {
        if (instance == null) instance = new HangSubsystem();
        return instance;
    }

    public HangSubsystem() {
        motorConfig = new SparkMaxConfig().apply(Constants.kBrakeConfig).smartCurrentLimit(40, 40);;
        for(int i=0; i<=5; i++){
            motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
    }



}
