package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{

    private final SparkMax rTop = new SparkMax(ShooterConstants.RCanId, MotorType.kBrushless);
    private final SparkMax rBottom = new SparkMax(ShooterConstants.R2CanId, MotorType.kBrushless);
    private final SparkMax lTop = new SparkMax(ShooterConstants.LCanId, MotorType.kBrushless);
    private final SparkMax lBottom = new SparkMax(ShooterConstants.L2CanId, MotorType.kBrushless);

    private final Servo hoodServo = new Servo(ShooterConstants.ServoPWMID);

    private final PIDController shooterPidR=new PIDController(ShooterConstants.p, ShooterConstants.i, ShooterConstants.d);
    private final PIDController shooterPidL=new PIDController(ShooterConstants.p, ShooterConstants.i, ShooterConstants.d);

    private final SparkBaseConfig rightConfig;
    private final SparkBaseConfig right2Config;
    private final SparkBaseConfig leftConfig;
    private final SparkBaseConfig left2Config;

    private static ShooterSubsystem instance;

    public static ShooterSubsystem getInstance(){
        if (instance == null) {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

    public ShooterSubsystem(){
        SparkBaseConfig sharedConfig = new SparkMaxConfig().apply(Constants.kCoastConfig).smartCurrentLimit(40, 40);
        rightConfig=new SparkMaxConfig().apply(sharedConfig).inverted(false);
        right2Config=new SparkMaxConfig().apply(sharedConfig).follow(rTop);
        leftConfig=new SparkMaxConfig().apply(sharedConfig).inverted(false);
        left2Config=new SparkMaxConfig().apply(sharedConfig).follow(lTop);
        shooterPidR.setSetpoint(0);
        shooterPidL.setSetpoint(0);
        for(int i=0; i<=5; i++){
            rTop.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            rBottom.configure(right2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            lTop.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            lBottom.configure(left2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }

    }

    public void setIncreaseP(){
        shooterPidL.setP(shooterPidL.getP()+0.00005);
        shooterPidR.setP(shooterPidR.getP()+0.00005);
    }

    public void setDecreaseP(){
        shooterPidL.setP(shooterPidL.getP()-0.00005);
        shooterPidR.setP(shooterPidR.getP()-0.00005);
    }

    public void setIncreaseF(){
        ShooterConstants.f+=0.00005;
    }
    public void addI(){
        shooterPidL.reset();
        shooterPidR.reset();
        shooterPidL.setI(0.000001);
        shooterPidR.setI(0.000001);
    }

    public void setDecreaseF(){
        ShooterConstants.f-=0.00005;
    }
    
    public void setSpeed(double speed){
        shooterPidR.setSetpoint(speed);
        shooterPidL.setSetpoint(speed);
        //addI();
    }

    public void setStop(){
        shooterPidR.setSetpoint(0);
        shooterPidL.setSetpoint(0);
    }
    public void setIdle(){
        shooterPidR.setSetpoint(ShooterConstants.idleSpeed);
        shooterPidL.setSetpoint(ShooterConstants.idleSpeed);
        shooterPidL.setI(0.00);
        shooterPidR.setI(0.00);
    }

    public void setFeed(){
        shooterPidR.setSetpoint(-ShooterConstants.feedSpeed);
        shooterPidL.setSetpoint(-ShooterConstants.feedSpeed);
    }

    public void runPid(){
        double rTopSet = (shooterPidR.getSetpoint()*ShooterConstants.f)+shooterPidR.calculate(rTop.getEncoder().getVelocity());
        SmartDashboard.putNumber("rTop Power Set", rTopSet);
        rTop.set(rTopSet);
        double lTopSet = -((shooterPidL.getSetpoint()*ShooterConstants.f)+shooterPidL.calculate(lTop.getEncoder().getVelocity()));
        SmartDashboard.putNumber("lTop Power Set", lTopSet);
        lTop.set(lTopSet);
    }

    public Command runPidCommand(){
        return new InstantCommand(()->runPid(),this);
    }
    public void setHood(double position){
        hoodServo.set(position);
    }

    public void telemetry(){
      SmartDashboard.putNumber("p",shooterPidL.getP());
      SmartDashboard.putNumber("f",ShooterConstants.f);
      SmartDashboard.putNumber("speed",rTop.getEncoder().getVelocity());
      SmartDashboard.putNumber("POWER",rTop.getAppliedOutput());
      //SmartDashboard.putNumber("Pidput",shooterPidR.calculate(rTop.getEncoder().getVelocity()));
      runPid();
      SmartDashboard.updateValues();
    }

    @Override
    public void periodic(){
        telemetry();
    }

    //commands
    public Command shootCommand(){
        return this.runOnce(()->setSpeed(ShooterConstants.shootSpeed));
    }
    public Command stopCommand(){
        return this.runOnce(()->setStop());
    }
    public Command idleCommand(){
        return this.runOnce(()->setIdle());
    }
    public Command feedCommand(){
        return this.runOnce(()->setFeed());
    }
    public Command hoodCommand(double position){
        return this.runOnce(()->setHood(position));
    }

}
