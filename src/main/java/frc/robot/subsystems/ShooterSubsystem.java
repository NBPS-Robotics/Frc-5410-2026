package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{

    private static ShooterSubsystem instance;
    public static ShooterSubsystem getInstance(){
        if (instance == null) {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

    private final SparkMax rTop = new SparkMax(ShooterConstants.RCanId, MotorType.kBrushless);
    private final SparkMax rBottom = new SparkMax(ShooterConstants.R2CanId, MotorType.kBrushless);
    private final SparkMax lTop = new SparkMax(ShooterConstants.LCanId, MotorType.kBrushless);
    private final SparkMax lBottom = new SparkMax(ShooterConstants.L2CanId, MotorType.kBrushless);

    private final Servo hoodServoR = new Servo(ShooterConstants.RServoPWMID);
    private final Servo hoodServoL = new Servo(ShooterConstants.LServoPWMID);

    public final PIDController shooterPidR=new PIDController(ShooterConstants.pr, ShooterConstants.ir, ShooterConstants.dr);
    public final PIDController shooterPidL=new PIDController(ShooterConstants.pl, ShooterConstants.il, ShooterConstants.dl);
    public final PIDController PIDToTune = shooterPidR;

    private final SparkBaseConfig rightConfig;
    private final SparkBaseConfig right2Config;
    private final SparkBaseConfig leftConfig;
    private final SparkBaseConfig left2Config;

    public ShooterSubsystem(){
        SparkBaseConfig sharedConfig = new SparkMaxConfig().apply(Constants.kCoastConfig).smartCurrentLimit(40, 40)
        .apply(new EncoderConfig().quadratureMeasurementPeriod(16).quadratureAverageDepth(2));



        rightConfig=new SparkMaxConfig().apply(sharedConfig).inverted(false);
        rightConfig.closedLoop.outputRange(-1, 1)
                                    .pid(ShooterConstants.pr, ShooterConstants.ir, ShooterConstants.dr)
                                    .feedForward.kV(ShooterConstants.fr);
                            
        right2Config=new SparkMaxConfig().apply(sharedConfig).follow(rTop);
        leftConfig=new SparkMaxConfig().apply(sharedConfig).inverted(true);
        leftConfig.closedLoop.outputRange(-1, 1)
                                    .pid(ShooterConstants.pl, ShooterConstants.il, ShooterConstants.dl)
                                    .feedForward.kV(ShooterConstants.fl);

        left2Config=new SparkMaxConfig().apply(sharedConfig).follow(lTop);
        shooterPidR.setSetpoint(0);
        shooterPidL.setSetpoint(0);
        for(int i=0; i<=5; i++){
            rTop.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            rBottom.configure(right2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            lTop.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            lBottom.configure(left2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
        setHood(0.49);
    }



    public void setIncreaseFRHigh(){
        ShooterConstants.frHigh+=0.000001;
        if (ShooterConstants.shootSpeed==ShooterConstants.shootSpeedConst) ShooterConstants.fr=ShooterConstants.frHigh;
    }
    public void setDecreaseFRHigh(){
        ShooterConstants.frHigh-=0.000001;
        if (ShooterConstants.shootSpeed==ShooterConstants.shootSpeedConst) ShooterConstants.fr=ShooterConstants.frHigh;
    }
    public void setIncreaseFRLow(){
        ShooterConstants.frLow+=0.000001;
        if (ShooterConstants.shootSpeed==ShooterConstants.shootSpeedConstLow) ShooterConstants.fr=ShooterConstants.frLow;
    }
    public void setDecreaseFRLow(){
        ShooterConstants.frLow-=0.000001;
        if (ShooterConstants.shootSpeed==ShooterConstants.shootSpeedConstLow) ShooterConstants.fr=ShooterConstants.frLow;
    }


    public boolean atSpeed(){
        return(Math.abs(ShooterConstants.shootSpeed-rTop.getEncoder().getVelocity())<120)&&(Math.abs(ShooterConstants.shootSpeed-lTop.getEncoder().getVelocity())<120);
    }

   
    
    public void setSpeed(double speed){
        lTop.getClosedLoopController().setSetpoint(speed,ControlType.kVelocity);
        rTop.getClosedLoopController().setSetpoint(speed,ControlType.kVelocity);
        //addI();
    }

    public void setStop(){
        rTop.getClosedLoopController().setSetpoint(0,ControlType.kVelocity);
        lTop.getClosedLoopController().setSetpoint(0,ControlType.kVelocity);
    }
    public void setIdle(){
        lTop.getClosedLoopController().setSetpoint(ShooterConstants.idleSpeed,ControlType.kVelocity);
        rTop.getClosedLoopController().setSetpoint(ShooterConstants.idleSpeed,ControlType.kVelocity);
    }
    public void setIdleHigh(){
        lTop.getClosedLoopController().setSetpoint(ShooterConstants.idleSpeed*2.5,ControlType.kVelocity);
        rTop.getClosedLoopController().setSetpoint(ShooterConstants.idleSpeed*2.5,ControlType.kVelocity);
    }

    public void setFeed(){
        lTop.getClosedLoopController().setSetpoint(ShooterConstants.feedSpeed,ControlType.kVelocity);
        rTop.getClosedLoopController().setSetpoint(ShooterConstants.feedSpeed,ControlType.kVelocity);
    }



    public void setHood(double val){
        if(val<=0.7){
        hoodServoR.set(val);
        hoodServoL.set(val+0.02);
        }else{
        hoodServoR.set(0.7);
        hoodServoL.set(0.72);
        }
    }



    @Override
    public void periodic(){
      /*
      SmartDashboard.putNumber("pr",shooterPidR.getP());
      SmartDashboard.putNumber("dr",shooterPidR.getD());
      SmartDashboard.putNumber("fr",ShooterConstants.fr);
      SmartDashboard.putNumber("frHigh", ShooterConstants.frHigh);
      SmartDashboard.putNumber("frLow", ShooterConstants.frLow);
      */
      SmartDashboard.putNumber("shootSpeed", ShooterConstants.shootSpeed);
      SmartDashboard.putNumber("r speed",rTop.getEncoder().getVelocity());
      SmartDashboard.putNumber("l speed",lTop.getEncoder().getVelocity());
      SmartDashboard.putNumber("POWER",rTop.getAppliedOutput());
      SmartDashboard.putNumber("Servo Set", hoodServoR.get());
      //SmartDashboard.putNumber("Pidput",shooterPidR.calculate(rTop.getEncoder().getVelocity()));

      SmartDashboard.updateValues();
    }



    public Command shootCommand(){
        return this.runOnce(()->setSpeed(ShooterConstants.shootSpeed));
    }
    public Command stopCommand(){
        return this.runOnce(()->setStop());
    }
    public Command idleCommand(){
        return this.runOnce(()->setIdle());
    }
    public Command idleHighCommand() {
        return this.runOnce(()->setIdleHigh());
    }
    public Command feedCommand(){
        return this.runOnce(()->setFeed());
    }
    public Command hoodCommand(double val){
        return this.runOnce(()->setHood(val));
    }
    public Command hoodAdjustCommand(double val){
        return this.runOnce(()->setHood(hoodServoR.get()+val));
    }
    
}
