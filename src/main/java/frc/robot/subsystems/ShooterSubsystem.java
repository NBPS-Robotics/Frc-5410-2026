package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ShooterCommands;

public class ShooterSubsystem extends SubsystemBase{

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
        leftConfig=new SparkMaxConfig().apply(sharedConfig).inverted(true);
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

    public void setIncreasePR(){
        ShooterConstants.pr+=0.00001;
		shooterPidR.setP(ShooterConstants.pr);
    }
    public void setDecreasePR(){
        ShooterConstants.pr-=0.00001;
		shooterPidR.setP(ShooterConstants.pr);
    }
    public void setIncreaseDR(){
        ShooterConstants.dr+=0.000001;
        shooterPidR.setD(ShooterConstants.dr);
    }
    public void setDecreaseDR(){
        ShooterConstants.dr-=0.000001;
        shooterPidR.setD(ShooterConstants.dr);
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
        return (Math.abs(ShooterConstants.shootSpeed-lTop.getEncoder().getVelocity())<120)&&(Math.abs(ShooterConstants.shootSpeed-rTop.getEncoder().getVelocity())<120);
    }

    public void addI(){
        shooterPidL.reset();
        shooterPidR.reset();
        shooterPidL.setI(0.000001);
        shooterPidR.setI(0.000001);
    }
    
    public void setSpeed(double speed){
        shooterPidR.setSetpoint(speed);
        shooterPidL.setSetpoint(speed);
        shooterPidR.setP(ShooterConstants.pr);
        shooterPidL.setP(ShooterConstants.pl);
        //addI();
    }

    public void setStop(){
        shooterPidR.setSetpoint(0);
        shooterPidL.setSetpoint(0);
    }
    public void setIdle(){
        shooterPidR.setSetpoint(ShooterConstants.idleSpeed);
        shooterPidL.setSetpoint(ShooterConstants.idleSpeed);
        shooterPidR.setP(0.00002);
        shooterPidL.setP(0.00002);
        shooterPidL.setI(0.00);
        shooterPidR.setI(0.00);
    }
    public void setIdleHigh(){
        shooterPidR.setSetpoint(ShooterConstants.idleSpeed*2.5);
        shooterPidL.setSetpoint(ShooterConstants.idleSpeed*2.5);
        shooterPidR.setP(0.00004);
        shooterPidL.setP(0.00004);
        shooterPidL.setI(0.00);
        shooterPidR.setI(0.00);
    }

    public void setFeed(){
        shooterPidR.setSetpoint(-ShooterConstants.feedSpeed);
        shooterPidL.setSetpoint(-ShooterConstants.feedSpeed);
    }

    public void runPid(){
        double rTopSet = (shooterPidR.getSetpoint()*ShooterConstants.fr)+shooterPidR.calculate(rTop.getEncoder().getVelocity());
        SmartDashboard.putNumber("rTop Power Set", rTopSet);
        rTop.set(rTopSet);
        double lTopSet = (shooterPidL.getSetpoint()*ShooterConstants.fl)+shooterPidL.calculate(lTop.getEncoder().getVelocity());
        SmartDashboard.putNumber("lTop Power Set", lTopSet);
        lTop.set(lTopSet);
        if(shooterPidL.getSetpoint()==ShooterConstants.shootSpeed&&lTop.getEncoder().getVelocity()-shooterPidL.getSetpoint()<-70){
            lTop.set(1);
            SmartDashboard.putBoolean("gofastL",true);
        } else SmartDashboard.putBoolean("gofastL",false);
        if(shooterPidR.getSetpoint()==ShooterConstants.shootSpeed&&rTop.getEncoder().getVelocity()-shooterPidR.getSetpoint()<-70){
            rTop.set(1);
            SmartDashboard.putBoolean("gofastR",true);
        }else SmartDashboard.putBoolean("gofastR",false);
    }

    public Command runPidCommand(){
        return new InstantCommand(()->runPid(),this);
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
    public void changeHood(double increment){
        setHood(hoodServoR.getPosition()+increment);
    }
    public boolean shootMode=false;

    public void telemetry(){
      SmartDashboard.putNumber("pr",shooterPidR.getP());
      SmartDashboard.putNumber("dr",shooterPidR.getD());
      SmartDashboard.putNumber("fr",ShooterConstants.fr);
      SmartDashboard.putNumber("frHigh", ShooterConstants.frHigh);
      SmartDashboard.putNumber("frLow", ShooterConstants.frLow);
      SmartDashboard.putNumber("shootSpeed", ShooterConstants.shootSpeed);
      SmartDashboard.putNumber("r speed",rTop.getEncoder().getVelocity());
      SmartDashboard.putNumber("l speed",lTop.getEncoder().getVelocity());
      SmartDashboard.putNumber("POWER",rTop.getAppliedOutput());
      SmartDashboard.putNumber("Servo Set", hoodServoL.get());
      SmartDashboard.putBoolean("at speed", shooterPidL.atSetpoint());
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
    public Command idleHighCommand() {
        return this.runOnce(()->setIdleHigh());
    }
    public Command feedCommand(){
        return this.runOnce(()->setFeed());
    }
    public Command hoodCommand(double val){
        return this.runOnce(()->setHood(val));
    }
    public Command adjustHoodCommand(double increment){
        return this.runOnce(()->changeHood(increment));
    }

}
