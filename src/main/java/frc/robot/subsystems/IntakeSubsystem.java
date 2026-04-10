package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.UtilCommands.WaitCommand;
import swervelib.math.SwerveMath;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeSubsystem extends SubsystemBase{

    private static IntakeSubsystem intakeSingleton;
    public static IntakeSubsystem getInstance() {
        if (intakeSingleton==null) intakeSingleton = new IntakeSubsystem();
        return intakeSingleton;
    }

    private SparkMax motor = new SparkMax(Constants.IntakeConstants.canID, MotorType.kBrushless);
    private SparkMax motor2 = new SparkMax(Constants.IntakeConstants.canID2, MotorType.kBrushless);
    public SparkMax pivot = new SparkMax(Constants.IntakeConstants.pivotID, MotorType.kBrushless);

    SparkBaseConfig sharedConfig = new SparkMaxConfig().apply(Constants.kCoastConfig).smartCurrentLimit(30, 30);
    SparkBaseConfig pivConfig = new SparkMaxConfig().apply(Constants.kBrakeConfig).smartCurrentLimit(30, 30);
    SparkBaseConfig motorConfig = new SparkMaxConfig().apply(sharedConfig);
    SparkBaseConfig motor2Config = new SparkMaxConfig().apply(sharedConfig);
    SparkBaseConfig pivotConfig = new SparkMaxConfig().apply(pivConfig).inverted(true);

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
        return Math.abs(pivot.getClosedLoopController().getSetpoint()-pivot.getEncoder().getPosition())<1;
    }

    public Trigger atStow(){
        return new Trigger(()->(getAtPosistion()&&pivot.getClosedLoopController().getSetpoint()==IntakeConstants.stow));
    }
    public Trigger atDeploy(){
        return new Trigger(()->(getAtPosistion()&&pivot.getClosedLoopController().getSetpoint()==IntakeConstants.deploy));
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
        //SmartDashboard.putNumber("Intake Pos", pivot.getEncoder().getPosition());
        //SmartDashboard.putNumber("setpoint", pivot.getClosedLoopController().getSetpoint());
       // SmartDashboard.putNumber("dist to position", Math.abs(pivot.getClosedLoopController().getSetpoint()-pivot.getEncoder().getPosition()));
       SmartDashboard.putNumber("Intake speed", motor.getEncoder().getVelocity());
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



    public Command smartIntakeCommand() {
        if (atStow().getAsBoolean()) {
            return new SequentialCommandGroup(
                deployCommand(),
                new ParallelRaceGroup(
                    new WaitUntilCommand(atDeploy()),
                    new WaitCommand(0.25)),
                intakeCommand()
            );
        } else {
            return new SequentialCommandGroup(
                deployCommand(),
                intakeCommand()
            );
        }
    }

    public Command autoPrepIntakeCommand() {
        return this.runOnce(()->{
        deploy();
        doIntake();
        ShooterSubsystem.getInstance().setIdleHigh();
        });
    }

    




































    private boolean slowEnabled = true;
    public void toggleSlowEnabled() {
        slowEnabled = !slowEnabled;
    }

    public class IntakeAndSlowCommand extends Command {

        SwerveSubsystem drivebase;
        CommandPS5Controller gamepad;
        double startTime;

        boolean startedRunning;

        double speed = 0.6;
        double timeToSlow = 1.0;

        public IntakeAndSlowCommand(SwerveSubsystem p_drivebase, CommandPS5Controller p_gamepad) {
            drivebase = p_drivebase;
            gamepad = p_gamepad;

            addRequirements(getInstance(), drivebase);
        }

        @Override
        public void initialize() {
            deploy();
            startTime = Timer.getFPGATimestamp();
            startedRunning = false;
        }

        @Override
        public void execute() {
            double time = Timer.getFPGATimestamp() - startTime;
            if (!startedRunning && (!atStow().getAsBoolean() || time > 0.25)) {
                doIntake();
                startedRunning = true;
            }

            double mult = (slowEnabled && time > timeToSlow)
                        ? speed
                        : 1.0 - (timeToSlow-time)*(1-speed);
            
            double[] transV = SwerveSubsystem.deadband2d(-gamepad.getLeftY()*mult, -gamepad.getLeftX()*mult, Constants.OIConstants.kDriveDeadband);
            double angRot = MathUtil.applyDeadband(-gamepad.getRightX(), Constants.OIConstants.kDriveDeadband);
            drivebase.swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                                transV[0] * drivebase.swerveDrive.getMaximumChassisVelocity() * drivebase.driveMultiplier,
                                transV[1] * drivebase.swerveDrive.getMaximumChassisVelocity() * drivebase.driveMultiplier), 0.8),
                                angRot * drivebase.swerveDrive.getMaximumChassisAngularVelocity() * drivebase.driveMultiplier,
                                true,
                                true
            );


        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            stopIntake();
        }
    }




    public class IntakeShakeCommand extends Command {

        SwerveSubsystem drivebase;
        CommandPS5Controller gamepad;
        double startTime;

        boolean startedRunning;

        double speed = 0.6;
        double timeToSlow = 1.0;

        public IntakeShakeCommand() {
            addRequirements(IntakeSubsystem.getInstance());
        }
        
        @Override
        public void initialize() {
            doIntake();
            deploy();
            startTime = Timer.getFPGATimestamp();
            startedRunning = false;
        }

        @Override
        public void execute() {
            double time = Timer.getFPGATimestamp() - startTime;
            if (time%1.5 > 0.75) {
                stow();
            } else {
                deploy();
            }


        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            stow();
            stopIntake();
        }
    }
}