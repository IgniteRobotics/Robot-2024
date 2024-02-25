// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.comm.preferences.DoublePreference;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Annotations.Log;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import java.lang.Math;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;



public class ShooterSubsystem extends SubsystemBase implements Logged {
  private final CANSparkMax shooterMotor;
  private final CANSparkMax shooterIndexMotor;
  private final TalonFX shooterPositionMotor;

  private final SparkPIDController m_RollerPidController;

  
  //TODO change these name
  private Slot0Configs positionSlot0Configs = new Slot0Configs();

  // shooterRoller preferences
  private DoublePreference shooterkPPreference = new DoublePreference("shooter/RPMkP", Constants.ShooterConstants.ROLLER_kP);
  private DoublePreference shooterkIPreference = new DoublePreference("shooter/RPMkI", Constants.ShooterConstants.ROLLER_kI);
  private DoublePreference shooterkDPreference = new DoublePreference("shooter/RPMkD", Constants.ShooterConstants.ROLLER_kD);

  // position PID preferences
  private DoublePreference positionkPPreference = new DoublePreference("shooter/PositionkP", Constants.ShooterConstants.POSITION_kD);
  private DoublePreference positionkIPreference = new DoublePreference("shooter/PositionkI", Constants.ShooterConstants.POSITION_kD);
  private DoublePreference positionkDPreference = new DoublePreference("shooter/PositionkD", Constants.ShooterConstants.POSITION_kD);
  
  @Log.File
  @Log.NT
  private double temp;

  @Log.File
  @Log.NT
  private double current;

  @Log.File
  @Log.NT
  private double velocity;

  @Log.File
  @Log.NT
  private double tempIndex;

  @Log.File
  @Log.NT
  private double currentIndex;

  @Log.File
  @Log.NT
  private double velocityIndex;

  @Log.File
  @Log.NT
  private double targetSetPoint = 0;

  @Log.File
  @Log.NT
  private Pose2d robotPose2d;

  public MotionMagicVoltage shooterPosition = new MotionMagicVoltage(0);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {



    shooterMotor = new CANSparkMax(Constants.CANConstants.SHOOTER_MOTOR_LEADERCanId, MotorType.kBrushless);
    shooterIndexMotor = new CANSparkMax(Constants.CANConstants.SHOOTER_INDEX_MOTOR, MotorType.kBrushless);
    shooterPositionMotor = new TalonFX(Constants.CANConstants.SHOOTER_POSITION_MOTOR);
 
    shooterMotor.setInverted(false);
    shooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    //TODO: Make actual constants
    shooterMotor.setSmartCurrentLimit(40);
    shooterMotor.setClosedLoopRampRate(1);
    shooterMotor.burnFlash();
 

    m_RollerPidController = shooterMotor.getPIDController();
    m_RollerPidController.setP(shooterkPPreference.get());
    m_RollerPidController.setD(shooterkDPreference.get());
    m_RollerPidController.setI(shooterkIPreference.get());
    m_RollerPidController.setOutputRange(Constants.ShooterConstants.ROLLER_MIN_OUTPUT, Constants.ShooterConstants.ROLLER_MAX_OUTPUT);
    m_RollerPidController.setFF(Constants.ShooterConstants.ROLLER_kFF);

    positionSlot0Configs.kV = Constants.ShooterConstants.POSITION_kV;
    positionSlot0Configs.kP = positionkPPreference.get();
    positionSlot0Configs.kI = positionkIPreference.get();
    positionSlot0Configs.kD = positionkDPreference.get();

    //configure indexer motor
    shooterIndexMotor.setInverted(false);
    shooterIndexMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    //TODO: Make actual constants
    shooterIndexMotor.setSmartCurrentLimit(40);
    shooterIndexMotor.setClosedLoopRampRate(1);
    shooterIndexMotor.burnFlash();
  }

  public void spinPower(double power) {
    shooterMotor.set(MathUtil.clamp(power, Constants.ShooterConstants.ROLLER_MIN_OUTPUT, Constants.ShooterConstants.ROLLER_MAX_OUTPUT));
  }

  public void spinRPM(double rpm) {
    m_RollerPidController.setReference(MathUtil.clamp(rpm, -Constants.ShooterConstants.ROLLER_MAX_RPM, Constants.ShooterConstants.ROLLER_MAX_RPM), CANSparkFlex.ControlType.kVelocity);
  }

  public void runIndex(double power){
    shooterIndexMotor.set(MathUtil.clamp(power, -1, 1));
  }

  @Log.File
  @Log.NT
  public double getPosition(){
    return shooterPositionMotor.getPosition().getValueAsDouble();
  }

  public double getAngle(){
    return getPosition();
  }

  @Log.File
  @Log.NT
  public Pose3d getPose3d(){
    return new Pose3d(new Translation3d(robotPose2d.getX()+Math.cos(getAngle())*ShooterConstants.LENGTH, robotPose2d.getY(), 
              Math.sin(getAngle())*ShooterConstants.LENGTH), new Rotation3d(0, getAngle(),0));
  }

  public double getShootingVelocity(double robotvelocity)
  {
    return velocity+robotvelocity;
  }

  public void setPosition(double position) {
    this.targetSetPoint = position;
    shooterPositionMotor.setControl(shooterPosition.withPosition(position));
  }

  public boolean atSetpoint() {
    if(Robot.isSimulation()) return true;
    return getPosition() >= targetSetPoint - Constants.ShooterConstants.POSITION_TOLERANCE && getPosition() <= targetSetPoint + Constants.ShooterConstants.POSITION_TOLERANCE; 
  }

  public void stopRoller() {
    shooterMotor.stopMotor();
  }

  public void stopIndexer() {
    shooterIndexMotor.stopMotor();
  }

  public void stopPositioner(){
    shooterPositionMotor.stopMotor();
  }

  public void resetPosition(){
    this.setPosition(0);
  }

  public void stopAll(){
    this.stopRoller();
    this.stopIndexer();
    this.resetPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    temp = shooterMotor.getMotorTemperature();
    velocity = shooterMotor.getEncoder().getVelocity();
    current = shooterMotor.getOutputCurrent();
    
    tempIndex = shooterIndexMotor.getMotorTemperature();
    velocityIndex = shooterIndexMotor.getEncoder().getVelocity();
    currentIndex = shooterIndexMotor.getOutputCurrent();

 
    

    //TODO remove once tuned.
    m_RollerPidController.setP(shooterkPPreference.get());
    m_RollerPidController.setI(shooterkIPreference.get());
    m_RollerPidController.setD(shooterkDPreference.get());
    
    // apply gains, 50 ms total timeout
    //TODO remove once tuned.
    positionSlot0Configs.kP = positionkPPreference.get();
    positionSlot0Configs.kI = positionkIPreference.get();
    positionSlot0Configs.kD = positionkDPreference.get();
    
    shooterPositionMotor.getConfigurator().apply(positionSlot0Configs, 0.050);
  
}
}
