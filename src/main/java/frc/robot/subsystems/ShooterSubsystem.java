// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotState;



public class ShooterSubsystem extends SubsystemBase implements Logged {
  private final CANSparkMax m_shooterMotor;
  private final CANSparkMax m_shooterIndexMotor;
  private final TalonFX m_shooterPositionMotor;

  private final RelativeEncoder m_shooterEncoder;
  private final RelativeEncoder m_shooterIndexEncoder;
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
  
  private SoftwareLimitSwitchConfigs m_positionSoftLimitConfig = new SoftwareLimitSwitchConfigs();
  private MotionMagicConfigs m_positionMotionMagicConfigs = new MotionMagicConfigs();
  private MotorOutputConfigs m_positionMotorConfig = new MotorOutputConfigs();
  
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
  public double robotVelocity;

  @Log.File
  @Log.NT
  public Pose2d robotPose2d;

  @Log.File
  @Log.NT
  public double armPosition;

  @Log.File
  @Log.NT
  public double armVelocity;

  @Log.File
  @Log.NT
  public double armPower;

  public MotionMagicVoltage shooterPosition = new MotionMagicVoltage(0);

  private RobotState m_robotState = RobotState.getInstance();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    
    m_shooterMotor = new CANSparkMax(Constants.CANConstants.SHOOTER_MOTOR_LEADERCanId, MotorType.kBrushless);
    m_shooterIndexMotor = new CANSparkMax(Constants.CANConstants.SHOOTER_INDEX_MOTOR, MotorType.kBrushless);
    m_shooterPositionMotor = new TalonFX(Constants.CANConstants.SHOOTER_POSITION_MOTOR);

    m_shooterEncoder = m_shooterMotor.getEncoder();
    m_shooterIndexEncoder = m_shooterIndexMotor.getEncoder();
 
    m_shooterMotor.setInverted(false);
    m_shooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    //TODO: Make actual constants
    m_shooterMotor.setSmartCurrentLimit(40);
    m_shooterMotor.setClosedLoopRampRate(1);
    m_shooterMotor.burnFlash();
 

    m_RollerPidController = m_shooterMotor.getPIDController();
    m_RollerPidController.setP(shooterkPPreference.get());
    m_RollerPidController.setD(shooterkDPreference.get());
    m_RollerPidController.setI(shooterkIPreference.get());
    m_RollerPidController.setFeedbackDevice(m_shooterEncoder);
    m_RollerPidController.setOutputRange(Constants.ShooterConstants.ROLLER_MIN_OUTPUT, Constants.ShooterConstants.ROLLER_MAX_OUTPUT);
    m_RollerPidController.setFF(Constants.ShooterConstants.ROLLER_kFF);

    m_positionMotorConfig.NeutralMode = NeutralModeValue.Brake;

    m_shooterPositionMotor.getConfigurator().apply(m_positionMotorConfig, 0.050);

    positionSlot0Configs.kV = Constants.ShooterConstants.POSITION_kV;
    positionSlot0Configs.kP = positionkPPreference.get();
    positionSlot0Configs.kI = positionkIPreference.get();
    positionSlot0Configs.kD = positionkDPreference.get();

    m_shooterPositionMotor.getConfigurator().apply(positionSlot0Configs, 0.050);


    m_positionSoftLimitConfig.ForwardSoftLimitEnable = true;
    m_positionSoftLimitConfig.ForwardSoftLimitThreshold = ShooterConstants.POSITION_ForwardsLimit;
    m_positionSoftLimitConfig.ReverseSoftLimitEnable = true;
    m_positionSoftLimitConfig.ReverseSoftLimitThreshold = ShooterConstants.POSITION_ReverseLimit;

    m_shooterPositionMotor.getConfigurator().apply(m_positionSoftLimitConfig, 0.050);

    m_positionMotionMagicConfigs.MotionMagicCruiseVelocity = ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    m_positionMotionMagicConfigs.MotionMagicAcceleration = ShooterConstants.MOTION_MAGIC_ACCELERATION;
    m_positionMotionMagicConfigs.MotionMagicJerk = ShooterConstants.MOTION_MAGIC_JERK;

    m_shooterPositionMotor.getConfigurator().apply(m_positionMotionMagicConfigs);

    //configure indexer motor
    m_shooterIndexMotor.setInverted(false);
    m_shooterIndexMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    //TODO: Make actual constants
    m_shooterIndexMotor.setSmartCurrentLimit(40);
    m_shooterIndexMotor.setClosedLoopRampRate(1);
    m_shooterIndexMotor.burnFlash();

      
  }

  public void spinPower(double power) {
    m_shooterMotor.set(MathUtil.clamp(power, Constants.ShooterConstants.ROLLER_MIN_OUTPUT, Constants.ShooterConstants.ROLLER_MAX_OUTPUT));
  }

  public void spinRPM(double rpm) {
    m_RollerPidController.setReference(MathUtil.clamp(rpm, -Constants.ShooterConstants.ROLLER_MAX_RPM, Constants.ShooterConstants.ROLLER_MAX_RPM), CANSparkFlex.ControlType.kVelocity);
  }

  public void runIndex(double power){
    m_shooterIndexMotor.set(MathUtil.clamp(power, -1, 1));
  }

  public double getPositionRevolutions(){
    return m_shooterPositionMotor.getPosition().getValueAsDouble();
  }

  @Log.File
  @Log.NT
  public double getAngleDegrees(){
    return getPositionRevolutions()*ShooterConstants.POSITION_DEGREE_PER_MOTOR_REV;
  }

  public double getAngleRadians(){
    return Units.degreesToRadians(getAngleDegrees());
  }


  @Log.File
  @Log.NT
  public Pose3d getStartPose3d(){
    return new Pose3d(robotPose2d).plus(new Transform3d(ShooterConstants.TRANSLATION_OFFSET, 0, ShooterConstants.ELEVATION,
    new Rotation3d(0, -getAngleRadians(), Math.PI)));
  }


  /* 
  @Log.File
  @Log.NT
  public Pose3d getEndPose3d(){
    return new Pose3d(new Translation3d(robotPose3d.getX()+Math.cos(getAngle())*ShooterConstants.LENGTH, robotPose3d.getY(), 
              Math.sin(getAngle())*ShooterConstants.LENGTH + ShooterConstants.ELEVATION), new Rotation3d(0, getAngle(),0));
  }

  */
  
  @Log.File
  @Log.NT
  public double getShootingVelocity()
  {
    return velocity+robotVelocity;
  }

  public void setPositionRevolutions(double position) {
    this.targetSetPoint = position;
    m_shooterPositionMotor.setControl(shooterPosition.withPosition(position));
  }

  public void setAngleDegrees(double angle){
    setPositionRevolutions(angle/ShooterConstants.POSITION_DEGREE_PER_MOTOR_REV);
  }

  public boolean atSetpoint() {
    if(Robot.isSimulation()) return true;
    return getPositionRevolutions() >= targetSetPoint - Constants.ShooterConstants.POSITION_TOLERANCE && getPositionRevolutions() <= targetSetPoint + Constants.ShooterConstants.POSITION_TOLERANCE; 
  }

  public void stopRoller() {
    m_shooterMotor.stopMotor();
  }

  public void moveArm(double motorpower) {
    m_shooterPositionMotor.set(MathUtil.clamp(motorpower, -1, 1));
  }


  public void stopIndexer() {
    m_shooterIndexMotor.stopMotor();
  }

  public void stopPositioner(){
    m_shooterPositionMotor.stopMotor();
  }

  public void resetPosition(){
    this.setPositionRevolutions(0);
  }

  public void stopAll(){
    this.stopRoller();
    this.stopIndexer();
    this.resetPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    temp = m_shooterMotor.getMotorTemperature();
    velocity = m_shooterEncoder.getVelocity();
    current = m_shooterMotor.getOutputCurrent();
    
    tempIndex = m_shooterIndexMotor.getMotorTemperature();
    velocityIndex = m_shooterIndexEncoder.getVelocity();
    currentIndex = m_shooterIndexMotor.getOutputCurrent();

   armPower = m_shooterPositionMotor.get();
   armPosition = m_shooterPositionMotor.getPosition().getValueAsDouble();
   armVelocity = m_shooterPositionMotor.getVelocity().getValueAsDouble();
    
 
    

    //TODO remove once tuned.
    // m_RollerPidController.setP(shooterkPPreference.get());
    // m_RollerPidController.setI(shooterkIPreference.get());
    // m_RollerPidController.setD(shooterkDPreference.get());
    
    // // apply gains, 50 ms total timeout
    // //TODO remove once tuned.
    // positionSlot0Configs.kP = positionkPPreference.get();
    // positionSlot0Configs.kI = positionkIPreference.get();
    // positionSlot0Configs.kD = positionkDPreference.get();
    
    // m_shooterPositionMotor.getConfigurator().apply(positionSlot0Configs, 0.050);

    robotPose2d = m_robotState.getRobotPose();
  
}
public void simulationPeriodic(){
  m_shooterPositionMotor.setPosition(targetSetPoint);
}






}
