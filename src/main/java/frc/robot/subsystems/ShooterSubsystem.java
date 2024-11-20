// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import java.lang.Math;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.Servo;
import com.ctre.phoenix6.signals.GravityTypeValue;



public class ShooterSubsystem extends SubsystemBase implements Logged {
  private final CANSparkMax m_shooterMotor;
  private final CANSparkMax m_shooterIndexMotor;
  private final TalonFX m_shooterPositionMotor;
  private final CANcoder m_shooterPositionCancoder;
  private final Servo m_RightServo;
  private final Servo m_LeftServo;

  private final RelativeEncoder m_shooterEncoder;
  private final RelativeEncoder m_shooterIndexEncoder;
  private final SparkPIDController m_RollerPidController;

  
  //TODO change these name
  private Slot0Configs positionSlot0Configs = new Slot0Configs();

  // shooterRoller preferences
  private DoublePreference shooterkPPreference = new DoublePreference("shooter/RPMkP", Constants.ShooterConstants.ROLLER_kP);
  private DoublePreference shooterkIPreference = new DoublePreference("shooter/RPMkI", Constants.ShooterConstants.ROLLER_kI);
  private DoublePreference shooterkDPreference = new DoublePreference("shooter/RPMkD", Constants.ShooterConstants.ROLLER_kD);
  private DoublePreference shooterkFPreference = new DoublePreference("shooter/RPMkF", Constants.ShooterConstants.ROLLER_kF);
  private SoftwareLimitSwitchConfigs m_positionSoftLimitConfig = new SoftwareLimitSwitchConfigs();
  private MotionMagicConfigs m_positionMotionMagicConfigs = new MotionMagicConfigs();
  private MotorOutputConfigs m_positionMotorConfig = new MotorOutputConfigs();
  private TalonFXConfiguration m_fxCfg = new TalonFXConfiguration();

  public MotionMagicVoltage shooterPosition = new MotionMagicVoltage(0);
  private RobotState m_robotState = RobotState.getInstance();


  private DigitalInput m_indexerBeamBreak = new DigitalInput(0);

  //temp preference
  private DoublePreference shooterPositionkGPreference = new DoublePreference("shooter/positionkG", Constants.ShooterConstants.POSITION_kG);


  /*********************  Telemetry Variables *********************/

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
  private double targetVelocity;

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
  private double targetPosition = 0;

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
  public double armCancoderPosition;

  @Log.File
  @Log.NT
  public double armCancoderVelocity;

  @Log.File
  @Log.NT
  public double armPower;

  @Log.File
  @Log.NT
  public double armVoltage;

  @Log.File
  @Log.NT
  public double armTemp;

  @Log.File
  @Log.NT
  public double armCurrent;

  private String armNeutralMode;

  private boolean armCurrentFault;

  private boolean armRevLimiFault;

  @Log.File
  @Log.NT
  private boolean Ready = true;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    
    m_shooterMotor = new CANSparkMax(Constants.CANConstants.SHOOTER_MOTOR_LEADERCanId, MotorType.kBrushless);
    m_shooterIndexMotor = new CANSparkMax(Constants.CANConstants.SHOOTER_INDEX_MOTOR, MotorType.kBrushless);
    m_shooterPositionMotor = new TalonFX(Constants.CANConstants.SHOOTER_POSITION_MOTOR);
    m_shooterPositionCancoder = new CANcoder(Constants.CANConstants.SHOOTER_POSITION_CANCODER);
  

    m_shooterEncoder = m_shooterMotor.getEncoder();
    m_shooterIndexEncoder = m_shooterIndexMotor.getEncoder();
    
    m_RollerPidController = m_shooterMotor.getPIDController();

    m_RightServo = new Servo(ShooterConstants.RIGHT_SERVO_PORT);
    m_LeftServo = new Servo(ShooterConstants.LEFT_SERVO_PORT);

    this.configureShooterMotor(m_shooterMotor, m_shooterEncoder, m_RollerPidController);
    this.configureIndexMotor(m_shooterIndexMotor);
    this.configureCancoder(m_shooterPositionCancoder);
    this.configurePositionMotor(m_shooterPositionMotor,m_fxCfg, m_positionMotorConfig, positionSlot0Configs, m_positionSoftLimitConfig, m_positionMotionMagicConfigs);
      
  }

  private void configureShooterMotor(CANSparkMax motor, RelativeEncoder encoder, SparkPIDController pidController){
    encoder.setVelocityConversionFactor(1);
    motor.setInverted(false);
    motor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    //TODO: Make actual constants
    motor.setSmartCurrentLimit(40);
    motor.setClosedLoopRampRate(1);
    motor.burnFlash();
    pidController.setP(shooterkPPreference.get());
    pidController.setD(shooterkDPreference.get());
    pidController.setI(shooterkIPreference.get());
    pidController.setFeedbackDevice(m_shooterEncoder);
    pidController.setOutputRange(Constants.ShooterConstants.ROLLER_MIN_OUTPUT, Constants.ShooterConstants.ROLLER_MAX_OUTPUT);
    pidController.setFF(shooterkFPreference.get());

  }

  private void configureIndexMotor(CANSparkMax motor){
     //configure indexer motor
     motor.setInverted(false);
     motor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
     //TODO: Make actual constants
     motor.setSmartCurrentLimit(40);
     motor.setClosedLoopRampRate(0);
     motor.burnFlash();
  }

  private void configurePositionMotor(TalonFX motor,
      TalonFXConfiguration baseConfiguration,  
      MotorOutputConfigs motorOutputConfigs,
      Slot0Configs slot0PID, 
      SoftwareLimitSwitchConfigs limitSwitchConfigs,
      MotionMagicConfigs mmConfig){

    TalonFXConfigurator configurator = m_shooterPositionMotor.getConfigurator();

    //grab any settings already on the motor, just in case.
    configurator.refresh(baseConfiguration);
    configurator.refresh(motorOutputConfigs);
    configurator.refresh(slot0PID);
    configurator.refresh(limitSwitchConfigs);
    configurator.refresh(mmConfig);
    
    baseConfiguration.Feedback.FeedbackRemoteSensorID = m_shooterPositionCancoder.getDeviceID();
    baseConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    
    configurator.apply(baseConfiguration);

    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    configurator.apply(motorOutputConfigs, 0.050);

    slot0PID.kV = Constants.ShooterConstants.POSITION_kV;
    slot0PID.kS = Constants.ShooterConstants.POSITION_kS;
    slot0PID.kP = Constants.ShooterConstants.POSITION_kP;
    slot0PID.kI = Constants.ShooterConstants.POSITION_kI;
    slot0PID.kD = Constants.ShooterConstants.POSITION_kD;
    slot0PID.kG = shooterPositionkGPreference.getValue(); 
    slot0PID.GravityType = GravityTypeValue.Arm_Cosine;

    configurator.apply(slot0PID, 0.050);


    limitSwitchConfigs.ForwardSoftLimitEnable = true;
    limitSwitchConfigs.ForwardSoftLimitThreshold = ShooterConstants.POSITION_ForwardsLimit;
    limitSwitchConfigs.ReverseSoftLimitEnable = true;
    limitSwitchConfigs.ReverseSoftLimitThreshold = ShooterConstants.POSITION_ReverseLimit;

    configurator.apply(limitSwitchConfigs, 0.050);

    mmConfig.MotionMagicCruiseVelocity = ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    mmConfig.MotionMagicAcceleration = ShooterConstants.MOTION_MAGIC_ACCELERATION;
    mmConfig.MotionMagicJerk = ShooterConstants.MOTION_MAGIC_JERK;

    configurator.apply(mmConfig);
    
  }

  private void configureCancoder(CANcoder cancoder){
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    config.MagnetSensor.MagnetOffset = 0;
    cancoder.getConfigurator().apply(config);
    cancoder.getAbsolutePosition().setUpdateFrequency(100);
    cancoder.getPosition().setUpdateFrequency(100);
    cancoder.getVelocity().setUpdateFrequency(100);
    cancoder.setPosition(degreesToCANcoder(0.5, Constants.ShooterConstants.ARM_CANCODER_RATIO));
  }
  
  public void spinPower(double power) {
    m_shooterMotor.set(MathUtil.clamp(power, Constants.ShooterConstants.ROLLER_MIN_OUTPUT, Constants.ShooterConstants.ROLLER_MAX_OUTPUT));
  }

  public void spinRPM(double rpm) {
    targetVelocity = rpm;
    m_RollerPidController.setReference(MathUtil.clamp(rpm, -Constants.ShooterConstants.ROLLER_MAX_RPM, Constants.ShooterConstants.ROLLER_MAX_RPM), CANSparkFlex.ControlType.kVelocity);
  }

  @Log.File
  @Log.NT
  public boolean atRPM(){
    if(Robot.isSimulation()) return true;
    return velocity >= targetVelocity - Constants.ShooterConstants.VELOCITY_TOLERANCE && velocity <= targetVelocity + Constants.ShooterConstants.VELOCITY_TOLERANCE;
  }

  public void runIndex(double power){
    m_shooterIndexMotor.set(MathUtil.clamp(power, -1, 1));
  }

  // public double getPositionRevolutions(){
  //   return m_shooterPositionMotor.getPosition().getValueAsDouble();
  // }

  // @Log.File
  // @Log.NT
  // public double getRotorAngleDegrees(){
  //   return m_shooterPositionMotor.getPosition().getValueAsDouble()*ShooterConstants.POSITION_DEGREE_PER_MOTOR_REV;
  // }

  public double getAngleRadians(){
    return Units.degreesToRadians(getAngleDegrees());
  }

  @Log.File
  @Log.NT
  public double getAngleDegrees(){
    return CANcoderToDegrees(m_shooterPositionCancoder.getAbsolutePosition().getValueAsDouble(), Constants.ShooterConstants.ARM_CANCODER_RATIO);
  }


  @Log.File
  @Log.NT
  public Pose3d getStartPose3d(){
    return new Pose3d(robotPose2d).plus(new Transform3d(ShooterConstants.TRANSLATION_OFFSET, 0, ShooterConstants.ELEVATION,
    new Rotation3d(0, -getAngleRadians(), Math.PI)));
  }
  
  @Log.File
  @Log.NT
  public double getShootingVelocity()
  {
    return velocity+robotVelocity;
  }

  private void setPositionRevolutions(double position) {
    m_shooterPositionMotor.setControl(shooterPosition.withPosition(position));
    //m_shooterPositionMotor.setControl(mmDutyCycleRequest.withPosition(position));
  }

  public void setAngleDegrees(double angle){
    this.targetPosition = angle;
    //setPositionRevolutions(angle.ShooterConstants.POSITION_DEGREE_PER_MOTOR_REV);
    setPositionRevolutions(degreesToCANcoder(angle, Constants.ShooterConstants.ARM_CANCODER_RATIO));
  }

  @Log.File
  @Log.NT
  public boolean armAtSetpoint() {
    if(Robot.isSimulation()) return true;
    return getAngleDegrees() >= targetPosition - Constants.ShooterConstants.POSITION_TOLERANCE && getAngleDegrees() <= targetPosition + Constants.ShooterConstants.POSITION_TOLERANCE; 
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
    this.setAngleDegrees(0);
  }
  
    /**
   * @param positionCounts CANCoder Position Counts
   * @param gearRatio Gear Ratio between CANCoder and Mechanism
   * @return Degrees of Rotation of Mechanism
   */

  public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
    //return positionCounts * (360.0 / (gearRatio * 4096.0));
    return positionCounts * 360 /gearRatio;
  }

  /**
   * @param degrees Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between CANCoder and Mechanism
   * @return CANCoder Position Counts
   */
  public static double degreesToCANcoder(double degrees, double gearRatio) {
      return degrees * gearRatio /360.0;
  }


  public void stopAll(){
    this.stopRoller();
    this.stopIndexer();
    this.resetPosition();
  }

  @Log.File
  @Log.NT
  public boolean getIndexerBeamBreak(){
    return !m_indexerBeamBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (getIndexerBeamBreak()){
      m_robotState.setHasNote(true);
    } else {
      m_robotState.setHasNote(false);
    }

    temp = m_shooterMotor.getMotorTemperature();
    velocity = m_shooterEncoder.getVelocity();
    current = m_shooterMotor.getOutputCurrent();

    tempIndex = m_shooterIndexMotor.getMotorTemperature();
    velocityIndex = m_shooterIndexEncoder.getVelocity();
    currentIndex = m_shooterIndexMotor.getOutputCurrent();

    armPower = m_shooterPositionMotor.get();
    armPosition = m_shooterPositionMotor.getPosition().getValueAsDouble();
    armVelocity = m_shooterPositionMotor.getVelocity().getValueAsDouble();
    armVoltage = m_shooterPositionMotor.getMotorVoltage().getValueAsDouble();
    armTemp = m_shooterPositionMotor.getDeviceTemp().getValueAsDouble();
    armCurrent = m_shooterPositionMotor.getTorqueCurrent().getValueAsDouble();
    armCancoderPosition = m_shooterPositionCancoder.getAbsolutePosition().getValueAsDouble();
    armCancoderVelocity = m_shooterPositionCancoder.getVelocity().getValueAsDouble();

    //m_shooterPositionMotor.getConfigurator().refresh(fxCfg);
    //armNeutralMode = fxCfg.MotorOutput.NeutralMode.toString();
    // armCurrentFault = m_shooterPositionMotor.getFault_StatorCurrLimit().getValue();
    // armRevLimiFault = m_shooterPositionMotor.getFault_ReverseSoftLimit().getValue();
    

    //TODO remove once tuned.
    // m_RollerPidController.setP(shooterkPPreference.get());
    // m_RollerPidController.setI(shooterkIPreference.get());
    // m_RollerPidController.setD(shooterkDPreference.get());
    // m_RollerPidController.setFF(shooterkFPreference.get());
    
    // apply gains, 50 ms total timeout
    //TODO remove once tuned.
    // positionSlot0Configs.kP = positionkPPreference.get();
    // positionSlot0Configs.kI = positionkIPreference.get();
    // positionSlot0Configs.kD = positionkDPreference.get();
    
    //m_shooterPositionMotor.getConfigurator().apply(positionSlot0Configs, 0.050);

    robotPose2d = m_robotState.getRobotPose();
  
  }

  public void setPositionMotorSpeed(double speed){
    m_shooterPositionMotor.set(speed);
  }

  public Command powerTestBuilder(double staticTimeout, double powerForward, double powerBackward){
    return
      new InstantCommand(() -> this.resetPosition()).withTimeout(staticTimeout)

      .andThen(new InstantCommand(() -> this.setPositionMotorSpeed(powerForward)).withTimeout(staticTimeout))
      .andThen(new InstantCommand(() -> this.setPositionMotorSpeed(powerBackward)).withTimeout(staticTimeout))
      .andThen(new InstantCommand(() -> this.resetPosition()).withTimeout(staticTimeout));
  }
public Command positionerTestBuilder(double staticTimeout, double angle1, double angle2, double angle3){
    return
      new InstantCommand(() -> this.resetPosition()).withTimeout(staticTimeout)

      //angle 1 test
      .andThen(new InstantCommand(() -> this.setAngleDegrees(angle1)).withTimeout(staticTimeout))
      .andThen(new InstantCommand(() -> this.resetPosition()).withTimeout(staticTimeout))
      .andThen(new WaitCommand(2))
    

      //angle 2 test
      .andThen(new InstantCommand(() -> this.setAngleDegrees(angle2)).withTimeout(staticTimeout))
      .andThen(new InstantCommand(() -> this.resetPosition()).withTimeout(staticTimeout))
      .andThen(new WaitCommand(2))

      //angle 3 test
      .andThen(new InstantCommand(() -> this.setAngleDegrees(angle3)).withTimeout(staticTimeout))
      .andThen(new InstantCommand(() -> this.resetPosition()).withTimeout(staticTimeout))
      .andThen(new WaitCommand(2));

  }


  public void simulationPeriodic(){
    m_shooterPositionMotor.setPosition(targetPosition);
  }

  public void setReady(boolean bool){
    Ready = bool;
  }

  public boolean getReady(){
    return Ready;
  }

  //position 0.0-1.0 (one extreme to other extreme)
  public void setServoPosition(double position){
    m_RightServo.set(position);
    m_LeftServo.set(1.0 - position);
  }

  public void resetServoPosition(){
    m_RightServo.set(0);
    m_LeftServo.set(1);
  }

}
