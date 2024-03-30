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



public class Indexer extends SubsystemBase implements Logged {
  private final CANSparkMax m_shooterIndexMotor;
 
  private final RelativeEncoder m_shooterIndexEncoder;
  
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

@Log.File
@Log.NT
double tempIndex;
@Log.File
@Log.NT
double velocityIndex;
@Log.File
@Log.NT
double currentIndex;
  /** Creates a new ShooterSubsystem. */
  public Indexer() {
    m_shooterIndexMotor = new CANSparkMax(Constants.CANConstants.SHOOTER_INDEX_MOTOR, MotorType.kBrushless);
    m_shooterIndexEncoder = m_shooterIndexMotor.getEncoder();
    this.configureIndexMotor(m_shooterIndexMotor);
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
  public void runIndex(double power){
    m_shooterIndexMotor.set(MathUtil.clamp(power, -1, 1));
  }

  public void stopIndexer() {
    m_shooterIndexMotor.stopMotor();
  }
  @Log.File
  @Log.NT
  public boolean getIndexerBeamBreak(){
    return !m_indexerBeamBreak.get();
  }

  @Override
  public void periodic() {
    if (getIndexerBeamBreak()){
        m_robotState.setHasNote(true);
      } else {
        m_robotState.setHasNote(false);
      }
    tempIndex = m_shooterIndexMotor.getMotorTemperature();
    velocityIndex = m_shooterIndexEncoder.getVelocity();
    currentIndex = m_shooterIndexMotor.getOutputCurrent();  
  }
}
