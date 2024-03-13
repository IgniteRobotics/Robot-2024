// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.comm.preferences.DoublePreference;
import monologue.Annotations.Log;
import monologue.Logged;

public class Climber extends SubsystemBase implements Logged {
  private final TalonFX m_climberMotor;

  private Slot0Configs m_Slot0Configs = new Slot0Configs();

  private DoublePreference m_kPPreference = new DoublePreference("climber/kP", ClimberConstants.POSITION_kP);
  private DoublePreference m_kIPreference = new DoublePreference("climber/kI", ClimberConstants.POSITION_kI);
  private DoublePreference m_kDPreference = new DoublePreference("climber/kD", ClimberConstants.POSITION_kD);

  private DoublePreference m_powerPreference = new DoublePreference("climber/power", ClimberConstants.POWER);

  
  private SoftwareLimitSwitchConfigs m_softLimitConfig = new SoftwareLimitSwitchConfigs();
  private MotionMagicConfigs m_motionMagicConfigs = new MotionMagicConfigs();
  private MotorOutputConfigs m_motorConfig = new MotorOutputConfigs();
  private TalonFXConfiguration m_fxCfg = new TalonFXConfiguration();

  public MotionMagicVoltage m_MMPosition =   new MotionMagicVoltage(0);

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
  private double targetPosition = 0;

  @Log.File
  @Log.NT
  private double position;

  @Log.File
  @Log.NT
  private double power;


  /** Creates a new Climber. */
  public Climber() {
    m_climberMotor = new TalonFX(Constants.CANConstants.CLIMBER_MOTOR);

    m_motorConfig.NeutralMode = NeutralModeValue.Brake;
    m_motorConfig.withInverted(InvertedValue.Clockwise_Positive);

    m_climberMotor.getConfigurator().apply(m_motorConfig, 0.050);

    m_Slot0Configs.kV = Constants.ClimberConstants.POSITION_kV;
    m_Slot0Configs.kS = Constants.ClimberConstants.POSITION_kS;
    m_Slot0Configs.kP = m_kPPreference.get();
    m_Slot0Configs.kI = m_kIPreference.get();
    m_Slot0Configs.kD = m_kDPreference.get();

    m_climberMotor.getConfigurator().apply(m_Slot0Configs, 0.050);


    m_softLimitConfig.ForwardSoftLimitEnable = true;
    m_softLimitConfig.ForwardSoftLimitThreshold = ClimberConstants.UPPER_LIMIT;
    m_softLimitConfig.ReverseSoftLimitEnable = true;
    m_softLimitConfig.ReverseSoftLimitThreshold = ClimberConstants.LOWER_LIMIT;

    m_climberMotor.getConfigurator().apply(m_softLimitConfig, 0.050);

    m_motionMagicConfigs.MotionMagicCruiseVelocity = ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    m_motionMagicConfigs.MotionMagicAcceleration = ShooterConstants.MOTION_MAGIC_ACCELERATION;
    m_motionMagicConfigs.MotionMagicJerk = ShooterConstants.MOTION_MAGIC_JERK;

    m_climberMotor.getConfigurator().apply(m_motionMagicConfigs);

  }

  public double getPositionRevolutions(){
    return m_climberMotor.getPosition().getValueAsDouble();
  }

  public void setPositionRevolutions(double position) {
    this.targetPosition = position;
    m_climberMotor.setControl(m_MMPosition.withPosition(position));
  }

  @Log.File
  @Log.NT
  public boolean atSetpoint() {
    if(Robot.isSimulation()) return true;
    return getPositionRevolutions() >= targetPosition - Constants.ClimberConstants.CLIMBER_TOLERANCE && getPositionRevolutions() <= targetPosition + Constants.ClimberConstants.CLIMBER_TOLERANCE; 
  }

  public void move(double power){
    m_climberMotor.set(MathUtil.clamp(power, -1, 1));
  }

  public void stop(){
    m_climberMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    temp = m_climberMotor.getDeviceTemp().getValueAsDouble();
    velocity = m_climberMotor.getVelocity().getValueAsDouble();
    current = m_climberMotor.getStatorCurrent().getValueAsDouble();
    position = m_climberMotor.getPosition().getValueAsDouble();
  }
}
