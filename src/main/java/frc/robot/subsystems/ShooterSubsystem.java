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



public class ShooterSubsystem extends SubsystemBase implements Logged {
  private final CANSparkMax shooterMotor;
  private final CANSparkMax shooterIndexMotor;
  private final TalonFX shooterPositionMotor;

  private final SparkPIDController m_PidController;

  
  //TODO change these name
  private Slot0Configs slot0Configs = new Slot0Configs();

// shooterPosition variables
private double shooterPositionkV;
private double shooterPositionkP;
private double shooterPositionkI;
private double shooterPostionkD;

// shooterRoller preferences
private DoublePreference shooterkPPreference = new DoublePreference("shooter/kP", Constants.ShooterConstants.kRPMP);
private DoublePreference shooterkIPreference = new DoublePreference("shooter/kI", Constants.ShooterConstants.kRPMI);
private DoublePreference shooterkDPreference = new DoublePreference("shooter/kD", Constants.ShooterConstants.kRPMD);

private double TOLERANCE = ShooterConstants.POSITION_TOLERANCE;
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

  public MotionMagicVoltage shooterPosition = new MotionMagicVoltage(0);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {



    shooterMotor = new CANSparkMax(Constants.CANConstants.SHOOTER_MOTOR_LEADERCanId, MotorType.kBrushless);
    shooterIndexMotor = new CANSparkMax(Constants.CANConstants.SHOOTER_INDEX_MOTORCanId, MotorType.kBrushless);
    //TODO move canID to contants class
    shooterPositionMotor = new TalonFX(Constants.CANConstants.kshooterPositionMotorCanId);
 
    shooterMotor.setInverted(false);

    shooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
  
     
    slot0Configs.kV = shooterPositionkV;
    slot0Configs.kP = shooterPositionkP;
    slot0Configs.kI = shooterPositionkI;
    slot0Configs.kD = shooterPostionkD;

    //TODO: Make actual constants
    shooterMotor.setSmartCurrentLimit(40);
 
    shooterMotor.setClosedLoopRampRate(1);
    
    shooterMotor.burnFlash();
 

    m_PidController = shooterMotor.getPIDController();
    m_PidController.setP(shooterkPPreference.get());
    m_PidController.setD(shooterkDPreference.get());
    m_PidController.setI(shooterkIPreference.get());
    m_PidController.setOutputRange(Constants.ShooterConstants.shooterRollerkMinOutput, Constants.ShooterConstants.shooterRollerkMaxOutput);
    m_PidController.setFF(Constants.ShooterConstants.shooterRollerkFF);
  }

  public void spinPower(double power) {
    shooterMotor.set(MathUtil.clamp(power, -1, 1));
  }

  public void spinRPM(double rpm) {
    m_PidController.setReference(MathUtil.clamp(rpm, -Constants.ShooterConstants.shooterRollermaxRPM, Constants.ShooterConstants.shooterRollermaxRPM), CANSparkFlex.ControlType.kVelocity);
  }

  @Log.File
  @Log.NT
  public double getPosition(){
    return shooterPositionMotor.getPosition().getValueAsDouble();
  }

  public void setPosition(double position) {
    this.targetSetPoint = position;
    shooterPositionMotor.setControl(shooterPosition.withPosition(position));
  }

  public boolean atSetpoint() {
    if(Robot.isSimulation()) return true;
    return getPosition() >= targetSetPoint - TOLERANCE && getPosition() <= targetSetPoint + 500; 
  }

  public void stop() {
    shooterMotor.stopMotor();
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

 
    


    m_PidController.setP(shooterkPPreference.get());
    m_PidController.setI(shooterkIPreference.get());
    m_PidController.setD(shooterkDPreference.get());
    
    // apply gains, 50 ms total timeout
    shooterPositionMotor.getConfigurator().apply(slot0Configs, 0.050);
  
}
}
