// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
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

  
  
  private double kP, kI, kD, kFF, kMaxOutput, kMinOutput, maxRPM;


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

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    shooterMotor = new CANSparkMax(Constants.SHOOTER_MOTOR_LEADER, MotorType.kBrushless);
    shooterIndexMotor = new CANSparkMax(Constants.SHOOTER_INDEX_MOTOR, MotorType.kBrushless);
    shooterPositionMotor = new TalonFX(7);
 
    
    

    shooterMotor.setInverted(false);

    shooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
  

    //TODO: Make actual constants
    shooterMotor.setSmartCurrentLimit(40);
 

    shooterMotor.setClosedLoopRampRate(1);
    
    shooterMotor.burnFlash();
 

    m_PidController = shooterMotor.getPIDController();
  }

  public void spinPower(double power) {
    shooterMotor.set(MathUtil.clamp(power, -1, 1));
  }

  public void spinRPM(double rpm) {
    m_PidController.setReference(MathUtil.clamp(rpm, -maxRPM, maxRPM), CANSparkFlex.ControlType.kVelocity);
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

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_PidController.setP(p); kP = p; }
    if((i != kI)) { m_PidController.setI(i); kI = i; }
    if((d != kD)) { m_PidController.setD(d); kD = d; }
    if((ff != kFF)) { m_PidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_PidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    // set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.11;
    slot0Configs.kI = 0.5;
    slot0Configs.kD = 0.001;

    // apply gains, 50 ms total timeout
    shooterPositionMotor.getConfigurator().apply(slot0Configs, 0.050);
  
}
}
