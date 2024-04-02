// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Annotations.Log;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotState;



public class Indexer extends SubsystemBase implements Logged {
  private final CANSparkMax m_shooterIndexMotor;
 
  private final RelativeEncoder m_shooterIndexEncoder;

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
