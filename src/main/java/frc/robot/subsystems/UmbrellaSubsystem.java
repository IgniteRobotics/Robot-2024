// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import monologue.Logged;
import monologue.Annotations.Log;


public class UmbrellaSubsystem extends SubsystemBase implements Logged{

  private final CANSparkFlex  umbrellaMotor; 
  private final RelativeEncoder m_umbrellaEncoder;


  @Log.File
  @Log.NT
  private double current;
  @Log.File
  @Log.NT
  private double velocity;
  @Log.File
  @Log.NT
  private double temperature;


  public UmbrellaSubsystem() {
    umbrellaMotor = new CANSparkFlex(Constants.CANConstants.UMBRELLA_MOTOR_1, MotorType.kBrushless);
    m_umbrellaEncoder = umbrellaMotor.getEncoder();

    //configuring motor
    umbrellaMotor.setInverted(false);//TODO: fix when certain
    umbrellaMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    umbrellaMotor.setSmartCurrentLimit(40); //maybe?
    umbrellaMotor.burnFlash();
  }

  public void setSpeed (double speed)
    {
      umbrellaMotor.set(MathUtil.clamp(speed, -1, 1));
    }
  
  public void stop(){
      umbrellaMotor.stopMotor();
    }

  public void periodic() {

    temperature = umbrellaMotor.getMotorTemperature();
    velocity = m_umbrellaEncoder.getVelocity();
    current =umbrellaMotor.getOutputCurrent();
    SmartDashboard.putNumber("umbrella/outputCurrent", current);
    SmartDashboard.putNumber("umbrella/temperature", temperature);
    SmartDashboard.putNumber("umbrella/velocity", velocity);
   }
}
