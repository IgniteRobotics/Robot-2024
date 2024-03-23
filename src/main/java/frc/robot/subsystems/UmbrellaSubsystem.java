// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.concurrent.PriorityBlockingQueue;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import monologue.Logged;
import monologue.Annotations.Log;





public class UmbrellaSubsystem extends SubsystemBase implements Logged{

  private final CANSparkFlex  umbrellaIndexMotor; 
  private final RelativeEncoder m_umbrellaIndexEncoder;
  private final CANSparkMax umbrellaElavatorMotor;
  private final RelativeEncoder m_umbrellaElavatorEncoder;
  private final SparkPIDController m_umbrellaElavatorPID;

  
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
    umbrellaIndexMotor = new CANSparkFlex(Constants.CANConstants.UMBRELLA_MOTOR_1, MotorType.kBrushless);
    m_umbrellaIndexEncoder = umbrellaIndexMotor.getEncoder();
    umbrellaElavatorMotor = new CANSparkMax(Constants.CANConstants.UMBRELLA_MOTOR_2, MotorType.kBrushless);
    m_umbrellaElavatorEncoder = umbrellaElavatorMotor.getEncoder();
    m_umbrellaElavatorPID = umbrellaElavatorMotor.getPIDController();
    

    //configuring motor
    umbrellaIndexMotor.setInverted(false);//TODO: fix when certain
    umbrellaIndexMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    umbrellaIndexMotor.setSmartCurrentLimit(40); //maybe?
    umbrellaIndexMotor.burnFlash();

    umbrellaElavatorMotor.setInverted(false);
    umbrellaElavatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    umbrellaElavatorMotor.setSmartCurrentLimit(40);
    umbrellaElavatorMotor.burnFlash();

    configureubrellaElavatorMotor(umbrellaElavatorMotor, m_umbrellaElavatorEncoder);


  }

  public void setelavatorSpeed (double speed){
    umbrellaElavatorMotor.set(MathUtil.clamp(speed, -1, 1));
  }


  public void setumbrellaSpeed (double speed)
    {
      umbrellaIndexMotor.set(MathUtil.clamp(speed, -1, 1));
    }
 
  public void stopUmbrella(){
      umbrellaIndexMotor.stopMotor();
    }
  public void stopElavator(){
      umbrellaElavatorMotor.stopMotor();
  }
   private void configureubrellaElavatorMotor(CANSparkMax motor, RelativeEncoder encoder) {
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(20);
    motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    motor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.setSoftLimit(SoftLimitDirection.kForward,10 );
    encoder.setPositionConversionFactor(10); // need the actual conversion rate
    //Assume intake is up on power-on.
    encoder.setPosition(0);
    motor.burnFlash();
  }

  public void periodic() {

    temperature = umbrellaIndexMotor.getMotorTemperature();
    velocity = m_umbrellaIndexEncoder.getVelocity();
    current =umbrellaIndexMotor.getOutputCurrent();
    SmartDashboard.putNumber("umbrella/outputCurrent", current);
    SmartDashboard.putNumber("umbrella/temperature", temperature);
    SmartDashboard.putNumber("umbrella/velocity", velocity);
   }
}
