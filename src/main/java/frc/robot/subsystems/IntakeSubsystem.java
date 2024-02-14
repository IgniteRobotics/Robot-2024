// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import monologue.Logged;
import monologue.Annotations.Log;






public class IntakeSubsystem extends SubsystemBase implements Logged {
  private final CANSparkMax intakeMotor, positionMotor;

  @Log.File
  @Log.NT
  private double current;
  @Log.File
  @Log.NT
  private double velocity;
  @Log.File
  @Log.NT
  private double temperature;

  
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax (Constants.INTAKE_ROLLER_MOTOR_1, MotorType.kBrushless);
    // additionalIntakeMotor = new CANSparkMax(Constants.CANConstants.INTAKE_MOTOR_2, MotorType.kBrushless);

        // Configuring the main intake motor
        intakeMotor.setInverted(true);
        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(25); // Don't modify or remove
        intakeMotor.burnFlash();

      // postion control motor neo 550
    positionMotor = new CANSparkMax (Constants.INTAKE_POSITION_MOTOR_1, MotorType.kBrushless);
    }

    public void setSpeed (double speed)
    {
      intakeMotor.set(MathUtil.clamp(speed, -1, 1));
    }
    
    public void stop(){
      intakeMotor.stopMotor();
    }

  public void periodic() {

    temperature = intakeMotor.getMotorTemperature();
    velocity = intakeMotor.getEncoder().getVelocity();
    current = intakeMotor.getOutputCurrent();
    SmartDashboard.putNumber("intake/outputCurrent", current);
    SmartDashboard.putNumber("intake/temperature", temperature);
    SmartDashboard.putNumber("intake/velocity", velocity);
   }

    
  

    // Additional methods for the subsystem can be added here


  public void runMotor(double speed) {
        intakeMotor.set(speed);
    }

}
