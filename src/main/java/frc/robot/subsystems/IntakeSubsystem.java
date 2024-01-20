// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;


import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


/*
public class IntakeSubsystem extends SubsystemBase {
  // Creates a new IntakeSubsystem. 
  private CANSparkMax intakeMotor;

  private DigitalInput cubeBeamBreak = new DigitalInput(1);
  private DigitalInput coneBeamBreak = new DigitalInput(0);
  public IntakeSubsystem() {
    this.intakeMotor = new CANSparkMax(Constants.CANConstants.INTAKE, MotorType.kBrushless);

    this.intakeMotor.setInverted(false);
    this.intakeMotor.setIdleMode(IdleMode.kBrake);
    this.intakeMotor.setSmartCurrentLimit(25);
  }
 @Override
  public void periodic() {
    SmartDashboard.putBoolean("intake/cubeBeamBreak", cubeBeamBreak.get());
    SmartDashboard.putBoolean("intake/coneBeamBreak", coneBeamBreak.get());
    SmartDashboard.putNumber("intake/outputCurrent", intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("intake/temperature", intakeMotor.getMotorTemperature());
  }
} */

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  ///private final CANSparkMax additionalIntakeMotor;
  int INTAKE_MOTOR_1 = 0;
  
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax (INTAKE_MOTOR_1, MotorType.kBrushless);
    // additionalIntakeMotor = new CANSparkMax(Constants.CANConstants.INTAKE_MOTOR_2, MotorType.kBrushless);

        // Configuring the main intake motor
        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(25); // Don't modify or remove

      
    }

    public void setSpeed (double speed)
    {
      intakeMotor.set(MathUtil.clamp(speed, -1, 1));
    }
    
    public void stop(){
      intakeMotor.stopMotor();
      //intakeMotor.set(0);
    }

  public void periodic() {
    AtomicBoolean cubeBeamBreak;
	  SmartDashboard.putBoolean("intake/cubeBeamBreak", cubeBeamBreak.get());
    AtomicBoolean coneBeamBreak;
    SmartDashboard.putBoolean("intake/coneBeamBreak", coneBeamBreak.get());
    SmartDashboard.putNumber("intake/outputCurrent", intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("intake/temperature", intakeMotor.getMotorTemperature());
  }

    
  

    // Additional methods for the subsystem can be added here
}

  public void runMotor(double speed) {
        intakeMotor.set(speed);
    }

    

    

  public CommandBase runIntakeConeCommand() {
    // to intake a cone, run the intake with negative effort
    return runIntakeCommand(RobotPreferences.coneBeamBreakDelay, () -> -RobotPreferences.intakeEffort.get());
  }


