// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;
import monologue.Logged;
import monologue.Annotations.Log;
import edu.wpi.first.wpilibj.Preferences;




public class IntakeSubsystem extends SubsystemBase implements Logged {
  
  private final CANSparkMax intakeMotor, positionMotor;
  private final SparkPIDController positionController;
  private final double POSITION_TOLERANCE = 50;

  public static class positionConstants {
    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;
  }
  @Log.File
  @Log.NT
  private double intakecurrent;
  @Log.File
  @Log.NT
  private double intakevelocity;
  @Log.File
  @Log.NT
  private double intaketemperature;
  @Log.File
  @Log.NT
  private double postioncurrent;
  @Log.File
  @Log.NT
  private double positionvelocity;
  @Log.File
  @Log.NT
  private double positiontemperature;
  @Log.File
  @Log.NT
  private double positionTarget;
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax (Constants.CANConstants.INTAKE_ROLLER_MOTOR, MotorType.kBrushless);
    // additionalIntakeMotor = new CANSparkMax(Constants.CANConstants.INTAKE_MOTOR_2, MotorType.kBrushless);

        // Configuring the main intake motor
        intakeMotor.setInverted(true);
        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(25); // Don't modify or remove
        intakeMotor.burnFlash();

      // postion control motor neo 550
        positionMotor = new CANSparkMax (Constants.CANConstants.INTAKE_POSITION_MOTOR, MotorType.kBrushless);
        positionMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        positionMotor.setSmartCurrentLimit(25);
        positionMotor.burnFlash();

     
        // the variable numbers for the softlimit are from 2023 they need to be fixed
        this.positionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        this.positionMotor.setSoftLimit(SoftLimitDirection.kReverse, 150);
        this.positionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        this.positionMotor.setSoftLimit(SoftLimitDirection.kForward, 1125);
    
        this.positionController = positionMotor.getPIDController();
       
    
        configureSparkMax(positionMotor);
      
    
        configurepositionController(positionController);
        
    
        Preferences.initDouble("arm/arm/kP", 0);
        Preferences.initDouble("arm/arm/kI", 0);
        Preferences.initDouble("arm/arm/kD", 0);
        




        







    }

    private void configurepositionController(SparkPIDController controller) {
          controller.setP(positionConstants.kP);
          controller.setI(positionConstants.kI);
          controller.setD(positionConstants.kD);
          controller.setOutputRange(-1, 1);
    }

    private void configureSparkMax(CANSparkMax motor) {
    positionMotor.setInverted(false);
    positionMotor.setIdleMode(IdleMode.kBrake);

    positionMotor.setSmartCurrentLimit(20);
  }
    public void setSpeed (double speed)
    {
      intakeMotor.set(MathUtil.clamp(speed, -1, 1));    
    }

    public void setPosition (double position)
    {
      positionController.setReference(position, ControlType.kSmartMotion);
        positionTarget = position;
    }
    public boolean wristAtSetpoint() {
        if(Robot.isSimulation()) return true;
         return getIntakePosition() >= positionTarget - POSITION_TOLERANCE
             && getIntakePosition() <= positionTarget + POSITION_TOLERANCE;
  }
  public double getIntakePosition() {
    return positionMotor.getEncoder().getPosition();
  }
    public void stop(){
      intakeMotor.stopMotor();
    }

  public void periodic() {

    intaketemperature = intakeMotor.getMotorTemperature();
    intakevelocity = intakeMotor.getEncoder().getVelocity();
    intakecurrent = intakeMotor.getOutputCurrent();
    positiontemperature = positionMotor.getMotorTemperature();
    positionvelocity = positionMotor.getEncoder().getVelocity();
    postioncurrent = positionMotor.getOutputCurrent();

   }



    
  

    // Additional methods for the subsystem can be added here


  public void runMotor(double speed) {
        intakeMotor.set(speed);
    }

  
    public Command dogetDefaultCommand(){
      return runEnd(() -> {
        setSpeed(0);
        setPosition(0);
      }, this::stop);
    }


}
