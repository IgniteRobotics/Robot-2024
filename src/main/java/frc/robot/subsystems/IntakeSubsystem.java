// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.comm.preferences.DoublePreference;
import frc.robot.subsystems.IntakeSubsystem;
import monologue.Logged;
import monologue.Annotations.Log;
import edu.wpi.first.wpilibj.Preferences;




public class IntakeSubsystem extends SubsystemBase implements Logged {
  
  private final CANSparkMax intakeMotor, positionMotor;
  private final SparkPIDController positionController;

  private DoublePreference positionkP = new DoublePreference("intake/positionkP", Constants.IntakeConstants.POSITION_kP);
  private DoublePreference positionkI = new DoublePreference("intake/positionkI", Constants.IntakeConstants.POSITION_kI);
  private DoublePreference positionkD = new DoublePreference("intake/positionkD", Constants.IntakeConstants.POSITION_kD);

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
    
    // postion control motor neo 550
    positionMotor = new CANSparkMax (Constants.CANConstants.INTAKE_POSITION_MOTOR, MotorType.kBrushless);

    this.positionController = positionMotor.getPIDController();

    configurePosistionMotor(positionMotor);    
    configurePositionController(positionController);
    
    configureIntakeMotor(intakeMotor);
  

  }

  private void configurePositionController(SparkPIDController controller) {
    controller.setP(positionkP.get());
    controller.setI(positionkI.get());
    controller.setD(positionkD.get());
    controller.setOutputRange(-1, 1);
  }

  private void configurePosistionMotor(CANSparkMax motor) {
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(20);
    // the variable numbers for the softlimit are from 2023 they need to be fixed
    motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    motor.setSoftLimit(SoftLimitDirection.kReverse, 150);
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.setSoftLimit(SoftLimitDirection.kForward, 1125);
    motor.burnFlash();
  }


  private void configureIntakeMotor(CANSparkMax motor) {
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(25);
    motor.burnFlash();

  }

  

  public void setSpeed (double speed){
      intakeMotor.set(MathUtil.clamp(speed, -1, 1));    
  }

  public void setPosition (double position){
    positionController.setReference(position, ControlType.kSmartMotion);
    positionTarget = position;
  }
  
  public boolean atSetpoint() {
        if(Robot.isSimulation()) return true;
         return getIntakePosition() >= positionTarget - Constants.IntakeConstants.POSITION_TOLERANCE
             && getIntakePosition() <= positionTarget + Constants.IntakeConstants.POSITION_TOLERANCE;
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


  
    public Command dogetDefaultCommand(){
      return runEnd(() -> {
        setSpeed(0);
        setPosition(0);
      }, this::stop);
    }


}
