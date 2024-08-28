// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Measure;




public class IntakeSubsystem extends SubsystemBase implements Logged {
  
  private final CANSparkMax intakeMotor, positionMotor;
  private final SparkPIDController positionController;
  private final RelativeEncoder positionEncoder;
  private final RelativeEncoder intakeEncoder;

  // intakepostion constants 
  private final static float  positionUp = 1;
  private final static float  positionDown = 110;
  private final static double positionInputmin = -1;
  private final static double positionInputmax = 1;

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

  @Log.File
  @Log.NT
  private double kP;

  @Log.File
  @Log.NT
  private double kD;

  
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax (Constants.CANConstants.INTAKE_ROLLER_MOTOR, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();


    // postion control motor neo 550
    positionMotor = new CANSparkMax (Constants.CANConstants.INTAKE_POSITION_MOTOR, MotorType.kBrushless);

    this.positionController = positionMotor.getPIDController();
    this.positionEncoder = positionMotor.getEncoder();

    configurePosistionMotor(positionMotor, positionEncoder);    
    configurePositionController(positionController, positionEncoder);
    
    configureIntakeMotor(intakeMotor);
  

  }

  private void configurePositionController(SparkPIDController controller, RelativeEncoder encoder) {
    controller.setP(positionkP.get());
    controller.setI(positionkI.get());
    controller.setD(positionkD.get());
    controller.setFeedbackDevice(encoder);
    controller.setOutputRange(positionInputmin, positionInputmax);
    this.positionMotor.burnFlash();
  }

  private void configurePosistionMotor(CANSparkMax motor, RelativeEncoder encoder) {
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(20);
    motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    motor.setSoftLimit(SoftLimitDirection.kReverse, positionUp);
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.setSoftLimit(SoftLimitDirection.kForward, positionDown);
    encoder.setPositionConversionFactor(10);
    //Assume intake is up on power-on.
    encoder.setPosition(positionUp);
    motor.burnFlash();
  }


  private void configureIntakeMotor(CANSparkMax motor) {
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(40);
    motor.burnFlash();
    motor.setOpenLoopRampRate(0);

  }

  

  public void setSpeed (double speed){
      intakeMotor.set(MathUtil.clamp(speed, -1, 1));    
  }

  public void setPosition (double position){
    positionController.setReference(position, ControlType.kPosition);
    positionTarget = position;
  }
  
 // public boolean atSetpoint() {
  //      if(Robot.isSimulation()) return true;
   //      return getIntakePosition() >= positionTarget - Constants.IntakeConstants.POSITION_TOLERANCE
    //         && getIntakePosition() <= positionTarget + Constants.IntakeConstants.POSITION_TOLERANCE;
 // }

 @Log.File
 @Log.NT
  public double getIntakePosition() {
    return this.positionEncoder.getPosition();
  }

  @Log.File
  @Log.NT
   public double getPositionCF() {
     return this.positionEncoder.getPositionConversionFactor();
   }
  
  public void stop(){
    intakeMotor.stopMotor();
  }
  public void stopPositionMotor(){
    positionMotor.stopMotor();
  }

  public void periodic() {

    intaketemperature = intakeMotor.getMotorTemperature();
    intakevelocity = this.intakeEncoder.getVelocity();
    intakecurrent = intakeMotor.getOutputCurrent();
    positiontemperature = positionMotor.getMotorTemperature();
    positionvelocity = this.positionEncoder.getVelocity();
    postioncurrent = positionMotor.getOutputCurrent();
    this.kP = this.positionController.getP();
    this.kD = this.positionController.getD();
    //TODO remove once comfortable with PID
    this.positionController.setP(positionkP.get());
    this.positionController.setI(positionkI.get());
    this.positionController.setD(positionkD.get());
    this.positionMotor.burnFlash();

   }



    public Command dogetDefaultCommand(){
      return runOnce(() -> {
        setSpeed(0);
        setPosition(positionUp);
      });
    }
    
    public Command intakeCommand(){
      return runEnd(()  ->{
        setSpeed(1);
        setPosition(positionDown);
       }, this::stop);
    }
    public Command extakeCommand(){
      return runEnd(()   ->{
        setSpeed(-.85);
        setPosition(positionDown);
      }, this::stop);
    }
     public Command spinRollers(){
        return runEnd(() ->{
          setSpeed(.85);
        }, this::stop);

      }

    public Command intakeMotorTestBuilder(double staticTimeout, double dynamicTimeout){
    return
      new InstantCommand(() -> this.stop())
      .andThen(new InstantCommand(() -> this.setSpeed(0.5))).withTimeout(staticTimeout)
      .andThen(new InstantCommand(() -> this.setSpeed(-0.5)).withTimeout(staticTimeout))
      .andThen(new InstantCommand(() -> this.stop()))
      .andThen(new WaitCommand(2))
      .andThen(new InstantCommand(() -> this.setSpeed(1)).withTimeout(dynamicTimeout))
      .andThen(new InstantCommand(() -> this.setSpeed(-1)).withTimeout(dynamicTimeout))
      .andThen(new InstantCommand(() -> this.stop()));
  }
/* 
  public Command intakePositionTestBuilder(double staticTimeout, double dynamicTimeout){
    return
      new InstantCommand(() -> this.setSpeed(0))
      .andThen(new InstantCommand(() -> this.setSpeed(0.5))).withTimeout(staticTimeout)
      .andThen(new InstantCommand(() -> this.setSpeed(-0.5)).withTimeout(staticTimeout))
      .andThen(new InstantCommand(() -> this.setSpeed(0)))
      .andThen(new WaitCommand(2))
      .andThen(new InstantCommand(() -> this.setSpeed(1)).withTimeout(dynamicTimeout))
      .andThen(new InstantCommand(() -> this.setSpeed(-1)).withTimeout(dynamicTimeout))
      .andThen(new InstantCommand(() -> this.setSpeed(0)));
  }

*/


     
}
