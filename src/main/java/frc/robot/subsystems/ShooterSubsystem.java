// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private final CANSparkFlex shooterMotorLeader;
  private final CANSparkFlex shooterMotorFollower;
  private final SparkPIDController m_pidControllerLeader;
  private final SparkPIDController m_pidControllerFollower;
  
  private double kP, kI, kD, kFF, kMaxOutput, kMinOutput, maxRPM;


  @Log.File
  @Log.NT
  private double tempLeader;

  @Log.File
  @Log.NT
  private double tempFollower;

  @Log.File
  @Log.NT
  private double currentLeader;

  @Log.File
  @Log.NT
  private double currentFollower;

  @Log.File
  @Log.NT
  private double velocityLeader;

  @Log.File
  @Log.NT
  private double velocityFollower;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    shooterMotorLeader = new CANSparkFlex(Constants.SHOOTER_MOTOR_LEADER, MotorType.kBrushless);
    shooterMotorFollower = new CANSparkFlex(Constants.SHOOTER_MOTOR_FOLLOWER, MotorType.kBrushless);

    shooterMotorLeader.setInverted(false);
    shooterMotorFollower.setInverted(true);

    shooterMotorLeader.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    shooterMotorFollower.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    shooterMotorLeader.setSmartCurrentLimit(40);
    shooterMotorFollower.setSmartCurrentLimit(40);

    shooterMotorLeader.setClosedLoopRampRate(1);
    shooterMotorFollower.setClosedLoopRampRate(1);

    shooterMotorLeader.burnFlash();
    shooterMotorFollower.burnFlash();

    m_pidControllerLeader = shooterMotorLeader.getPIDController();
    m_pidControllerFollower = shooterMotorFollower.getPIDController();
  }

  public void spinPower(double power) {
    shooterMotorFollower.set(MathUtil.clamp(power, -1, 1));
    shooterMotorLeader.set(MathUtil.clamp(power, -1, 1));
  }

  public void spinRPM(double rpm) {
    m_pidControllerLeader.setReference(MathUtil.clamp(rpm, -maxRPM, maxRPM), CANSparkFlex.ControlType.kVelocity);
    m_pidControllerFollower.setReference(MathUtil.clamp(rpm, -maxRPM, maxRPM), CANSparkFlex.ControlType.kVelocity);
  }



  public void stop() {
    shooterMotorLeader.stopMotor();
    shooterMotorFollower.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tempLeader = shooterMotorLeader.getMotorTemperature();
    velocityLeader = shooterMotorLeader.getEncoder().getVelocity();
    currentLeader =shooterMotorLeader.getOutputCurrent();

    tempFollower = shooterMotorFollower.getMotorTemperature();
    velocityFollower = shooterMotorFollower.getEncoder().getVelocity();
    currentFollower =shooterMotorFollower.getOutputCurrent();

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidControllerLeader.setP(p); m_pidControllerFollower.setP(p); kP = p; }
    if((i != kI)) { m_pidControllerLeader.setI(i); m_pidControllerFollower.setI(i); kI = i; }
    if((d != kD)) { m_pidControllerLeader.setD(d); m_pidControllerFollower.setD(d); kD = d; }
    if((ff != kFF)) { m_pidControllerLeader.setFF(ff); m_pidControllerFollower.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidControllerLeader.setOutputRange(min, max); 
      m_pidControllerFollower.setOutputRange(min, max);
      kMinOutput = min; kMaxOutput = max; 
    }

}
}
