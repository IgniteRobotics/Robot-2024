// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Annotations.Log;;

public class ShooterSubsystem extends SubsystemBase implements Logged {
  private final CANSparkFlex shooterMotorLeader;
  private final CANSparkFlex shooterMotorFollower;

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
    shooterMotorLeader = new CANSparkFlex(Constants.SHOOTER_MOTOR_LEADER, MotorType.kBrushless);
    shooterMotorFollower = new CANSparkFlex(Constants.SHOOTER_MOTOR_FOLLOWER, MotorType.kBrushless);

    shooterMotorLeader.setInverted(false);
    shooterMotorFollower.setInverted(true);

    shooterMotorLeader.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    shooterMotorFollower.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    shooterMotorLeader.setSmartCurrentLimit(40);
    shooterMotorFollower.setSmartCurrentLimit(40);

    shooterMotorFollower.follow(shooterMotorLeader);

    shooterMotorLeader.burnFlash();
    shooterMotorFollower.burnFlash();
  }

  public void spinPower(double power) {
    shooterMotorLeader.set(MathUtil.clamp(power, -1, 1));
  }

  public void stop() {
    shooterMotorLeader.stopMotor();
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
  }
}
