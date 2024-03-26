// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.Arrays;
import java.util.function.Supplier;
import monologue.Logged;
import monologue.Annotations.Log;
import edu.wpi.first.math.MathUtil;


public class StowShooter extends Command {
  private final ShooterSubsystem m_shooter;
  private final Supplier<Double> m_targetAngle;



  private final double m_waitTime = 0.10;
  private final Timer m_Timer = new Timer();
  private boolean m_inWait = false;
  private boolean m_hasReset = false;

  /** Creates a new RunShooterRPM. */
  public StowShooter(ShooterSubsystem shooter, Supplier<Double> angle) {
    m_shooter = shooter;
    m_targetAngle = angle;
    m_inWait = false;
    m_hasReset = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  public StowShooter(ShooterSubsystem shooter, double angle)
  {
    m_shooter = shooter;
    m_targetAngle = () -> angle;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hasReset = false;
    m_inWait = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_hasReset){    
      if (!m_shooter.armAtSetpoint()){
        m_inWait = false;
        m_shooter.setAngleDegrees(MathUtil.clamp(m_targetAngle.get(), 0, 120));
      } else { // arm is at setpoint
        if (!m_inWait) {  //start waiting if you need to
          m_inWait = true;
          m_Timer.reset();
          m_Timer.start();
        } else if (m_Timer.hasElapsed(m_waitTime)) {
          m_shooter.reZeroEncoder();
          m_hasReset = true;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopPositioner();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
