// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.Supplier;
import monologue.Logged;
import monologue.Annotations.Log;
import edu.wpi.first.math.MathUtil;


public class PositionShooter extends Command {
  private final ShooterSubsystem m_shooter;
  private final Supplier<Double> m_targetAngle;

  /** Creates a new RunShooterRPM. */
  public PositionShooter(ShooterSubsystem shooter, Supplier<Double> angle) {
    m_shooter = shooter;
    m_targetAngle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  public PositionShooter(ShooterSubsystem shooter, double angle)
  {
    m_shooter = shooter;
    m_targetAngle = () -> angle;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setAngleDegrees(MathUtil.clamp(m_targetAngle.get(), 0, 120));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopPositioner();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.atSetpoint();
  }
}
