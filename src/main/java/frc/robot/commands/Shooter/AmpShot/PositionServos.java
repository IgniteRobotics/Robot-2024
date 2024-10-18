// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter.AmpShot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.Supplier;

public class PositionServos extends Command {
  ShooterSubsystem m_shooter;
  Supplier<Double> m_newPosition;
  Supplier<Double> m_defaultPosition; 
  public PositionServos(ShooterSubsystem shooter, Supplier<Double> newPosition, Supplier<Double> defaultPosition) {
    m_shooter = shooter;
    m_newPosition = newPosition; 
    m_defaultPosition = defaultPosition; 
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setServoPosition(m_newPosition.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setServoPosition(m_defaultPosition.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
