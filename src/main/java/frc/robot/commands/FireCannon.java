// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.comm.preferences.DoublePreference;
import frc.robot.subsystems.Cannon;
import java.util.function.Supplier;

public class FireCannon extends Command {
  /** Creates a new FireCannon. */
  Cannon m_Cannon;
  Supplier<Double> m_Position;
  Supplier<Double> m_rpm;

  public FireCannon(Cannon cannon, Supplier<Double> position, Supplier<Double> rpm) {
    m_Cannon = cannon;
    m_Position = position;
    m_rpm = rpm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Cannon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Cannon.spinRPM(m_rpm.get());
    m_Cannon.setAngleDegrees(m_Position.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Cannon.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
