// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import frc.robot.RobotState;

public class Adjust extends Command {
  private Supplier<Double> m_rpm;
  private Supplier<Double> m_position;
  /** Creates a new Adjust. */
  public Adjust(Supplier<Double> rpm, Supplier<Double> position) {
    m_rpm = rpm;
    m_position = position;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotState.getInstance().adjustAutoRPM(m_rpm.get());
    RobotState.getInstance().adjustAutoPosition(m_position.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
