// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ResetGyro extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private boolean reset = false;
  /** Creates a new ResetGyro. */
  public ResetGyro(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = drive;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    reset = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.zeroHeading();
    reset = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return reset;
  }
}
