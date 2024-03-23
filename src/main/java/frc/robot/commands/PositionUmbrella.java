// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UmbrellaSubsystem;

public class PositionUmbrella extends Command {
  /** Creates a new umbrelllacommands. */

  private final UmbrellaSubsystem m_Umbrella;
  private final Supplier<Double> targetPosition;


  public PositionUmbrella(UmbrellaSubsystem Umbrella, Supplier<Double> position) {
    // Use addRequirements() here to declare subsystem dependencies.
  m_Umbrella = Umbrella;
  targetPosition = position;
  addRequirements(m_Umbrella);
  
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
