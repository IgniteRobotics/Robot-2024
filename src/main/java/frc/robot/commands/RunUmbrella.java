// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UmbrellaSubsystem;

public class RunUmbrella extends Command {
  private final UmbrellaSubsystem m_umbrella;
  private final Supplier<Double> m_powSupplier;
  /** Creates a new RunUmbrella. */
  public RunUmbrella(UmbrellaSubsystem subsystem, Supplier<Double> powSupplier) {
    m_umbrella = subsystem;
    m_powSupplier = powSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_umbrella.setSpeed(m_powSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_umbrella.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
