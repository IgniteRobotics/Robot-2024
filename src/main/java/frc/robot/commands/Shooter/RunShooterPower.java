// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


public class RunShooterPower extends Command {
  private final ShooterSubsystem m_shooter;
  private final Supplier<Double> m_power;
  /** Creates a new RunShooterPower. */
  public RunShooterPower(ShooterSubsystem shooter, Supplier<Double> power) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_power = power;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.spinPower(m_power.get());
    m_shooter.runIndex(.25);
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopRoller();
    m_shooter.stopIndexer();
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
