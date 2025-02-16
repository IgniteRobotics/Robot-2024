// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.Supplier;

public class RunShooterRPM extends Command {
  private final ShooterSubsystem m_shooter;
  private final Supplier<Double> m_RPM;

  /** Creates a new RunShooterRPM. */
  public RunShooterRPM(ShooterSubsystem shooter, Supplier<Double> RPM) {
    m_shooter = shooter;
    m_RPM = RPM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.spinRPM(m_RPM.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
