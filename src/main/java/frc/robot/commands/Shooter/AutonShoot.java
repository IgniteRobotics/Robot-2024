// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.Supplier;


public class AutonShoot extends Command {
  private final ShooterSubsystem m_shooter;
  private final Supplier<Double> m_position;
  private final Supplier<Double> m_rpm;
  private final Supplier<Double> m_indexPower;
  /** Creates a new ShootPiece. */
  public AutonShoot(ShooterSubsystem shooter, Supplier<Double> position, Supplier<Double> rpm, Supplier<Double> indexpower) {
    m_shooter = shooter;
    m_position = position;
    m_rpm = rpm;
    m_indexPower = indexpower;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.spinRPM(m_rpm.get());
    m_shooter.setAngleDegrees(m_position.get());
    if (m_shooter.armAtSetpoint() && m_shooter.atRPM()){
      m_shooter.runIndex(m_indexPower.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_shooter.getIndexerBeamBreak();
  }
}
