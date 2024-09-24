// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.Supplier;

public class RunIndexFrom extends Command {
  private final ShooterSubsystem m_shooter;
  private final Supplier<Double> m_indexPower;



  /** Creates a new IntakePiece. */
  public RunIndexFrom(ShooterSubsystem shooter, Supplier<Double> indexPower) {
    
    m_shooter = shooter;
    m_indexPower = indexPower;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.runIndex(m_indexPower.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopIndexer();
    m_shooter.setReady(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO use beam break
    return !m_shooter.getIndexerBeamBreak();
  }
}