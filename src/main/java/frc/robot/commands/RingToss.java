// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RingToss extends Command {
  private final IntakeSubsystem m_intake;
  private final ShooterSubsystem m_shooter;
  private final Supplier<Double> m_intakePower;
  private final Supplier<Double> m_intakePosition;
  private final Supplier<Double> m_indexPower;
  private final Supplier<Double> m_indexPosition;
  private final Supplier<Double> m_shooterRPM;
  /** Creates a new RingToss. */
  public RingToss(IntakeSubsystem intake, ShooterSubsystem shooter, Supplier<Double> intakePower, Supplier<Double> intakePosition, Supplier<Double> indexPower, Supplier<Double> indexPosition, Supplier<Double> shooterRPM ) {
    
    m_intake = intake; 
    m_shooter = shooter;
    m_shooterRPM = shooterRPM;
    m_intakePower = intakePower;
    m_intakePosition = intakePosition;
    m_indexPower = indexPower;
    m_indexPosition = indexPosition;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_shooter.getIndexerBeamBreak()){
      m_intake.setPosition(m_intakePosition.get());
      m_intake.setSpeed(m_intakePower.get());
    } else {
      m_intake.stop();
      m_intake.setPosition(1);
    }
    
    m_shooter.setAngleDegrees(m_indexPosition.get());
    m_shooter.runIndex(m_indexPower.get());
    m_shooter.spinRPM(m_shooterRPM.get());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
