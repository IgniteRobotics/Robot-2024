// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class StowIntake extends Command {
  private IntakeSubsystem m_intake;
  /** Creates a new StowIntake. */
  public StowIntake(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intake = intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setPosition(1);
    m_intake.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopPositionMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean wta =m_intake.getIntakePosition() < 2 ;
    return wta;

  }
}
