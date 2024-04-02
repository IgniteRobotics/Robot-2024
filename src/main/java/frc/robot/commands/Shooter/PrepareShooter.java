// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.InterCalculator;
import frc.robot.RobotState;
import frc.robot.subsystems.RumbleSubsystem;

import java.util.function.Supplier;


public class PrepareShooter extends Command {
  private final ShooterSubsystem m_shooter;
  private final RobotState m_robotState;
  private final Supplier<Double> m_maxDistance;
  private final InterCalculator m_iCalculator = Constants.ShooterConstants.SHOOTER_INTER_CALCULATOR;
  /** Creates a new ShootPiece. */
  public PrepareShooter(ShooterSubsystem shooter, Supplier<Double> maxDistance) {
    m_shooter = shooter;
    m_robotState = RobotState.getInstance();
    m_maxDistance = maxDistance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_robotState.hasNote() && m_robotState.getDistancetoSpeaker() < m_maxDistance.get()){
      var shotParams = m_iCalculator.calculateParameter(m_robotState.getDistancetoSpeaker());
      m_shooter.setAngleDegrees(shotParams.vals[0]);
      m_shooter.spinRPM(shotParams.vals[1]);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!m_robotState.hasNote()){
      m_shooter.stopAll();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
