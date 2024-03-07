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

import java.util.function.Supplier;


public class ShootInterpolated extends Command {
  private final ShooterSubsystem m_shooter;
  private final Supplier<Double> m_indexPower;
  private final Supplier<Boolean> m_ready; 
  private final RobotState m_robotState;
  private final InterCalculator m_iCalculator = Constants.ShooterConstants.SHOOTER_INTER_CALCULATOR;
  /** Creates a new ShootPiece. */
  public ShootInterpolated(ShooterSubsystem shooter, Supplier<Double> indexpower, Supplier<Boolean> ready) {
    m_shooter = shooter;
    m_indexPower = indexpower;
    m_ready = ready;
    m_robotState = RobotState.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var shotParams = m_iCalculator.calculateParameter(m_robotState.getDistancetoSpeaker());
    m_shooter.setAngleDegrees(shotParams.vals[0]);
    m_shooter.spinRPM(shotParams.vals[1]);
    if (m_ready.get() && m_shooter.atRPM() && m_shooter.armAtSetpoint()){
      m_shooter.runIndex(m_indexPower.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopIndexer();
    m_shooter.stopRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //todo use beam break
    return false;
  }
}
