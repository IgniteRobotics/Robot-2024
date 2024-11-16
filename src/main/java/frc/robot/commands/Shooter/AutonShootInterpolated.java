// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.PhotonCameraWrapper;
import frc.robot.subsystems.drive.PhotonCameraWrapper.TargetInfo;
import frc.utils.InterCalculator;
import frc.robot.RobotState;
import frc.robot.Constants.CameraConstants;

import java.util.Optional;
import java.util.function.Supplier;


public class AutonShootInterpolated extends Command {
  private final ShooterSubsystem m_shooter;
  private final Supplier<Double> m_indexPower;
  private final Supplier<Integer> m_targetId;
  private final PhotonCameraWrapper m_cameras;
  private final InterCalculator m_iCalculator = Constants.ShooterConstants.SHOOTER_INTER_CALCULATOR;
  /** Creates a new ShootPiece. */
  public AutonShootInterpolated(ShooterSubsystem shooter, PhotonCameraWrapper cameras, Supplier<Double> indexpower, Supplier<Integer> targetId) {
    m_shooter = shooter;
    m_indexPower = indexpower;
    m_cameras = cameras;
    m_targetId = targetId;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_cameras.unlockTargeting();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<TargetInfo> targeting = m_cameras.seekTarget(m_targetId.get());
    if (targeting.isPresent()){
      var shotParams = m_iCalculator.calculateParameter(targeting.get().getDistance());
      m_shooter.setAngleDegrees(shotParams.vals[0]);
      m_shooter.spinRPM(shotParams.vals[1]);
      if (m_shooter.atRPM() && m_shooter.armAtSetpoint()){
        m_shooter.runIndex(m_indexPower.get());
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopIndexer();
    m_shooter.stopRoller();
    m_cameras.unlockTargeting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //todo use beam break
    return false;
  }
}
