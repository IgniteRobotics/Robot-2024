// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.comm.preferences.DoublePreference;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PhotonCameraWrapper;
import frc.robot.subsystems.drive.PhotonCameraWrapper.TargetInfo;
import frc.utils.InterCalculator;

public class TurnAndShoot extends Command {
  private final DriveSubsystem m_drive;
  private final ShooterSubsystem m_shooter;
  private final PhotonCameraWrapper m_cameras;
  private final Supplier<Integer> m_targetId;
  private final Supplier<Double> m_indexPower;

  PIDController rotationController;

  private DoublePreference rotKP = new DoublePreference("turnTest/kP", ShooterConstants.AUTO_TARGET_ROT_kP);
  private DoublePreference rotKD = new DoublePreference("turnTest/kD", ShooterConstants.AUTO_TARGET_ROT_kD);
  private DoublePreference rotTolerance = new DoublePreference("turnTest/tolerance", 2);

  private final InterCalculator m_iCalculator = Constants.ShooterConstants.SHOOTER_INTER_CALCULATOR;


  /** Creates a new DriveToTarget. */
  public TurnAndShoot(DriveSubsystem drive, ShooterSubsystem shooter, PhotonCameraWrapper cameras, Supplier<Integer> targetId,  Supplier<Double> indexPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_shooter = shooter;
    m_cameras = cameras;
    m_targetId = targetId;
    m_indexPower = indexPower;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController =  new PIDController(rotKP.get(), 0, rotKD.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("targetid",m_targetId.get());
    Optional<TargetInfo> targeting = m_cameras.seekTarget(m_targetId.get());
    double rotation = 0.0;
    if (targeting.isPresent()){
      SmartDashboard.putNumber("yawtotarget",targeting.get().getYaw());
      rotation = rotationController.calculate(targeting.get().getYaw(),0);
      var shotParams = m_iCalculator.calculateParameter(targeting.get().getDistance());
      m_shooter.setAngleDegrees(shotParams.vals[0]);
      m_shooter.spinRPM(shotParams.vals[1]);
      SmartDashboard.putNumber("autoRotnput", rotation);
      if (m_shooter.atRPM() && m_shooter.armAtSetpoint() && rotation < rotTolerance.get()) {
        m_shooter.runIndex(m_indexPower.get());
      }
    }
    m_drive.drive(0,
                  0,
                  rotation, 
                  true, 
                  true);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_shooter.getIndexerBeamBreak();
  }
}
