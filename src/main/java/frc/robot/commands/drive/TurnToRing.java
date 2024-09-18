// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.comm.preferences.DoublePreference;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PhotonCameraWrapper;
import frc.robot.subsystems.drive.PhotonCameraWrapper.TargetInfo;

public class TurnToRing extends Command {
  private final DriveSubsystem m_drive;
  private final PhotonCameraWrapper m_camera;
  private final Supplier<Integer> m_ringId;
  PIDController rotationController;

  private DoublePreference rotKP = new DoublePreference("turnTest/kP", ShooterConstants.AUTO_TARGET_ROT_kP);
  private DoublePreference rotKD = new DoublePreference("turnTest/kD", ShooterConstants.AUTO_TARGET_ROT_kD);
  private DoublePreference rotTolerance = new DoublePreference("turnTest/tolerance", 2);

  /** Creates a new DriveToTarget. */
  public TurnToRing(DriveSubsystem drive, PhotonCameraWrapper camera, Supplier<Integer> ringId) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_camera = camera;
    m_ringId = ringId;
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
    Optional<TargetInfo> targeting = m_camera.seekNote();
    double rotation = 0.0;
    if (targeting.isPresent()){
      SmartDashboard.putNumber("ring/yawtotarget",targeting.get().getYaw());
      rotation = rotationController.calculate(targeting.get().getYaw(),0);
    } else {
      rotation = 0;
      SmartDashboard.putNumber("ring/yawtotarget",0);
    }
    SmartDashboard.putNumber("ring/autoRotnput", rotation);
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
