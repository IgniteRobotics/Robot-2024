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
import frc.robot.Constants.CameraConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class DriveToRing extends Command {
  //if necessary incorporate Shooter to see if beambreak activates
  private final DriveSubsystem m_drive;
  private final PhotonCameraWrapper m_camera;
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;
  Supplier<Double> m_intakePower;
  Supplier<Double> m_intakePosition;
  Supplier<Double> m_indexPower;
  Supplier<Double> m_indexPosition;
  PIDController rotationController;


  private DoublePreference rotKP = new DoublePreference("turnTest/kP", ShooterConstants.AUTO_TARGET_ROT_kP);
  private DoublePreference rotKD = new DoublePreference("turnTest/kD", ShooterConstants.AUTO_TARGET_ROT_kD);
  private DoublePreference rotTolerance = new DoublePreference("turnTest/tolerance", 2);
  
  /** Creates a new DriveToTarget. */
  public DriveToRing(DriveSubsystem drive, PhotonCameraWrapper camera, IntakeSubsystem intake, ShooterSubsystem shooter, Supplier<Double> intakePower, Supplier<Double> intakePosition, Supplier<Double> indexPower, Supplier<Double> indexPosition ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_camera = camera;
    m_shooter = shooter;
    m_intake = intake;
    m_intakePower = intakePower;
    m_intakePosition = intakePosition;
    m_indexPower = indexPower;
    m_indexPosition = indexPosition;
    addRequirements(m_drive, m_shooter, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController =  new PIDController(rotKP.get(), 0, rotKD.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setPosition(m_intakePosition.get());
    m_intake.setSpeed(m_intakePower.get());
    m_shooter.setAngleDegrees(m_indexPosition.get());
    m_shooter.runIndex(m_indexPower.get());
    Optional<TargetInfo> targeting = m_camera.seekNote();
    double rotation = 0.0;
    if (targeting.isPresent()){
      SmartDashboard.putNumber("ring/yawtotarget",targeting.get().getYaw());
      SmartDashboard.putBoolean("ring/turnedtoRing", Math.abs(targeting.get().getYaw()) < CameraConstants.TURNED_TO_NOTE_TOLERANCE);
      rotation = rotationController.calculate(targeting.get().getYaw(),0);
    } else {
      rotation = 0;
      SmartDashboard.putNumber("ring/yawtotarget",0);
    }
    SmartDashboard.putNumber("ring/autoRotnput", rotation);
    m_drive.drive(0.5, 0,
    rotation, 
    false, 
    true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopAll();
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.getIndexerBeamBreak();
  }
}