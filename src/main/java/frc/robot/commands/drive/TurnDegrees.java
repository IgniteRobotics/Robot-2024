// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.comm.preferences.DoublePreference;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.drive.PhotonCameraWrapper;

public class TurnDegrees extends Command {
  private final DriveSubsystem m_drive;
  private double m_turnDegrees = 0;
  private double m_initialHeading = 0;
  private double m_targetHeading = 0;
  PIDController rotationController;

  private DoublePreference rotKP = new DoublePreference("turnTest/kP", ShooterConstants.AUTO_TARGET_ROT_kP);
  private DoublePreference rotKD = new DoublePreference("turnTest/kD", ShooterConstants.AUTO_TARGET_ROT_kD);
  private DoublePreference rotTolerance = new DoublePreference("turnTest/tolerance", 2);

  /** Creates a new TurnDegrees. */
  public TurnDegrees(DriveSubsystem drive, double turnDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_turnDegrees = turnDegrees;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialHeading = m_drive.getAngle();
    m_targetHeading = m_initialHeading + m_turnDegrees;
    rotationController =  new PIDController(rotKP.get(), 0, rotKD.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotation = m_targetHeading - m_drive.getAngle();
    rotation = -rotationController.calculate(rotation,0);
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
    return Math.abs(m_drive.getAngle() - m_targetHeading) < rotTolerance.get();
  }
}
