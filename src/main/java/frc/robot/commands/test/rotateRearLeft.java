// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class rotateRearLeft extends Command {
  DriveSubsystem m_testDrive;
  /** Creates a new rotateRearLeft. */
  public rotateRearLeft(DriveSubsystem testDrive) {
    m_testDrive = testDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_testDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_testDrive.rotateRL360();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // return m_testDrive.getRLPos() == 360;
   return false;
  }
}
