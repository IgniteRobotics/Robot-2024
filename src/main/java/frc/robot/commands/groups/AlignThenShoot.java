// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.Shooter.ShootInterpolated;
import frc.robot.commands.drive.DriveToTarget;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.PhotonCameraWrapper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignThenShoot extends ParallelDeadlineGroup {
  /** Creates a new AlignThenShoot. */
  public AlignThenShoot(ShooterSubsystem shooter, DriveSubsystem drive, PhotonCameraWrapper cameras,
                        Supplier<Double> driveX,
                        Supplier<Double> driveY,
                        Supplier<Double> indexPower,
                        Supplier<Boolean> trigger,
                        Supplier<Integer> targetID
                        ) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new ShootInterpolated(shooter, indexPower, trigger)); 
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveToTarget(drive, cameras, targetID, driveX, driveY));
  }
}
