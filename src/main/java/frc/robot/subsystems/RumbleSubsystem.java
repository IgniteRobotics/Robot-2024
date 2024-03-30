// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class RumbleSubsystem extends SubsystemBase {
  /** Creates a new RumbleSubsystem. */
  XboxController m_driveController;
  XboxController m_manipController;
  double m_rumbleTime;
  double m_driverRumblePower;
  double m_manipRumblePower;
  Timer rumbleTimer = new Timer();
  double noteLoops;
  boolean isContinuous;

  public RumbleSubsystem(XboxController driverController, XboxController manipController, double driverRumblePower, double manipRumblePower) {
    m_driveController = driverController;
    m_manipController = manipController;
    m_driverRumblePower = driverRumblePower;
    m_manipRumblePower = manipRumblePower;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(rumbleTimer.hasElapsed(m_rumbleTime)){
      m_driveController.setRumble(RumbleType.kBothRumble, 0);
      m_manipController.setRumble(RumbleType.kBothRumble, 0);
    }
    
    if(noteLoops <= 2 && !isContinuous) {
      noteLoadedRumbleLoop(0.25);
      noteLoops++;
    } else {
      m_driveController.setRumble(RumbleType.kBothRumble, 0);
      m_manipController.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  public void teleopRumble(double rumbleTime) {
    rumbleTimer.reset();
    rumbleTimer.start();
    m_rumbleTime = rumbleTime;
    isContinuous = true;
    m_driveController.setRumble(RumbleType.kBothRumble, 1);
    m_manipController.setRumble(RumbleType.kBothRumble, 0);
  }

  public void noteLoadedRumble(double rumbleTime) {
    rumbleTimer.reset();
    rumbleTimer.start();
    m_rumbleTime = rumbleTime;
    isContinuous = false;
    m_driveController.setRumble(RumbleType.kBothRumble, 1);
    m_manipController.setRumble(RumbleType.kBothRumble, 1);
  }

  public void noteLoadedRumbleLoop(double rumbleTime) {
    rumbleTimer.reset();
    rumbleTimer.start();
    m_rumbleTime = rumbleTime;
    isContinuous = true;
    m_driveController.setRumble(RumbleType.kBothRumble, 1);
    m_manipController.setRumble(RumbleType.kBothRumble, 1);
  }

  public void shooterAtRPMAndAngle(double rumbleTime) {
    rumbleTimer.reset();
    rumbleTimer.start();
    m_rumbleTime = rumbleTime;
    isContinuous = true;
    m_driveController.setRumble(RumbleType.kBothRumble, 0.5);
    m_manipController.setRumble(RumbleType.kBothRumble, 0.5);
  }
}
