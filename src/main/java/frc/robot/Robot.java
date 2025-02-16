// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ShooterConstants;
import monologue.Monologue;
import monologue.Annotations.Log;
import monologue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.util.datalog.StringLogEntry;

/**f
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements Logged {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Log.NT
  @Log.File
  private CommandScheduler m_Scheduler;

  private boolean hasAlliance = false;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // DO THIS FIRST
    Pathfinding.setPathfinder(new LocalADStar());

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_Scheduler = CommandScheduler.getInstance();

    DataLogManager.start();

    //starting URCL in sim blows up.
    if (isReal()) {
      URCL.start();
    } else {
      //Shush telling me that I don't have a controller in the sim.
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    //do this LAST!!
    boolean fileOnly = false;
    boolean lazyLogging = false;
    Monologue.setupMonologue(this, "/Robot", fileOnly, lazyLogging);
    DriverStation.startDataLog(DataLogManager.getLog()); // same log used by monologue
    StringLogEntry MetaData = new StringLogEntry(DataLogManager.getLog(), "MetaData");
    MetaData.append("Project Name: " + BuildConstants.MAVEN_NAME);
    MetaData.append("Build Date: " + BuildConstants.BUILD_DATE);
    MetaData.append("Commit Hash: " + BuildConstants.GIT_SHA);
    MetaData.append("Git Date: " + BuildConstants.GIT_DATE);
    MetaData.append("Git Branch: " + BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        MetaData.append("GitDirty: " + "All changes commited");
        break;
      case 1:
        MetaData.append("GitDirty: " + "Uncomitted changes");
        break;
      default:
        MetaData.append("GitDirty: " + "Unknown");
        break;
    }
  }

  private void getAllianceInfo(){
    if (DriverStation.getAlliance().isPresent()) {
      hasAlliance = true;
      if (DriverStation.getAlliance().get() == Alliance.Red){
        RobotState.getInstance().setSpeakerPose(Constants.ShooterConstants.RED_SPEAKER,
                                                Constants.ShooterConstants.RED_SPEAKER_ID);
      } else {
        RobotState.getInstance().setSpeakerPose(Constants.ShooterConstants.BLUE_SPEAKER,
                                                Constants.ShooterConstants.BLUE_SPEAKER_ID);
      }
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    if (!hasAlliance) {getAllianceInfo();}
    
    
    Monologue.setFileOnly(DriverStation.isFMSAttached());
    Monologue.updateAll();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    getAllianceInfo();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    getAllianceInfo();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    getAllianceInfo();
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    getAllianceInfo();
    m_robotContainer.m_robotDrive.setPose(new Pose2d(4,5, Rotation2d.fromDegrees(0)));
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

}
