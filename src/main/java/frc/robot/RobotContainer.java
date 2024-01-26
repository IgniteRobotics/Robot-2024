// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.comm.preferences.DoublePreference;
import frc.robot.commands.ParkCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunUmbrella;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.UmbrellaSubsystem;
import monologue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj. smartdashboard.Field2d;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Logged {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  private final UmbrellaSubsystem m_umbrella  = new UmbrellaSubsystem();


  //Robot preferences
  private DoublePreference intakePower = new DoublePreference("intake/intakePower", 0.5);
  private DoublePreference outtakePower = new DoublePreference("intake/outtakePower", 0.5);
  private DoublePreference umbrellaPower = new DoublePreference( "umbrella/Power", 0.25);

  private final SendableChooser<Command> autonChooser;

  private final double pathSpeed = 2;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(){

    // Configure the button bindings
    configureButtonBindings();

    autonChooser = new SendableChooser<>();
    autonChooser.addOption("Autonomous Prototype Path", m_robotDrive.followPathCommand("Auto Prototype Path", pathSpeed));
    autonChooser.addOption("Simple Path", m_robotDrive.followPathCommand("New Path", pathSpeed));
    autonChooser.addOption( "Two Ring Auto", m_robotDrive.followPathCommand("Two Ring Auto", pathSpeed));
    autonChooser.addOption("None", null);
    SmartDashboard.putData("Autonomous", autonChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new ParkCommand(m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new ResetGyro(m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunIntake(m_robotIntake, intakePower));
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
    .whileTrue(new RunIntake(m_robotIntake, outtakePower));
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(new RunUmbrella(m_umbrella, umbrellaPower));
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .onTrue(m_robotDrive.pathFindertoPoseBuilder(new Pose2d(8,5, Rotation2d.fromDegrees(0)),
                                                          2,2,6.28,6.28));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
 
    
}




