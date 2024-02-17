// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.comm.preferences.DoublePreference;
import frc.robot.commands.ParkCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.RunUmbrella;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.RunShooterPower;
import frc.robot.commands.RunShooterRPM;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UmbrellaSubsystem;
import monologue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;



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
  //private final UmbrellaSubsystem m_umbrella  = new UmbrellaSubsystem();
  //private final ShooterSubsystem m_shooter = new ShooterSubsystem();


  //Robot preferences
  private DoublePreference intakePower = new DoublePreference("intake/intakePower", 0.5);
  private DoublePreference intakePosition = new DoublePreference("intake/intakePosition", 100);
  private DoublePreference outtakePower = new DoublePreference("intake/outtakePower", 0.5);
  private DoublePreference umbrellaPower = new DoublePreference( "umbrella/Power", 0.25);
  private DoublePreference shooterPower = new DoublePreference("shooter/Power", 0.25);
  private DoublePreference shooterRPM = new DoublePreference("shooter/RPM", 500);

  private final SendableChooser<Command> autonChooser;

  private final double pathSpeed = 2;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_manipController = new XboxController(OIConstants.kManipControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(){

    // Configure the button bindings
    configureButtonBindings();



    autonChooser = AutoBuilder.buildAutoChooser();
    autonChooser.addOption("Forward 2 Meters", AutoBuilder.buildAuto("Forward 2 meters"));
    autonChooser.addOption("Forward 2, Back 2", AutoBuilder.buildAuto("Forward 2, Back 2"));
    autonChooser.addOption("Forward 2, Back 2 with Rotation", AutoBuilder.buildAuto("Forward 2, Back 2 with Rotation"));
    autonChooser.addOption("Circle Around Stage", AutoBuilder.buildAuto("Circle Around Stage"));
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
        .whileTrue(new RunIntake(m_robotIntake, intakePower, intakePosition));
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
    .whileTrue(new RunIntake(m_robotIntake, outtakePower, intakePosition));
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(m_robotDrive.driveSysIdTestBuilder(6, 3));
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(m_robotDrive.turnSysIdTestBuilder(10, 5));
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




