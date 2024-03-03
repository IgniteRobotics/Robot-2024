// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.comm.preferences.DoublePreference;
import frc.robot.commands.ParkCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.RunUmbrella;
import frc.robot.commands.intake.IntakePiece;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.StowIntake;
import frc.robot.input.AxisButton;
import frc.robot.commands.Shooter.RunShooterPower;
import frc.robot.commands.Shooter.RunShooterRPM;
import frc.robot.commands.Shooter.ShootPiece;
import frc.robot.commands.Shooter.IndexPower;
import frc.robot.commands.Shooter.PositionShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UmbrellaSubsystem;
import monologue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.commands.ResetGyro;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;




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
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();


  //Robot preferences
  private DoublePreference intakePower = new DoublePreference("intake/intakePower", 0.5);
  private DoublePreference intakePosition = new DoublePreference("intake/intakePosition", 100);
  private DoublePreference outtakePower = new DoublePreference("intake/outtakePower", -0.75);
  private DoublePreference umbrellaPower = new DoublePreference( "umbrella/Power", 0.25);
  private DoublePreference shooterPower = new DoublePreference("shooter/Power", 0.25);
  private DoublePreference shooterRPM = new DoublePreference("shooter/RPM", 500);
  private DoublePreference intakeShooterPosition = new DoublePreference("shooter/Position", Constants.ShooterConstants.TARGET_POSITION_DEGREES);
  private DoublePreference indexPower = new DoublePreference("shooter/IndexPower", 0.5);
  private DoublePreference intakeIndexPower = new DoublePreference("shooter/IndexPower", 0.25);
  private DoublePreference shooterOuttakePower = new DoublePreference("shooter/OuttakePower", -0.1);  
  private DoublePreference shooterPosition = new DoublePreference("shooter/shootingPosition", 65); 
  private DoublePreference outdexPower = new DoublePreference("shooter/OutdexPower", -0.1);
  


  //Low Angle, Mid Angle, High Angle
  private DoublePreference shooterLowAngle = new DoublePreference("shooter/lowAngle", 35);
  private DoublePreference shooterMidAngle = new DoublePreference("shooter/midAngle", 50);
  private DoublePreference shooterHighAngle = new DoublePreference("shooter/highAngle", 75);

  //High power
  private DoublePreference shooterHighPower = new DoublePreference("shooter/highPower", 78);
  

   //preferences for slew rates
   private DoublePreference m_kDirectionSlewRate = new DoublePreference("Direction Slew Rate", Constants.DriveConstants.kDirectionSlewRate);
   private DoublePreference m_kMagnitudeSlewRate = new DoublePreference("Magnitude Slew Rate", Constants.DriveConstants.kMagnitudeSlewRate);
   private DoublePreference m_kRotationalSlewRate = new DoublePreference("Rotational Slew Rate", Constants.DriveConstants.kRotationalSlewRate);


    private final Command resetGyro = new ResetGyro(m_robotDrive);
    private final Command intakeCommand = new RunIntake(m_robotIntake, intakePower, intakePosition);
    private final Command extakeCommand = new RunIntake(m_robotIntake, outtakePower, intakePosition);
    private final Command intakePiece = new IntakePiece(m_robotIntake, m_shooter, intakePower, intakePosition, indexPower, intakeShooterPosition);
    private final Command stowIntake = new StowIntake(m_robotIntake);
    private final Command parkCommand = new ParkCommand(m_robotDrive);
    private final Command stowShooter = new PositionShooter(m_shooter, Constants.ShooterConstants.SHOOTER_HOME_DEGREES);
    private final Command raiseShooter = new PositionShooter(m_shooter, intakeShooterPosition);
    private final Command spinShooter = new RunShooterPower(m_shooter, shooterPower);
    private final Command spinIndex = new IndexPower(m_shooter, outdexPower, outtakePower);
    private final Command shootPiece = new ShootPiece(m_shooter, shooterPosition, shooterPower, indexPower, () -> Operator.driver_rightTrigger.getAsBoolean());
    private final Command shootHighAngle = new ShootPiece(m_shooter, shooterHighAngle, shooterPower, indexPower, () -> Operator.driver_leftTrigger.getAsBoolean());
    private final Command shootMidAngle = new ShootPiece(m_shooter, shooterMidAngle, shooterPower, indexPower, () -> Operator.driver_leftTrigger.getAsBoolean());
    private final Command shootLowAngle = new ShootPiece(m_shooter, shooterLowAngle, shooterHighPower, indexPower, () -> Operator.driver_leftTrigger.getAsBoolean());
  private final SendableChooser<Command> autonChooser;

  private final double pathSpeed = 2;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_manipController = new XboxController(OIConstants.kManipControllerPort);  
    
private static class Operator {

    private static Joystick driver = new Joystick(0);

    private static DoubleSupplier driver_axisLX = () -> MathUtil.applyDeadband(driver.getRawAxis(0), Constants.OIConstants.kDriveDeadband);
    private static DoubleSupplier driver_axisLY = () -> MathUtil.applyDeadband(-driver.getRawAxis(1), Constants.OIConstants.kDriveDeadband);
    private static DoubleSupplier driver_axisRY = () -> MathUtil.applyDeadband(-driver.getRawAxis(5), Constants.OIConstants.kDriveDeadband);

    private static JoystickButton driver_x = new JoystickButton(driver, XboxController.Button.kX.value);
    private static JoystickButton driver_a = new JoystickButton(driver, XboxController.Button.kA.value);
    private static JoystickButton driver_b = new JoystickButton(driver, XboxController.Button.kB.value);
    private static JoystickButton driver_y = new JoystickButton(driver, XboxController.Button.kY.value);
    private static JoystickButton driver_start = new JoystickButton(driver, XboxController.Button.kStart.value);
    private static JoystickButton driver_back = new JoystickButton(driver, XboxController.Button.kBack.value);
    private static JoystickButton driver_leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private static JoystickButton driver_rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private static AxisButton driver_rightTrigger = new AxisButton(driver, XboxController.Axis.kRightTrigger.value, 0.25);
    private static AxisButton driver_leftTrigger = new AxisButton(driver, XboxController.Axis.kLeftTrigger.value, 0.25);
    private static POVButton driver_dpad_up = new POVButton(driver, 0);
    private static POVButton driver_dpad_right = new POVButton(driver, 90);
    private static POVButton driver_dpad_down= new POVButton(driver, 180);
    private static POVButton driver_dpad_left = new POVButton(driver, 270);
    

    // subsystems


}

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(){

    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();



    autonChooser = AutoBuilder.buildAutoChooser();
    autonChooser.addOption("Forward 2 Meters", AutoBuilder.buildAuto("Forward 2 meters"));
    autonChooser.addOption("Forward 2, Back 2", AutoBuilder.buildAuto("Forward 2, Back 2"));
    autonChooser.addOption("Forward 2, Back 2 with Rotation", AutoBuilder.buildAuto("Forward 2, Back 2 with Rotation"));
    autonChooser.addOption("Circle Around Stage", AutoBuilder.buildAuto("Circle Around Stage"));
    autonChooser.addOption("None", null);
    autonChooser.addOption("de test", AutoBuilder.buildAuto("de test"));
    SmartDashboard.putData("Autonomous", autonChooser);


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

    
   Operator.driver_start.whileTrue(resetGyro);
   Operator.driver_rightBumper.whileTrue(intakePiece);
   Operator.driver_leftBumper.whileTrue(extakeCommand);
   Operator.driver_back.onTrue(parkCommand);
   Operator.driver_dpad_up.whileTrue(new InstantCommand(()-> m_shooter.moveArm(.1), m_shooter));
   Operator.driver_dpad_down.whileTrue(new InstantCommand(() -> m_shooter.moveArm(-.1), m_shooter));
  //  Operator.driver_leftTrigger.whileTrue(spinIndex);
  //  Operator.driver_rightTrigger.whileTrue(spinShooter);
   //Operator.driver_leftTrigger.whileTrue(shootPiece);
   Operator.driver_a.whileTrue(shootHighAngle);
   Operator.driver_b.whileTrue(shootMidAngle);
   Operator.driver_y.whileTrue(shootLowAngle);

   //Operator.driver_y.whileTrue(raiseShooter);
   //Operator.driver_b.onTrue(stowShooter);
   Operator.driver_dpad_left.whileTrue(spinIndex);

    // new JoystickButton(m_driverController, XboxController.Button.kY.value)
    //     .whileTrue(m_robotDrive.driveSysIdTestBuilder(6, 3));
    // new JoystickButton(m_driverController, XboxController.Button.kB.value)
    //     .whileTrue(m_robotDrive.turnSysIdTestBuilder(10, 5));
    

   
  }

  private void configureDefaultCommands(){
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

    m_robotIntake.setDefaultCommand(stowIntake);
    //m_shooter.setDefaultCommand(stowShooter);

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




