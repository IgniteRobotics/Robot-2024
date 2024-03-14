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
import frc.robot.Constants.ClimberConstants;
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
import frc.robot.commands.Shooter.ShootInterpolated;
import frc.robot.commands.Shooter.ShootPiece;
import frc.robot.commands.climb.ClimbMM;
import frc.robot.commands.climb.ClimbPower;
import frc.robot.commands.drive.DriveToTarget;
import frc.robot.commands.drive.TurnDegrees;
import frc.robot.commands.Shooter.AutonShoot;
import frc.robot.commands.Shooter.IndexPower;
import frc.robot.commands.Shooter.PositionShooter;
import frc.robot.commands.Shooter.PrepareShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UmbrellaSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PhotonCameraWrapper;
import monologue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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

  private final PhotonCameraWrapper m_photonCameraWrapper = new PhotonCameraWrapper();

  protected final DriveSubsystem m_robotDrive = new DriveSubsystem(m_photonCameraWrapper);
  
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  //private final UmbrellaSubsystem m_umbrella  = new UmbrellaSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  private final Climber m_Climber = new Climber();

  
  private final RobotState m_robotState = RobotState.getInstance();

  //Robot preferences
  private DoublePreference intakePower = new DoublePreference("intake/intakePower", 0.5);
  private DoublePreference intakePosition = new DoublePreference("intake/intakePosition", 100);
  private DoublePreference outtakePower = new DoublePreference("intake/outtakePower", -0.75);
  private DoublePreference umbrellaPower = new DoublePreference( "umbrella/Power", 0.25);
  private DoublePreference shooterPower = new DoublePreference("shooter/Power", 0.79);
  private DoublePreference shooterRPM = new DoublePreference("shooter/RPM", 4000);
  private DoublePreference intakeShooterPosition = new DoublePreference("shooter/Position", Constants.ShooterConstants.TARGET_POSITION_DEGREES);
  private DoublePreference indexPower = new DoublePreference("shooter/IndexPower", 0.1);
  private DoublePreference shooterIndexPower = new DoublePreference("shooter/ShootingIndexPower", 0.5);
  private DoublePreference shooterOuttakePower = new DoublePreference("shooter/OuttakePower", -0.1);  
  private DoublePreference shooterPosition = new DoublePreference("shooter/shootingPosition", 65); 
  private DoublePreference outdexPower = new DoublePreference("shooter/OutdexPower", -0.1);
  private DoublePreference preSpinDistanceM = new DoublePreference("shooter/preSpinDistanceM", 5.842);

  private DoublePreference climberUpPower = new DoublePreference("climber/UpPower", ClimberConstants.POWER);
  private DoublePreference climberDownPower = new DoublePreference("climber/DownPower", -ClimberConstants.POWER);
  private DoublePreference climberUpPosition = new DoublePreference("climber/UpPosition", ClimberConstants.TOP_POSITION);
  private DoublePreference climberDownPosition = new DoublePreference("climber/DownPosition", ClimberConstants.BOTTOM_POSITION);


  //For tuning
  private DoublePreference tuningPower = new DoublePreference("shooter/tuning_rpm", 2500);
  private DoublePreference tuningPosition = new DoublePreference("shooter/tuning_Position", 65);


  //Low Angle, Mid Angle, High Angle
  private DoublePreference wingShotAngle = new DoublePreference("shooter/wingShotAngle", 34);
  private DoublePreference podiumShotAngle = new DoublePreference("shooter/podiumShotAngle", 52.5);
  private DoublePreference subShotAngle = new DoublePreference("shooter/subShotAngle", 93);

  //Canned shot RPM
  private DoublePreference wingShotRPM = new DoublePreference("shooter/wingRPM", 4000);
  private DoublePreference podiumShotRPM = new DoublePreference("shooter/podiumRPM", 3200);
  private DoublePreference subShotRPM = new DoublePreference("shooter/subRPM", 3200);

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
    private final Command stowShooter = new PositionShooter(m_shooter, intakePosition);
    private final Command raiseShooter = new PositionShooter(m_shooter, intakeShooterPosition);
    private final Command spinShooter = new RunShooterPower(m_shooter, shooterPower);
    private final Command spinIndex = new IndexPower(m_shooter, outdexPower, outtakePower);
    private final Command shootSubwoofer = new ShootPiece(m_shooter, subShotAngle, subShotRPM, shooterIndexPower, () -> Operator.driver_leftTrigger.getAsBoolean());
    private final Command shootPodium = new ShootPiece(m_shooter, podiumShotAngle, podiumShotRPM, shooterIndexPower, () -> Operator.driver_leftTrigger.getAsBoolean());
    private final Command shootWing = new ShootPiece(m_shooter, wingShotAngle, wingShotRPM, shooterIndexPower, () -> Operator.driver_leftTrigger.getAsBoolean());
    private final Command spinRPM = new RunShooterRPM(m_shooter, shooterRPM);
    private final Command preSpinShooter = new PrepareShooter(m_shooter, preSpinDistanceM);

    private final Command climberPowerUp = new ClimbPower(m_Climber, climberUpPower);
    private final Command climberPowerDown = new ClimbPower(m_Climber, climberDownPower);
    private final Command climbMMUp = new ClimbMM(m_Climber, climberUpPosition);
    private final Command climbMMDown = new ClimbMM(m_Climber, climberDownPosition);

    private final Command testTurnPID = new TurnDegrees(m_robotDrive, 15);

    private final Command shooterTune = new ShootPiece(m_shooter, tuningPosition, tuningPower, shooterIndexPower, () -> Operator.driver_leftTrigger.getAsBoolean());
    private final Command shootInterpolated = new ShootInterpolated(m_shooter, m_photonCameraWrapper, shooterIndexPower, () -> Operator.driver_leftTrigger.getAsBoolean());
    private final Command driveToTarget = new DriveToTarget(m_robotDrive, 
            m_photonCameraWrapper, 
            m_robotState::getSpeakerID,
            Operator.driver_axisLY, 
            Operator.driver_axisLX);

            private final Command autoShootSubwoofer = new AutonShoot(m_shooter, subShotAngle, subShotRPM, shooterIndexPower);

    private final ParallelCommandGroup speakerShotGroup = new ParallelCommandGroup(shootInterpolated, driveToTarget);
    
    private final SendableChooser<Command> autonChooser;

  private final double pathSpeed = 2;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_manipController = new XboxController(OIConstants.kManipControllerPort);  
    
private static class Operator {

    private static Joystick driver = new Joystick(0);

    private static Supplier<Double> driver_axisLX = () -> MathUtil.applyDeadband(-driver.getRawAxis(0), Constants.OIConstants.kDriveDeadband);
    private static Supplier<Double> driver_axisLY = () -> MathUtil.applyDeadband(-driver.getRawAxis(1), Constants.OIConstants.kDriveDeadband);
    private static Supplier<Double> driver_axisRY = () -> MathUtil.applyDeadband(-driver.getRawAxis(5), Constants.OIConstants.kDriveDeadband);

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

    //TODO: reconcile these with paths.
    NamedCommands.registerCommand("FirstShot", autoShootSubwoofer);
    NamedCommands.registerCommand("RunIntake", intakePiece);
    NamedCommands.registerCommand("SecondShot", autoShootSubwoofer);

    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();



    autonChooser = AutoBuilder.buildAutoChooser();
    autonChooser.addOption("Forward 2 Meters", AutoBuilder.buildAuto("Forward 2 meters"));
    autonChooser.addOption("None", null);
    autonChooser.addOption("de test", AutoBuilder.buildAuto("de test"));
    autonChooser.addOption("CenterTwoRing", AutoBuilder.buildAuto("CenterTwoRing"));
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

   Operator.driver_dpad_up.whileTrue(climberPowerUp);
   Operator.driver_dpad_down.whileTrue(climberPowerDown);

  //  Operator.driver_dpad_up.whileTrue(new InstantCommand(()-> m_shooter.moveArm(.1), m_shooter));
  //  Operator.driver_dpad_down.whileTrue(new InstantCommand(() -> m_shooter.moveArm(-.1), m_shooter));
   //Operator.driver_a.whileTrue(shootHighAngle);
   //Operator.driver_b.whileTrue(shootMidAngle);
   //Operator.driver_y.whileTrue(shootLowAngle);
   //Operator.driver_x.whileTrue(shooterTune);

   //Operator.driver_y.whileTrue(raiseShooter);
   //Operator.driver_b.onTrue(stowShooter);
  //  Operator.driver_dpad_left.whileTrue(spinIndex);
  //  Operator.driver_dpad_right.whileTrue(spinRPM);
   Operator.driver_a.whileTrue(shootSubwoofer);
   Operator.driver_b.whileTrue(shootPodium);
   Operator.driver_y.whileTrue(shootWing);
   Operator.driver_x.whileTrue(speakerShotGroup);

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
    m_shooter.setDefaultCommand(preSpinShooter);

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




