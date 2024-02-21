// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.auto.AutoBuilder;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import monologue.Logged;
import monologue.Annotations.Log;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import frc.robot.subsystems.drive.PhotonCameraWrapper;
import frc.robot.subsystems.drive.PhotonCameraWrapper.Side;




public class DriveSubsystem extends SubsystemBase implements Logged{
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private final PhotonCameraWrapper m_photonCameraWrapper;
  private final SwerveDrivePoseEstimator poseEstimator;

  private SwerveModuleState[] m_moduleStates = new SwerveModuleState[4];


  // Slew rate filter variables for controlling lateral acceleration
  @Log.NT
@Log.File
  private double m_currentRotation = 0.0;
  @Log.NT
@Log.File
  private double m_currentTranslationDir = 0.0;
  @Log.NT
@Log.File
  private double m_currentTranslationMag = 0.0;
  @Log.NT
@Log.File
  private PathPlannerPath logged_path; 

  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.DriveConstants.kMagnitudeSlewRate);
  SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.DriveConstants.kRotationalSlewRate);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  //sysid routines for characterization
  SysIdRoutine m_DriveSysIdRoutine;
  SysIdRoutine m_TurnSysIdRoutine;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  
    m_photonCameraWrapper = new PhotonCameraWrapper();
    poseEstimator = new SwerveDrivePoseEstimator(
                          DriveConstants.kDriveKinematics, 
                          this.getYaw(),
                          new SwerveModulePosition[] {
                            m_frontLeft.getPosition(),
                            m_frontRight.getPosition(),
                            m_rearLeft.getPosition(),
                            m_rearRight.getPosition()
                            },
                            new Pose2d(),
                            VecBuilder.fill(0.95, 0.95, 0.95), 
                            VecBuilder.fill(0.05, 0.05, 0.05)
                          );

  // Configure AutoBuilder last
  AutoBuilder.configureHolonomic(
    this::getPose, // Robot pose supplier
    this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
    ),
    () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
        },
    this // Reference to this subsystem to set requirements
    );
    
    m_DriveSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> this.setDriveVolts(volts.in(Volts)),
        null, // No log consumer, since data is recorded by URCL
      this
      ) 
    );
    
    m_TurnSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> this.setTurnVolts(volts.in(Volts)),
        null, // No log consumer, since data is recorded by URCL
      this
      ) 
    );

   
  }

  @Override
  public void periodic(
  ) {
    // Update the odometry in the periodic block

    m_frontLeft.periodic();
    m_frontRight.periodic();
    m_rearLeft.periodic();
    m_rearRight.periodic();

    
    m_odometry.update(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
  
        });
    SmartDashboard.putNumber("left front turn", m_frontLeft.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("left front drive", m_frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("left back turn", m_rearLeft.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("left back drive", m_rearLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("right front turn", m_frontRight.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("right front drive", m_frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("right back turn", m_rearRight.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("right back drive", m_rearRight.getState().speedMetersPerSecond);


        

    m_moduleStates[0] = m_frontLeft.getState();
    m_moduleStates[1] = m_frontRight.getState();
    m_moduleStates[2] = m_rearLeft.getState();
    m_moduleStates[3] = m_rearRight.getState();

    Optional<EstimatedRobotPose> estimatedPoseFrontLeft = m_photonCameraWrapper.getEstimatedGlobalPose(getPose(), Side.FRONT_LEFT);
    Optional<EstimatedRobotPose> estimatedPoseFrontRight = m_photonCameraWrapper.getEstimatedGlobalPose(getPose(), Side.FRONT_RIGHT);
    Optional<EstimatedRobotPose> estimatedPoseRearLeft = m_photonCameraWrapper.getEstimatedGlobalPose(getPose(), Side.REAR_LEFT);
    Optional<EstimatedRobotPose> estimatedPoseRearRight = m_photonCameraWrapper.getEstimatedGlobalPose(getPose(), Side.REAR_RIGHT);
    if (estimatedPoseFrontLeft.isPresent()) {
      EstimatedRobotPose pose = estimatedPoseFrontLeft.get();
      boolean poseOK = true;
      for(PhotonTrackedTarget target: pose.targetsUsed) {
          if(target.getPoseAmbiguity() > 0.2) poseOK = false;
      }
      if(poseOK) poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    }

    if(estimatedPoseFrontRight.isPresent()) {
      EstimatedRobotPose pose = estimatedPoseFrontRight.get();
      boolean poseOK = true;
      for(PhotonTrackedTarget target: pose.targetsUsed) {
          if(target.getPoseAmbiguity() > 0.2) poseOK = false;
      }
      if(poseOK) poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    }

    if (estimatedPoseRearLeft.isPresent()) {
      EstimatedRobotPose pose = estimatedPoseRearLeft.get();
      boolean poseOK = true;
      for(PhotonTrackedTarget target: pose.targetsUsed) {
          if(target.getPoseAmbiguity() > 0.2) poseOK = false;
      }
      if(poseOK) poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    }

    if(estimatedPoseRearRight.isPresent()) {
      EstimatedRobotPose pose = estimatedPoseRearRight.get();
      boolean poseOK = true;
      for(PhotonTrackedTarget target: pose.targetsUsed) {
          if(target.getPoseAmbiguity() > 0.2) poseOK = false;
      }
      if(poseOK) poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    }

    poseEstimator.update(new Rotation2d(m_gyro.getYaw()), new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    });

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  @Log.NT
  @Log.File
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    SmartDashboard.putNumber("x stick speed", xSpeed);
    SmartDashboard.putNumber("y stick speed", ySpeed);
    SmartDashboard.putNumber("rot stick ", rot);

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(Constants.DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);  
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Set all drive motors to voltage
   * 
   * @param volts
   */
  public void setDriveVolts(double volts){
    m_frontLeft.setDriveVolts(volts);
    m_frontRight.setDriveVolts(volts);
    m_rearLeft.setDriveVolts(volts);
    m_rearRight.setDriveVolts(volts);
  }

  /**
   * Set all trurn motors to voltage
   * 
   * @param volts
   */
  public void setTurnVolts(double volts){
    m_frontLeft.setTurnVolts(volts);
    m_frontRight.setTurnVolts(volts);
    m_rearLeft.setTurnVolts(volts);
    m_rearLeft.setTurnVolts(volts);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  //set the modules directly to zero without optimization
  public void setModulesZero() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false);
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false);
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false);
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false);
  }
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(getAngle()).getDegrees();
  }

  public double getAngle() {
    return -m_gyro.getAngle();
  }

  public Rotation2d getYaw() {
    return m_gyro.getRotation2d().times(-1); // this inversion is a property of the AHRSGyro itself
}

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_moduleStates);
  }

  
  
  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
public  Command followPathCommand(String pathName, double speed) {
  //PathPlannerTrajectory​(PathPlannerPath path, ChassisSpeeds startingSpeeds, Rotation2d startingRotation)
  // PathPlannerTrajectory traj = new PathPlannerTrajectory();

  PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
  logged_path = path;

  
   return new FollowPathHolonomic(
                path,
                this::getPose, // Robot pose supplier
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        speed, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );



}



/**
 * @param targetPoseSupplier Supplier for the target pose to move to
 * @param maxVelSupplier Supplier for Max Linear Velocity in meters/sec
 * @param maxAccelSupplier Supplier for Max Linear Acceleration in meters/sec^2
 * @param maxRotSupplier Supplier for Max Angular Velocity in radians/sec
 * @param maxRotAccelSupplier Supplier for Max Angular Acceleration in meters/sec^2
 * @return A command group that follows the new path
 */
  public Command pathFindertoPoseBuilder(
    Supplier<Pose2d> targetPoseSupplier,
    DoubleSupplier maxVelSupplier,
    DoubleSupplier maxAccelSupplier,
    DoubleSupplier maxRotSupplier,
    DoubleSupplier maxRotAccelSupplier){

    Pose2d targetPose = targetPoseSupplier.get();
    double maxLinVelMPS = maxVelSupplier.getAsDouble();
    double maxLinAccelMPSSq = maxAccelSupplier.getAsDouble();
    double maxAglVelocityRps = maxRotSupplier.getAsDouble();
    double maxAngAccelRpsSq = maxRotAccelSupplier.getAsDouble();

    return this.pathFindertoPoseBuilder(targetPose, maxLinVelMPS, maxLinAccelMPSSq, maxAglVelocityRps, maxAngAccelRpsSq);

  }

  /**
   * @param targetPose The target pose to move to
   * @param maxLinVelMPS Max Linear Velocity in meters/sec
   * @param maxLinAccelMPSSq Max Linear Acceleration in meters/sec^2
   * @param maxAglVelocityRps Max Angular Velocity in radians/sec
   * @param maxAngAccelRpsSq Max Angular Acceleration in meters/sec^2
   * @return A command group that follows the new path
   */
  public Command pathFindertoPoseBuilder(
    Pose2d targetPose,
    double maxLinVelMPS,
    double maxLinAccelMPSSq,
    double maxAglVelocityRps,
    double maxAngAccelRpsSq){ 
    PathConstraints constraints = new PathConstraints(maxLinVelMPS, maxLinAccelMPSSq, maxAglVelocityRps, maxAngAccelRpsSq);

    return AutoBuilder.pathfindToPose(
          targetPose,
          constraints,
          0.0, // Goal end velocity in meters/sec
          0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
      );
  }

  public Command driveSysIdTestBuilder(double staticTimeout, double dynamicTimeout){
    return
      new InstantCommand(() -> this.setModulesZero())
      .andThen(m_DriveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(staticTimeout))
      .andThen(m_DriveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(staticTimeout))
      .andThen(m_DriveSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(dynamicTimeout))
      .andThen(m_DriveSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(dynamicTimeout))
      .finallyDo(() -> this.setDriveVolts(0));
  }

  public Command turnSysIdTestBuilder(double staticTimeout, double dynamicTimeout){ 
    return m_TurnSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(staticTimeout)
      .andThen(m_TurnSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(staticTimeout))
      .andThen(m_TurnSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(dynamicTimeout))
      .andThen(m_TurnSysIdRoutine   .dynamic(SysIdRoutine.Direction.kReverse).withTimeout(dynamicTimeout))
      .finallyDo(() -> this.setTurnVolts(0));
  }

}
