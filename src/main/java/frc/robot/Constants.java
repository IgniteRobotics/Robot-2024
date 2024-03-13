// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.utils.InterCalculator;
import frc.utils.InterParameter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 3; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 6; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
  
    

    public static final boolean kGyroReversed = false;


    //Autonomous PID
    public static final double autokD = 0.1;
    public static final double autokP = 0.4;
    public static final double autoRotkP = 1;
    public static final double autoRotkD = 0;

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.0676651;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 0.5725325;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0.0104125;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }
  public static final class ShooterConstants {

    public static final double POSITION_kF = 1;
    public static final double POSITION_kP = 1;
    public static final double POSITION_kI = 0;
    public static final double POSITION_kD = 0;
    public static final double POSITION_kS = 0.24;
    public static final double POSITION_kV = 0.12;
    public static final double POSITION_ForwardsLimit = 33;
    public static final double POSITION_ReverseLimit = 0;


    public static final double ROLLER_kP = 9e-5; 
    public static final double ROLLER_kI = 0;
    public static final double ROLLER_kD = 0.001; 
    public static final double ROLLER_kF = 0.0001975; 
    public static final double ROLLER_MAX_OUTPUT = 1;
    public static final double ROLLER_MIN_OUTPUT = -1;
    public static final double ROLLER_MAX_RPM = 5700;

    public static final double Index_IntakeSpeed = 0.2;

    public static final double TARGET_POSITION_DEGREES = 56;
    public static final double SHOOTER_HOME_DEGREES = 2;

    // DISTANCE, ANGLE, RPM
    public static final InterCalculator SHOOTER_INTER_CALCULATOR = new InterCalculator(
      new InterParameter(0.91756992, 93, 3200),
      new InterParameter(1.22236992, 80, 3200),
      new InterParameter(1.52716992, 75, 3200),
      new InterParameter(1.83196992, 67, 3200),
      new InterParameter(2.13676992, 61, 3200),
      new InterParameter(2.44156992, 58, 3200),
      new InterParameter(2.74636992, 53, 3200),
      new InterParameter(3.05116992, 50, 3200),
      new InterParameter(3.35596992, 50, 3200),
      new InterParameter(3.66076992, 49, 3200),
      new InterParameter(3.96556992, 39, 3200)
      );

    //RED 4 
    //"x": 16.579342,
    //"y": 5.547867999999999, 
    //BLUE 7
    //"x": -0.038099999999999995,
    //"y": 5.547867999999999,
    public static final int RED_SPEAKER_ID = 4;
    public static final Pose2d RED_SPEAKER = new Pose2d(16.58, 5.55, Rotation2d.fromDegrees(0));
    public static final int BLUE_SPEAKER_ID = 7;
    public static final Pose2d BLUE_SPEAKER =  new Pose2d(0, 5.55, Rotation2d.fromDegrees(180));


    //TODO: TUNE
    public static final double POSITION_TOLERANCE = 2.0;

    public static final double VELOCITY_TOLERANCE = 75;

    public static final double LENGTH = 1.5;

    //100 to 1 gear ration
    public static final double POSITION_DEGREE_PER_MOTOR_REV = 360.0/100.0;

    
    public static final double ELEVATION = Units.inchesToMeters(9.46);

   
    public static final double TRANSLATION_OFFSET = Units.inchesToMeters(1.66);

    // shooter motion magic constants
    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 400;
    public static final double MOTION_MAGIC_ACCELERATION = 1000; //full speed in .5 seconds
    public static final double MOTION_MAGIC_JERK = 2000;

    public static final double AUTO_TARGET_ROT_kP = 0.014;
    public static final double AUTO_TARGET_ROT_kD = 0.0;
  }

  public static final class ClimberConstants{
    
    public static final double POSITION_kP = 1.0;
    public static final double POSITION_kI = 0.0;
    public static final double POSITION_kD = 0.0;
    public static final double POSITION_kS = 0.12;
    public static final double POSITION_kV = 0.24;

    public static final double CLIMBER_TOLERANCE = 2.0;

    public static final double UPPER_LIMIT = 69;
    public static final double LOWER_LIMIT = 0;
    public static final double TOP_POSITION = 5;
    public static final double BOTTOM_POSITION = 1;

    public static final double POWER= 0.1;
  }

  public static final class IntakeConstants {
    public static double POSITION_kP = .5;
    public static double POSITION_kI = 0;
    public static double POSITION_kD = 0;

    public static final double POSITION_TOLERANCE = 0.5;
    
  }
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipControllerPort = 1;
    public static final double kDriveDeadband = 0.15;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static class CameraConstants {
    public static final String photonCameraNameFrontLeft = "FRONT_LEFT";
    // public static final Transform3d photonCameraTransformLeft = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public static final Transform3d photonCameraTransformFrontLeft = new Transform3d(new Translation3d(.324, 0.298, .235), new Rotation3d(0.0, -10 / 180.0 * Math.PI, -15.0/180 * Math.PI));
    public static final String photonCameraNameFrontRight = "FRONT_RIGHT";
    public static final Transform3d photonCameraTransformFrontRight = new Transform3d(new Translation3d(.324, -0.298, .235), new Rotation3d(0.0, -10 / 180.0 * Math.PI, 15.0/180 * Math.PI));
    public static final String photonCameraNameRearRight = "REAR_RIGHT";
    public static final Transform3d photonCameraTransformRearRight = new Transform3d(new Translation3d(-.324, -0.298, 0.235), new Rotation3d(0.0, -10 / 180.0 * Math.PI, -195.0/180 * Math.PI));
    public static final String photonCameraNameRearLeft = "REAR_LEFT";
    public static final Transform3d photonCameraTransformRearLeft = new Transform3d(new Translation3d(-.324, 0.298, 0.235), new Rotation3d(0.0, -10 / 180.0 * Math.PI, 195.0/180 * Math.PI));

    public static final Integer[] IGNORED_POSE_TARGETS = {3,4,7,8};

  }


  public class CANConstants {
    public static final int INTAKE_POSITION_MOTOR = 2;
    public static final int INTAKE_ROLLER_MOTOR = 1;
    public static final int UMBRELLA_MOTOR_1 = 3;

    public static final int CLIMBER_MOTOR = 4;
    
    public static final int SHOOTER_POSITION_MOTOR = 7;
    public static final int SHOOTER_MOTOR_LEADERCanId = 8;
    public static final int SHOOTER_INDEX_MOTOR = 9;

    public static final int FRONT_LEFT_DRIVE = 11;
    public static final int REAR_LEFT_DRIVE = 13;
    public static final int FRONT_RIGHT_DRIVE = 15;
    public static final int REAR_RIGHT_DRIVE = 17;

    public static final int FRONT_LEFT_TURN = 10;
    public static final int REAR_LEFT_TURN = 12;
    public static final int FRONT_RIGHT_TURN = 14;
    public static final int REAR_RIGHT_TURN = 16;

  }


}
