// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.Arrays;
import java.util.function.Supplier;
import monologue.Logged;
import monologue.Annotations.Log;
import edu.wpi.first.math.MathUtil;


public class StowShooter extends Command {
  private final ShooterSubsystem m_shooter;
  private final Supplier<Double> m_targetAngle;

  private final int m_windowSize = 5;
  private double[] m_positionMeasurements = new double[m_windowSize];
  private double[] m_currentMeasurements = new double[m_windowSize];
  private double[] m_powerMeasurements = new double[m_windowSize];
  private int m_index = 0;
  private int m_counter = 0;

  private final double MAX_POSITION = 0.5;
  private final double MAX_STDEV = 0.1;
  private final int INTERVAL = m_windowSize; 

  /** Creates a new RunShooterRPM. */
  public StowShooter(ShooterSubsystem shooter, Supplier<Double> angle) {
    m_shooter = shooter;
    m_targetAngle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  public StowShooter(ShooterSubsystem shooter, double angle)
  {
    m_shooter = shooter;
    m_targetAngle = () -> angle;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_counter = 0;
    m_index = 0;
    m_currentMeasurements = new double[m_windowSize];
    m_positionMeasurements = new double[m_windowSize];
    m_powerMeasurements = new double[m_windowSize];
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_shooter.armAtSetpoint()){
      m_shooter.setAngleDegrees(MathUtil.clamp(m_targetAngle.get(), 0, 120));
    }
    this.addMeasurements(m_shooter.getArmPosition(), m_shooter.getArmCurrent(), m_shooter.getArmPower());
    m_counter++;
    if (m_counter % INTERVAL == 0){
      if (
        getAvgCurrent() < 0 &&  //pushing down
        getAvgPosition() < MAX_POSITION &&  //we're close to home.
        getAvgPower() < 0 && // pushing down
        getStdDevCurrent() < MAX_STDEV &&  //current isn't changing
        getStdDevPosition() < MAX_STDEV && //postition isn't changing
        getStdDevPower() < MAX_STDEV // power isn't changing
      ) {
        m_shooter.resetPosition();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopPositioner();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return m_shooter.armAtSetpoint();
    return false;
  }

  private double mean(double[] values){
    return Arrays.stream(m_positionMeasurements).sum() / m_windowSize;
  }

  private double stdev(double[] values){

    double mean = this.mean(values);

    // calculate the standard deviation
    double standardDeviation = 0.0;
    for (double num : values) {
        standardDeviation += Math.pow(num - mean, 2);
    }

    return Math.sqrt(standardDeviation / m_windowSize);
  }

  private void addMeasurements(double position, double current, double power){
    m_positionMeasurements[m_index] = position;
    m_currentMeasurements[m_index] = current;
    m_powerMeasurements[m_index] = power;
    m_index++;
    if (m_index >= m_windowSize) {m_index = 0;}
  }

  private double getAvgPosition() {
    return this.mean(m_positionMeasurements);
  }

  private double getAvgCurrent() {
    return this.mean(m_currentMeasurements);
  }

  private double getAvgPower() {
    return this.mean(m_powerMeasurements);
  }

  private double getStdDevPosition(){
    return this.stdev(m_positionMeasurements);
  }

  private double getStdDevCurrent(){
    return this.stdev(m_currentMeasurements);
  }

  private double getStdDevPower(){
    return this.stdev(m_currentMeasurements);
  }





}
