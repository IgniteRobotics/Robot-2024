// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.comm.preferences.DoublePreference;

/**
 * A command that does nothing but takes a specified amount of time to finish.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class AutoWait extends Command {
  /** The timer used for waiting. */
  protected Timer m_timer = new Timer();

  private final double m_duration;

  /**
   * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
   *
   * @param seconds the time to wait, in seconds
   */
  @SuppressWarnings("this-escape")
  public AutoWait(DoublePreference seconds) {
    m_duration = seconds.get();
    SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_duration);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("duration", () -> m_duration, null);
  }
}
