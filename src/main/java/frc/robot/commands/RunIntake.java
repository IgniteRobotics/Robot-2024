package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends Command {
  private final IntakeSubsystem m_intake;
  private final Supplier<Double> m_powSupplier;

  

  /** Creates a new Run Intake Command. */
  public RunIntake(IntakeSubsystem m_subsystem,  Supplier<Double> powSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = m_subsystem;
    m_powSupplier = powSupplier;
    addRequirements(m_intake);
  }
  


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setSpeed(m_powSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

 


