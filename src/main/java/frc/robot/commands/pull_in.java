package frc.robot.commands;

import frc.robot.subsystems.Rollor;

import java.lang.reflect.Parameter;

import edu.wpi.first.wpilibj2.command.Command;
public class pull_in extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

private final Rollor m_rollor;
/**
   * Creates a new ExampleCommand.
   *
   * @param Rollor The subsystem used by this command.
   */
public pull_in(Rollor Rollor){
m_rollor = Rollor;
addRequirements(Rollor);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_rollor.grab();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rollor.hold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}