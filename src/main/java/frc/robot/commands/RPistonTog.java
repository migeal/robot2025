// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.rPiston;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RPistonTog extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final rPiston m_rPiston;

  /**
   * Creates a new ExampleCommand.
   *
   * @param climbPistons The subsystem used by this command.
   */
  public RPistonTog(rPiston rPiston) {
    m_rPiston = rPiston;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rPiston);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rPiston.OutnIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
