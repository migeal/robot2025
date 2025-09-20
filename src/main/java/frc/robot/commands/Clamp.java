// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class Clamp extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climb m_climb;

  /**
   * Creates a new ExampleCommand.
   *
   * @param Climb The subsystem used by this command.
   */
  public Clamp(Climb Climb) {
    m_climb = Climb;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.Limit==false){
    m_climb.climb();
    }
    else{
      m_climb.LBclimb();}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
