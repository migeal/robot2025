package frc.robot.commands;

import frc.robot.subsystems.Roller;
import edu.wpi.first.wpilibj2.command.Command;

public class push_outextends Command {

    private final Roller m_roller;

    public pull_in(Roller m_roller){
    m_roller = Roller;
    addRequirements(Roller);
    }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {}
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        m_roller.release();
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        m_roller.hold();
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
}
