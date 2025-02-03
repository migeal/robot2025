package frc.robot.commands;

import frc.robot.subsystems.Rotate_roller;
import edu.wpi.first.wpilibj2.command.Command;

public class rotate_up extends command{
    private final Rotate_roller m_rotate;

    public pull_in(Rotate_roller m_rotate){
    m_rotate = Rotate_roller;
    addRequirements(Rotate_roller);
    }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {}
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        m_rotate.Rotate_up();
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        m_roller.stay();
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
}
