package frc.robot.commands;

import frc.robot.subsystems.Rollor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
public class push_out extends Command {

    private final Rollor m_rollor;

    public push_out(Rollor Rollor){
    m_rollor = Rollor;
    addRequirements(Rollor);
    }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {}
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        m_rollor.release();

      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        m_rollor.stop();
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        
        return false;
      }
}
