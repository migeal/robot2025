package frc.robot.commands;

import frc.robot.subsystems.Rotate_rollor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class rotate_up extends Command{
    private final Rotate_rollor m_rotate;

    public rotate_up(Rotate_rollor Rotate_rollor){
    m_rotate = Rotate_rollor;
    addRequirements(Rotate_rollor);
    }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {}
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        if(RobotContainer.Limit=false){
        m_rotate.Rotate_up();
        }
        else{m_rotate.LBRotate_Up();}
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        m_rotate.stay(); 
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
}
