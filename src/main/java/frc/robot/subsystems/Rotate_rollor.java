
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.motorConstants;  
import com.revrobotics.Spark.SparkMax;
import com.revrobotics.RelativeEncoder;

public class Rotate_roller extends SubsystemBase {
 Public rotate(){} 
    private spark intake_rotate =new Spark(motorConstants.InmotorR);

public void Rotate_up(){
    intake_rotate.set(1)
}
@Override
public void stay(){
    intake_rotate.set(0)
}
public void rotate_down(){
    intake_rotate.set(-1)
}
@Override
   public void periodic() {
     // This method will be called once per scheduler run
   }
 
   @Override
   public void simulationPeriodic() {
     // This method will be called once per scheduler run during simulation
   }
}

