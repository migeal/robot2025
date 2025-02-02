
package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.motorConstants;  
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import com.revrobotics.RelativeEncoder;

public class Rotate_rollor extends SubsystemBase {
 public Rotate_rollor(){} 
    private Victor intake_rotate =new Victor(motorConstants.InmotorR);

public void Rotate_up(){
    intake_rotate.set(1);
}

public void stay(){
    intake_rotate.set(0);
}
public void rotate_down(){
    intake_rotate.set(-1);
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

