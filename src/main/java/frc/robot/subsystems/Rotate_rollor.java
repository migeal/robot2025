
package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.motorConstants;  
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder; 

public class Rotate_rollor extends SubsystemBase {
  Counter move= new Counter(1);
  Encoder Move= new Encoder(motorConstants.WA, motorConstants.WB);
  public Rotate_rollor(){
  move.setSemiPeriodMode(true);
 } 
    private PWMVictorSPX intake_rotate = new PWMVictorSPX(motorConstants.InmotorR);
   double value = move.getPeriod();


public void Rotate_up(){
 if(Move.getDistance() <10){;
 intake_rotate.set(1);
}
else{
  stay();
}
}


public void stay(){
    intake_rotate.set(0);
}
public void rotate_down(){
    
  if(Move.getDistance() >-1){;
    intake_rotate.set(-1);
   }
   else{
     stay();
   }
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

