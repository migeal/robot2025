
package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.motorConstants;  
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Transport;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder; 
import frc.robot.RobotContainer;

public class Rotate_rollor extends SubsystemBase {
  //Counter move= new Counter(1);
 public static Encoder TiltR= new Encoder(motorConstants.WA, motorConstants.WB);
 double dia = 16*2;
 double dis = (dia*Math.PI)/343;
 
  public Rotate_rollor(){
 // move.setSemiPeriodMode(true);
    TiltR.setDistancePerPulse(dis);
    TiltR.reset();
 } 
    private WPI_VictorSPX intake_rotate = new WPI_VictorSPX(motorConstants.WristMotor);
  // double value = move.getPeriod();


public void Rotate_up(){
 if(TiltR.getDistance() <37.6){;
 intake_rotate.set(-1);
 
}
else{
  stay();
}
}


public void stay(){
    intake_rotate.set(0);
    
}
public void rotate_down(){
    
  //if(TiltR.getDistance() <0){;
    intake_rotate.set(1);
    
   //}
   //else{
    // stay();
   //}
   
}
@Override
   public void periodic() {
     // This method will be called once per scheduler run
     SmartDashboard.putNumber("Wrist motor",TiltR.getDistance());
     SmartDashboard.putBoolean("Calibration mode", RobotContainer.Limit);
   }
 
   @Override
   public void simulationPeriodic() {
     // This method will be called once per scheduler run during simulation
    
   }
   public void Rotate(double degree){
    // specifide level finder for Rotate_roller (aka wrist).
   double start = TiltR.getDistance();
    double change = (degree/360)*(dia*Math.PI);
    double go = change - start;
     while(!(go<0.2)&&!(go>-0.2)){
    start = TiltR.getDistance();
     go = change - start;
    if (go>=0.2){
    Rotate_up();
    }
    else if(go<=-0.2){
      rotate_down();
    }
    else{
      stay();
      break;
    }
    }
    stay();
   }
   //limit break movement
   public void LBRotate_Up(){
    intake_rotate.set(-0.5);
   }
   public void LBRotate_Down(){
    intake_rotate.set(0.5);
   }

   public void Reset(){
    double Goal = Transport.Lastsave(4);
    double Progress = Goal+TiltR.getDistance();
    while(Progress!=0){
      Progress = Goal+TiltR.getDistance();
      if(Progress>0){
        rotate_down();
      }
      else if(Progress<0){
        Rotate_up();
      }
      else{
        stay();
        break;
      }
    }
    stay();
   }

  }

