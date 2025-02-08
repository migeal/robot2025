package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Counter;

import frc.robot.Constants.motorConstants;


public class Elevator extends SubsystemBase {
   private final PWMVictorSPX m_liftMotor = new PWMVictorSPX(motorConstants.Emotor);
   Counter placement = new Counter(2);
   
    
   //Encoder Flor = new Encoder(0,1, false, Encoder.CANcoder.k2x );
    public Elevator(){
      placement.setSemiPeriodMode(true);
    }

     //normal up/down for custom hights
     
    public void up(){ 
      if(placement.get() < 10){
       m_liftMotor.set(0.5);
      }
      else{
        stop();
      }
    }
    public void down(){
      if(placement.get() > 0){
    m_liftMotor.set(-0.5);
    }
    else{
      stop();
    }
    }
    // set elevator to called location, plan to call it directly from RobotContainer
   public void Hight(double level){
    //double start =  Flor.getPosition();
   // double start = 1;
   double start = placement.get();
    double go = level - start;
     while(!(go<0.5)&&!(go>-0.5)){
    start = placement.get();
     go = level - start;
    if (go>0.5){
    up();
    }
    else if(go<-0.5){
      down();
    }
    else{
      stop();
      break;
    }
    }
   }
   
   public void stop(){
    m_liftMotor.set(0);
    
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
