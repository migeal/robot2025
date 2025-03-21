package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Transport;
import frc.robot.Constants.motorConstants;


public class Elevator extends SubsystemBase {
   private final WPI_VictorSPX m_liftMotor = new WPI_VictorSPX(motorConstants.Emotor);
   //Counter placement = new Counter(1);
  public static Encoder place = new Encoder(motorConstants.ElvateA,motorConstants.ElvateB);
    double dia = 0.75;
    double dis = (dia*Math.PI/1024)/72;
    //10 times to top
   //Encoder Flor = new Encoder(0,1, false, Encoder.CANcoder.k2x );
    public Elevator(){
      //placement.setSemiPeriodMode(true);
      place.setDistancePerPulse(dis);
    }

     //normal up/down for custom hights
     //18.85
    public void up(){ 
      if(place.getDistance() < 18){
       m_liftMotor.set(0.7);
      }
      else{
        stop();
      }
    }
    public void down(){
      //if(place.getDistance() > 0){
    m_liftMotor.set(-0.7);
    //}
    //else{
     // stop();
   // }
    }
    public void DownH(){
      Hight(0);
    }
    public void LowH(){
      Hight(3.7);
    }
    public void medH(){
      Hight(4.11);
    }
    public void HieH(){
     Hight(13);
    }
    // set elevator to called location, plan to call it directly from RobotContainer
   public void Hight(double level){
    //double start =  Flor.getPosition();
   // double start = 1;
   double start = place.getDistance();
    double go = level - start;
     while(!(go<0.2)&&!(go>-0.2)){
    start = place.getDistance();
     go = level - start;
    if (go>=0.2){
    up();
    }
    else if(go<=-0.2){
      down();
    }
    else{
      stop();
      break;
    }
    }
   }
   public void LBup(){
    m_liftMotor.set(0.5);
   }
   public void LBdown(){
    m_liftMotor.set(-0.5);
   }
   public void stop(){
    m_liftMotor.set(0);
   
    
   }
   public void Reset(){
    double goal = Transport.Lastsave(1);
    double progress = goal+place.getDistance();
    while(progress!=0){
      progress=goal+place.getDistance();
      if(progress>0){
        LBdown();
      }
      else if(progress<0){
        LBup();
      }
      else{
        stop();
        break;
      }
    }
    stop();
   }

   @Override
   public void periodic() {
     // This method will be called once per scheduler run
     SmartDashboard.putNumber("Elevator motor",place.getDistance());
   }
 
   @Override
   public void simulationPeriodic() {
     // This method will be called once per scheduler run during simulation
   }
    
}
