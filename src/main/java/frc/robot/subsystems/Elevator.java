package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.motorConstants;


public class Elevator extends SubsystemBase {
   private final SparkMax m_liftMotor = new SparkMax(motorConstants.Emotor, MotorType.kBrushless);
   SparkClosedLoopController level = m_liftMotor.getClosedLoopController();
   double start = m_liftMotor.getEncoder().getPosition();
    
   //Encoder Flor = new Encoder(0,1, false, Encoder.CANcoder.k2x );
    public Elevator(){}

     //normal up/down for custom hights
     
    public void up(){ 
     

       m_liftMotor.set(0.5);

    }

    public void down(){
    m_liftMotor.set(-0.5);
    }
    // set elevator to called location
  
   public void Hight(double level){
    //double start =  Flor.getPosition();
   // double start = 1;
    double go = level - start;
    if (go>0){
      
    }
   }
   //@Override
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
