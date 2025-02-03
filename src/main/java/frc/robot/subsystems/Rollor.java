package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
  
import com.revrobotics.Spark.SparkMax;
import com.revrobotics.RelativeEncoder;

public class Roller extends SubsystemBase {
public Roller(){}
private spark intake = new SparkMax(motorConstants.InmotorL,MotorType.kBrushed);


//grabing the algee and coral 
public void grab(){
    intake.set(0.5);
}
//holding algee and coral
@Override
public void hold(){
    intake.set(0)
}
//relaseing algee 
public void release(){
    intake.set(-0.5) 
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

