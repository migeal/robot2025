package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.motorConstants;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import com.revrobotics.RelativeEncoder;

public class Rollor extends SubsystemBase {
public Rollor(){}
private Victor intake = new Victor(motorConstants.InmotorL);


//grabing the algee and coral 
public void grab(){
    intake.set(0.5);
}
//holding algee and coral

public void hold(){
    intake.set(0);
}
//relaseing algee 
public void release(){
    intake.set(-0.5); 
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

