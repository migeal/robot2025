package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;



import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class stableizerP extends SubsystemBase{
    private DoubleSolenoid fourth = new DoubleSolenoid(2,PneumaticsModuleType.CTREPCM, 6, 7);
   
  
  
public stableizerP(){}
//better than InnOut, toggle for the stablization piston.
public void OutnIn(){
    fourth.toggle();
   
   
   }
  public void out(){
    fourth.set(Value.kForward);
  }
  public void in (){
    fourth.set(Value.kReverse);
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
