package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


import edu.wpi.first.wpilibj.DoubleSolenoid;

public class rPiston extends SubsystemBase{
    private DoubleSolenoid fourth = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
   
  
  
public rPiston(){}
//better than InnOut, toggle for this random piston that is here I guess.
public void OutnIn(){
    fourth.toggle();
   
   
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
