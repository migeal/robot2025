package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


import edu.wpi.first.wpilibj.DoubleSolenoid;

public class stableizerP extends SubsystemBase{
    private DoubleSolenoid fourth = new DoubleSolenoid(PneumaticsConstants.k_pcmCANid, PneumaticsModuleType.CTREPCM, 6, 7);
   
  
  
public stableizerP(){}
//better than InnOut, toggle for the stablization piston.
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
