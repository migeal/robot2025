package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.PneumaticsConstants;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class climbPistons extends SubsystemBase{
  private DoubleSolenoid accendFR = new DoubleSolenoid(PneumaticsConstants.k_pcmCANid, PneumaticsModuleType.CTREPCM, 0, 1);
  private DoubleSolenoid accendFL = new DoubleSolenoid(PneumaticsConstants.k_pcmCANid, PneumaticsModuleType.CTREPCM, 2, 3);
  private DoubleSolenoid accendBR = new DoubleSolenoid(PneumaticsConstants.k_pcmCANid, PneumaticsModuleType.CTREPCM, 4, 5);
  
  
public climbPistons(){}

public void Accend(){
    accendFR.toggle();
    accendFL.toggle();
    accendBR.toggle();
   
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
