package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


import frc.robot.Constants.motorConstants;


public class Climb extends SubsystemBase {
  public Climb(){}
  
   private SparkMax leftClimb = new SparkMax(motorConstants.CmotorL, MotorType.kBrushless);
   private SparkMax rightClimb = new SparkMax(motorConstants.CmotorR, MotorType.kBrushless);
   private DoubleSolenoid accendFR = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
   private DoubleSolenoid accendFL = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
   private DoubleSolenoid accendBR = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
   private DoubleSolenoid accendBL = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
   //rightClimb.setInverted(true);
   // private Encoder CLE = new Encoder(0,1,false,Encoder.RelativeEncoder.k2X);
  public void Accend(){
    accendFR.toggle();
    accendFL.toggle();
    accendBR.toggle();
    accendBL.toggle();
   }
  public void climb(){
   leftClimb.set(0.5);
   rightClimb.set(0.5);
  }
  public void LetGo(){
    leftClimb.set(-0.5);
    rightClimb.set(-0.5);
  }
  //@Override
  public void stop(){
    leftClimb.set(0);
   rightClimb.set(0);
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
