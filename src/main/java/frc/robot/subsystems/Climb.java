package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;

import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.Constants.motorConstants;


public class Climb extends SubsystemBase {
  private PWMVictorSPX leftClimb = new PWMVictorSPX(motorConstants.CmotorL);
   private PWMVictorSPX rightClimb = new PWMVictorSPX(motorConstants.CmotorR);
   Encoder Left = new Encoder(4,5);
   Encoder Right = new Encoder(6,7,true);
  
  
   Counter RightTilt = new Counter(3);
   Counter LeftTilt = new Counter(4);
  public Climb(){
    rightClimb.setInverted(true);
   
  }
  
   
       
   // private Encoder CLE = new Encoder(0,1,false,Encoder.RelativeEncoder.k2X);
  
  public void climb(){
    if((Right.getDistance()<200)&&(Left.getDistance()<200)){
   leftClimb.set(0.5);
   rightClimb.set(0.5);
  }
  else{
    stop();
  }
  }
  //turn up speed for the final product
  public void LetGo(){
    if (!((Left.getDistance()<=0)||(Right.getDistance()<=0))){
    leftClimb.set(-0.3);
    rightClimb.set(-0.3);
    }
  else{
    stop();
  }
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
