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
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.spark.SparkBase;

import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.Constants.motorConstants;


public class Climb extends SubsystemBase {
  private WPI_VictorSPX leftClimb = new WPI_VictorSPX(motorConstants.CmotorL);
   private WPI_VictorSPX rightClimb = new WPI_VictorSPX(motorConstants.CmotorR);
  public static Encoder LeftE = new Encoder(motorConstants.LCA,motorConstants.LCB);
  public static Encoder RightE = new Encoder(motorConstants.RCA,motorConstants.RCB,true);
  double dia = 1*2;
  double dis = dia*3.14159/1024;
  
   //Counter RightTilt = new Counter(3);
   //Counter LeftTilt = new Counter(4);
  public Climb(){
    rightClimb.setInverted(true);
   
  }
  
   
       
   // private Encoder CLE = new Encoder(0,1,false,Encoder.RelativeEncoder.k2X);
  
  public void climb(){
    if((RightE.get()<0.5)&&(LeftE.get()<0.5)){
   leftClimb.set(0.5);
   rightClimb.set(0.5);
  }
  else{
    stop();
  }
  }
  //turn up speed for the final product
  public void LetGo(){
    if ((LeftE.get()>0)&&(RightE.get()>0)){
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
