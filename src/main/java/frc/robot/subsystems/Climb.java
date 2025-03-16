package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.Transport;

public class Climb extends SubsystemBase {
  private WPI_VictorSPX leftClimb = new WPI_VictorSPX(motorConstants.CmotorL);
   private WPI_VictorSPX rightClimb = new WPI_VictorSPX(motorConstants.CmotorR);
  public static Encoder LeftE = new Encoder(motorConstants.LCA,motorConstants.LCB, true);
  public static Encoder RightE = new Encoder(motorConstants.RCA,motorConstants.RCB);
  double dia = 5*2;
  double dis = (dia*Math.PI/1024)/256;
  
   //Counter RightTilt = new Counter(3);
   //Counter LeftTilt = new Counter(4);
  public Climb(){
    leftClimb.setInverted(true);
    RightE.setDistancePerPulse(dis);
    LeftE.setDistancePerPulse(dis);
  }
  
   
       
   // private Encoder CLE = new Encoder(0,1,false,Encoder.RelativeEncoder.k2X);
  
  public void climb(){
    if((RightE.getDistance()<11.78)&&(LeftE.getDistance()<11.78)){
   leftClimb.set(1);
   rightClimb.set(1);
  }
  else{
    stop();
  }
  }
  //turn up speed for the final product
  public void LetGo(){
    //if ((LeftE.getDistance()>0)&&(RightE.getDistance()>0)){
    leftClimb.set(-1);
    rightClimb.set(-1);
    //}
  //else{
    //stop();
  //}
  }
  // Limit break control
 public void LBclimb(){
  leftClimb.set(0.5);
  rightClimb.set(0.5);
 }

  public void LBLetGo(){
    leftClimb.set(-0.5);
    rightClimb.set(-0.5);
  }
  public void Reset(){
    double goalL= Transport.Lastsave(2); 
    double goalR= Transport.Lastsave(3);
   double ProgL = goalL+LeftE.getDistance();
   double ProgR = goalR+RightE.getDistance();
   while ((ProgL !=0)&&(ProgR !=0)){
    ProgL = goalL+LeftE.getDistance();
    ProgR = goalR+RightE.getDistance();
   if(ProgL>0){
    LBLetGo();
   }
   else if (ProgL <0){
    LBclimb();
   }
   else{
    stop();
    break;
   }
   }
   stop();
  }
  //@Override
  public void stop(){
    leftClimb.set(0);
   rightClimb.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //if (!(((LeftE.getDistance()>0)&&(RightE.getDistance()>0))&&((RightE.getDistance()<11.78)&&(LeftE.getDistance()<0.11.78)))){
     // leftClimb.set(0);
     // rightClimb.set(0);
    //}
    SmartDashboard.putNumber("Left motor",LeftE.getDistance());
    SmartDashboard.putNumber("Right motor",RightE.getDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
