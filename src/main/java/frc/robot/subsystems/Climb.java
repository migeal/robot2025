package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;

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
  private SparkMax leftClimb = new SparkMax(motorConstants.CmotorL, MotorType.kBrushless);
   private SparkMax rightClimb = new SparkMax(motorConstants.CmotorR, MotorType.kBrushless);
  
   private SparkClosedLoopController Left = leftClimb.getClosedLoopController();
   private SparkClosedLoopController right = rightClimb.getClosedLoopController();
   SparkMaxConfig setC = new SparkMaxConfig();
  public Climb(){
    setC.inverted(true).idleMode(IdleMode.kBrake);
    rightClimb.configure(setC, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
   
       
   // private Encoder CLE = new Encoder(0,1,false,Encoder.RelativeEncoder.k2X);
  
  public void climb(){
    Left.setReference(1, ControlType.kPosition);
    right.setReference(1, ControlType.kPosition);
   //leftClimb.set(0.5);
   //rightClimb.set(0.5);
  
  }
  //turn up speed for the final product
  public void LetGo(){
    if (!((leftClimb.getEncoder().getPosition()<=0)||(rightClimb.getEncoder().getPosition()<=0)))
    leftClimb.set(-0.3);
    rightClimb.set(-0.3);
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
