package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
public class calibration extends SubsystemBase {
public calibration(){}
public void activate (){
  RobotContainer.Limit=true;
}
public void normal (){
    RobotContainer.Limit=false;
}
}
