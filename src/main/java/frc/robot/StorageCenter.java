package frc.robot;

import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rotate_rollor;

import java.io.IOException;
import java.io.Serializable;
//template for storage of encoder outputs
public class StorageCenter implements Serializable {
private static final long key = 1042L;
//stored encoder values
private double elvtrVal;
private double ClmbValL;
private double ClmbValR;
private double RWVal;
   StorageCenter(){
    //sets encoder values
    elvtrVal = Elevator.place.getDistance();
    ClmbValL = Climb.LeftE.getDistance();
    ClmbValR = Climb.RightE.getDistance();
    RWVal = Rotate_rollor.TiltR.getDistance();
   }
   //selection zone for what past encoder value is needed for requests of values ranging from 1-4
  public double Get(int request){
    if(request== 1){
        return elvtrVal;
    }
    if(request== 2){
        return ClmbValL;
    }
    if(request== 3){
        return ClmbValR;
    }
    if(request== 4){
        return RWVal;
    }
  return 0;
   }
    //private void output(java.io.ObjectOutputStream Guak) throws IOException{
      //  Guak.defaultWriteObject();
   // }
   // private void search(java.io.ObjectInputStream Guak) throws IOException, ClassNotFoundException {
    //   Guak.defaultReadObject();
   // }
}
