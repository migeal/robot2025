package frc.robot;

import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rotate_rollor;

import java.io.IOException;
import java.io.Serializable;

public class StorageCenter implements Serializable {
private static final long key = 1042L;
private double elvtrVal;
private double ClbValL;
private double ClbValR;
private double RRVal;
   StorageCenter(){
    elvtrVal = Elevator.place.getDistance();
    ClbValL = Climb.LeftE.getDistance();
    ClbValR = Climb.RightE.getDistance();
    RRVal = Rotate_rollor.TiltR.getDistance();
   }
    private void output(java.io.ObjectOutputStream Guak) throws IOException{
        Guak.defaultWriteObject();
    }
    private void search(java.io.ObjectInputStream Guak) throws IOException, ClassNotFoundException {
       Guak.defaultReadObject();
    }
}
