package frc.robot;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import frc.robot.StorageCenter;
public class Transport {
    public static void Go(){
StorageCenter QSave = new StorageCenter();

try{
FileOutputStream repairIn = new FileOutputStream("StorageCenter.Guak");
ObjectOutputStream encOut = new ObjectOutputStream(repairIn);
encOut.writeObject(QSave);
encOut.close();
repairIn.close();
System.out.println("saving is functional");
}catch(IOException sad){
    sad.printStackTrace();
}
}
}
