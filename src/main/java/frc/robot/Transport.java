package frc.robot;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputFilter;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import frc.robot.StorageCenter;
public  class Transport {
    
    public static void Go(){
StorageCenter QSave = new StorageCenter();
// attempts to save a new StorageCenter object to Storage
try{
    //file chosen
FileOutputStream repairIn = new FileOutputStream("Storage.txt");
// object to be stored
ObjectOutputStream encOut = new ObjectOutputStream(repairIn);
// storeing the StorageCenter object
encOut.writeObject(QSave);
encOut.close();
repairIn.close();
System.out.println("saving is functional");
}catch(IOException sad){
    sad.printStackTrace();
}
}
public static double Lastsave(int wantedData){
    /*Deseiralisation command, needs int for selection and also the data needs 
    to remain as a Storage center to avoid errors, 
    uses catches to avoid crashing*/
    try{
    FileInputStream repairOut = new FileInputStream("Storage.txt");
    ObjectInputStream saved = new ObjectInputStream(repairOut);
    StorageCenter g = (StorageCenter) saved.readObject();
     saved.close();
     repairOut.close();
    return g.Get(wantedData);
    }catch(IOException memoryLoss  ){
     memoryLoss.printStackTrace();
    }catch(ClassNotFoundException waldo){
        waldo.printStackTrace();
    }
    
    return 1;
}
}
