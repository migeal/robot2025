package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Compress extends SubsystemBase {
Compress(){}
private final Compressor supply = new Compressor(PneumaticsModuleType.CTREPCM);
}
