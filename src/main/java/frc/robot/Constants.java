// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants {
    public static final double gearRatio1st = 14.0/50.0;
    public static final double gearRatio2nd = 27.0/17.0;
    public static final double gearRatio3rd = 15.0/45.0;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); // meters

    public static final double kDrivingEncoderPositionFactor = kWheelDiameterMeters/(gearRatio1st*gearRatio2nd*gearRatio3rd);
    public static final double kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor/60.0; // meters per second

    public static final double kturnGearRatio = 150.0/7.0;
    //public static final double kTurningEncoderPositionFactor = 1*(kturnGearRatio)/1.25; // position in degrees  //(2 * Math.PI) / kturnGearRatio; //position in radians
    public static final double kTurningEncoderPositionFactor = (Math.PI*2)/(kturnGearRatio); //position in radians
    public static final double kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor/60.0; // meters per second
    
    public static final double driveGainP = 0.0005;
    public static final double driveGainI = 0.00005;
    public static final double driveGainD = 0.0035;

    public static final double turnGainP = 0.5;
    public static final double turnGainI = 0.00001;
    public static final double turnGainD = 0.01;

    public static final double kAngleEncoderResolution = 42;
    public static final boolean kTurningEncoderInverted = true;
   // public static final double kTurningMinOutput = -0.1;
    //public static final double kTurningMaxOutput = 0.1;
    public static final double kTurningEncoderPositionPIDMinInput = -1;
    public static final double kTurningEncoderPositionPIDMaxInput = 1;
  }

  public static class DriveConstants {
    public static final double robotWidth = Units.inchesToMeters(37.0);
    public static final double robotLength = Units.inchesToMeters(37.0);
    public static final double WheelYdist = robotLength*0.5;
    public static final double WheelXdist = robotWidth*0.5;

    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = 0;

    /*public static final SwerveDriveKinematics kswerveDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(WheelXdist, WheelYdist),
      new Translation2d(WheelXdist, -WheelYdist),
      new Translation2d(-WheelXdist, WheelYdist),
      new Translation2d(-WheelXdist, -WheelYdist));*/

    // encoder angluar offset 
    public static final double kFrontLeftModuleAngularOffset = 0.295654;//*360;//Math.PI*2; 
    public static final double kFrontRightModuleAngularOffset = -.463135;//*360;//Math.PI*2;
    public static final double kBackLeftModuleAngularOffset = -0.054932;//*360;//Math.PI*2;
    public static final double kBackRightModuleAngularOffset = 0.187988;//*360;//Math.PI*2;
 
    public static final double kChassisAngularOffset = 0;

    public static final double kMaxSpeed = 40;
    public static final double kMaxAngularSpeed = 100;
    public static final double kDriveDeadband = 0.05;
    public static final double kDriveDeadbandZ = 0.1;
    public static final boolean kTeleField = true;
  }

  public static class ControlSystem {
    // Driving motor CAN IDs
    public static final int kLeftFrontDrive = 12;
    public static final int kLeftBackDrive = 17;
    public static final int kRightFrontDrive = 3;
    public static final int kRightBackDrive = 2;
    // Turning motors
    public static final int kLeftFrontTurn = 8;
    public static final int kLeftBackTurn = 11;
    public static final int kRightFrontTurn = 7;
    public static final int kRightBackTurn = 4;
    // CANCoder Can IDs for turning encoders
    public static final int kLFturn = 13;
    public static final int kLBturn = 16;
    public static final int kRFturn = 14;
    public static final int kRBturn = 15;
  }

  public static class motorConstants{
    // motors for subsystems CAN IDs
    public static final int CmotorL = 6;
    public static final int CmotorR = 1;
    public static final int Emotor = 20;
    public static final int WristMotor= 5;
    public static final int IntakeMotor= 10;
    // AM encoder DIO's
    public static final int ElvateA = 0;
    public static final int ElvateB = 1;
    public static final int WA = 2;
    public static final int WB = 3;
    public static final int LCA = 4;
    public static final int LCB = 5;
    public static final int RCA = 6;
    public static final int RCB = 7;
  }

  public static class PneumaticsConstants{
    public static final int k_pcmCANid = 2;
  }
  // Vendor Dep URLS
  // nav-x: "https://storage.googleapis.com/frc2025/NavX2025.json"
  // phoenix5: "https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2025-latest.json"
  // phoenix6: "https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2025-latest.json"
  // revrobotics: "https://software-metadata.revrobotics.com/REVLib-2025.json"
  
}
