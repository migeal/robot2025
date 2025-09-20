// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ControlSystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;


public class DriveTrain extends SubsystemBase {
  
  /** Creates a new Drive Train Subsystem. */

  private final Translation2d m_frontLeftLocation = new Translation2d(DriveConstants.WheelXdist, DriveConstants.WheelYdist);
  private final Translation2d m_frontRightLocation = new Translation2d(DriveConstants.WheelXdist, -DriveConstants.WheelYdist);
  private final Translation2d m_backLeftLocation = new Translation2d(-DriveConstants.WheelXdist, DriveConstants.WheelYdist);
  private final Translation2d m_backRightLocation = new Translation2d(-DriveConstants.WheelXdist, -DriveConstants.WheelYdist);

  private final SwerveModule m_frontLeft= new SwerveModule(
    ControlSystem.kLeftFrontDrive,
    ControlSystem.kLeftFrontTurn, 
    ControlSystem.kLFturn, 
    DriveConstants.kFrontLeftModuleAngularOffset);
    //DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
    ControlSystem.kRightFrontDrive,
    ControlSystem.kRightFrontTurn, 
    ControlSystem.kRFturn,
    DriveConstants.kFrontRightModuleAngularOffset);
    //DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_backLeft = new SwerveModule(
    ControlSystem.kLeftBackDrive,
    ControlSystem.kLeftBackTurn, 
    ControlSystem.kLBturn,
    DriveConstants.kBackLeftModuleAngularOffset);
    //DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_backRight = new SwerveModule(
    ControlSystem.kRightBackDrive,
    ControlSystem.kRightBackTurn, 
    ControlSystem.kRBturn,
    DriveConstants.kBackRightModuleAngularOffset);
    //DriveConstants.kBackRightChassisAngularOffset);


  //private final ADIS16448_IMU m_imu = new ADIS16448_IMU();
  private final AHRS m_imu = new AHRS(SPI.Port.kMXP);
  
    /***********************************************************************
     * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
     * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
     * 
     * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
     * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
     * 
     * VMX-pi: - Communication via USB. - See
     * https://vmx-pi.kauailabs.com/installation/roborio-installation/
     * 
     * Multiple navX-model devices on a single robot are supported.
     ************************************************************************/
    
  
  private final SwerveDriveKinematics m_kinematics =
    new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    
        
  private final SwerveDrivePoseEstimator m_odometry =
    new SwerveDrivePoseEstimator(
      m_kinematics,
      new Rotation2d(m_imu.getAngle()),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      },
      new Pose2d()
      );

  public DriveTrain() {}
  

  @Override
  public void periodic() {

   // LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults("limelight");
    // update odometry
    m_odometry.update(
        Rotation2d.fromDegrees(m_imu.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
    
   /*  LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (limelightMeasurement != null){
      if (limelightMeasurement.tagCount >= 2) {
        m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        m_odometry.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds
        );
      } 
    } */
   
    //boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    //if(useMegaTag2 == false)
        //System.out.println("Limelight code run");
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if(mt1 != null){
        //System.out.println("mt1 not null");
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        //System.out.println("mt1 == 0");
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        //System.out.println("Update successful");
        m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_odometry.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
   /* 
    else if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight", m_odometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(m_imu.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_odometry.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }    
    */
    // Put values to SmartDashboard 
    SmartDashboard.putNumber("Front Left Drive Speed", DriveVelFL());
    SmartDashboard.putNumber("Front Right Drive Speed", DriveVelFR());
    SmartDashboard.putNumber("Back Left Drive Speed", DriveVelBL());
    SmartDashboard.putNumber("Back Right Drive Speed", DriveVelBR());

    //Display Odometry IMU angle
    SmartDashboard.putNumber("Odometry Angle", getOdometryAngle());
    
    //Display Kinematics
    SmartDashboard.putNumber("Front Left Encoder Count", TurnCountFL());
    SmartDashboard.putNumber("Back Left Encoder Count", TurnCountBL());
    SmartDashboard.putNumber("Front Right Encoder Count", TurnCountFR());
    SmartDashboard.putNumber("Back Right Encoder Count", TurnCountBR());

     //Display Wheel orientations
     SmartDashboard.putNumber("FL Wheel Angle", wheelAngleFL());
     SmartDashboard.putNumber("FR Wheel Angle", wheelAngleFR());
     SmartDashboard.putNumber("BL Wheel Angle", wheelAngleBL());
     SmartDashboard.putNumber("BR Wheel Angle", wheelAngleBR());
      
     //Display Wheel orientations
     SmartDashboard.putNumber("FL NEO Wheel Angle", wheelAngleNEOFL());
     SmartDashboard.putNumber("FR NEO Wheel Angle", wheelAngleNEOFR());
     SmartDashboard.putNumber("BL NEO Wheel Angle", wheelAngleNEOBL());
     SmartDashboard.putNumber("BR NEO Wheel Angle", wheelAngleNEOBR());
  }

  public final double getOdometryAngle() {
    //System.out.printf("Odo Angle Call %f\n", m_imu.getAngle());
    double iMUAngle = m_imu.getAngle();
    return iMUAngle;
  } 
  
  /**
   * Method to drive the robot using the Joystick.
   * 
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param zRot Angular rotation of the robot by twisting the Joystick.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double zRot, boolean fieldRelative) {
    
    double xSpeedCommanded = xSpeed;
    double ySpeedCommanded = ySpeed;
    double m_currentRotation = zRot;


    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeed;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeed;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;
    
    SmartDashboard.putNumber("X Speed", xSpeed);
    SmartDashboard.putNumber("Y Speed", ySpeed);
    SmartDashboard.putNumber("Z Rot ", zRot);
    SmartDashboard.putBoolean("Field Relative ", fieldRelative);
    //if(xSpeed + ySpeed != 0) {System.out.printf("Field %b, x=%f, y=%f, rot=%f\n", fieldRelative, xSpeed, ySpeed, zRot);}

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_imu.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeed);
    
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]); 

    //System.out.printf("Module state 0 output %f", m_frontLeft.getPosition().angle.getDegrees());
    //System.out.printf("Module state 0 calc%f \n", swerveModuleStates[0].angle.getDegrees());
  }
    
  public void resetEncoders() {
    //m_frontLeft.resetEncoders();
    //m_backLeft.resetEncoders();
    //m_frontRight.resetEncoders();
    //m_backRight.resetEncoders();
  }

  // Measure turing encoder counts
  public double TurnCountFR() {
    double turningOut = m_frontRight.TurnOutput();
    return turningOut;
  }

  public double TurnCountFL() {
    double turningOut = m_frontLeft.TurnOutput();
    return turningOut;
  }

  public double TurnCountBR() {
    double turningOut = m_backRight.TurnOutput();
    return turningOut;
  }

  public double TurnCountBL() {
    double turningOut = m_backLeft.TurnOutput();
    return turningOut;
  }

  // Measure driving wheel speeds
  public double DriveVelFL() {
    double driveVelFL = m_frontLeft.DriveOutput();
    return driveVelFL;
  }

  public double DriveVelFR() {
    double driveVelFR = m_frontRight.DriveOutput();
    return driveVelFR;
  }

  public double DriveVelBL() {
    double driveVelBL = m_backLeft.DriveOutput();
    return driveVelBL;
  }

  public double DriveVelBR() {
    double driveVelBR = m_backRight.DriveOutput();
    return driveVelBR;
  }

  // Calculate wheel angles
  public double wheelAngleFL() {
    double angle = m_frontLeft.wheelAngle();
    return angle;
  }
  public double wheelAngleFR() {
    double angle = m_frontRight.wheelAngle();
    return angle;
  }
  public double wheelAngleBL() {
    double angle = m_backLeft.wheelAngle();
    return angle;
  }
  public double wheelAngleBR() {
    double angle = m_backRight.wheelAngle();
    return angle;
  }

  // Calculate wheel angles
  public double wheelAngleNEOFL() {
    double angle = m_frontLeft.getTurnAngle();
    return angle;
  }
  public double wheelAngleNEOFR() {
    double angle = m_frontRight.getTurnAngle();
    return angle;
  }
  public double wheelAngleNEOBL() {
    double angle = m_backLeft.getTurnAngle();
    return angle;
  }
  public double wheelAngleNEOBR() {
    double angle = m_backRight.getTurnAngle();
    return angle;
  }

  public double Distance() {
    var Distance = m_backLeft.distance();
    return Distance;
  }

/*
  public void DriveForward() {
    m_backLeft.DriveForward();
    m_frontLeft.DriveForward();
    m_backRight.DriveForward();
    m_frontRight.DriveForward();
  }
*/
  public void DriveStop() {
    m_backLeft.DriveStop();
    m_frontLeft.DriveStop();
    m_backRight.DriveStop();
    m_frontRight.DriveStop();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
}