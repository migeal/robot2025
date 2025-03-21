// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.print.attribute.standard.MediaSize.NA;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Transport;


//commands
import frc.robot.commands.EUp;
import frc.robot.commands.EDown;
import frc.robot.commands.Clamp;
import frc.robot.commands.letGo;
import frc.robot.commands.PistonTog;
import frc.robot.commands.push_out;
import frc.robot.commands.pull_in;
import frc.robot.commands.rotate_down;
import frc.robot.commands.rotate_up;
import frc.robot.commands.stableizerP_togle;

//subsystems
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.climbPistons;
import frc.robot.subsystems.Rotate_rollor;
import frc.robot.subsystems.Rollor;
import frc.robot.subsystems.stableizerP;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.motorConstants;
import frc.robot.subsystems.calibration;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static Boolean Limit;
  //private static Boolean togg;
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
   
  
  // subsystem
  
  private final DriveTrain m_robotDrive = new DriveTrain();
  private final Elevator m_Elevator = new Elevator();
  private final Climb m_climb = new Climb();
  private final climbPistons m_CP = new climbPistons();
  private final Rollor m_Rollor = new Rollor();
  private final Rotate_rollor m_Rotate_rollor = new Rotate_rollor();
  private final stableizerP m_Stab = new stableizerP();
  private final calibration m_Cal = new calibration();
  // joystick 
 // private final XboxController m_driverController = new XboxController(0);
  //private final CommandJoystick m_StickOfHope = new CommandJoystick(0);
  private final Joystick m_ButtonBoard = new Joystick(1);
 // private final XboxController m_gamerTime = new XboxController(0);
  //commands
  private final EUp m_EUp = new EUp(m_Elevator);
  private final EDown m_EDown = new EDown(m_Elevator);
  private final Clamp m_Clamp = new Clamp(m_climb);
  private final letGo m_LetGo = new letGo(m_climb);
  private final PistonTog m_PTog = new PistonTog(m_CP);
  private final rotate_up m_rotate_up = new rotate_up(m_Rotate_rollor);
  private final rotate_down m_rotate_down = new rotate_down(m_Rotate_rollor);
private final push_out m_push_out =new push_out(m_Rollor);
private final pull_in m_pull_in = new pull_in(m_Rollor);
private final stableizerP_togle m_stab = new stableizerP_togle(m_Stab);
  //buttons
   //private JoystickButton lock = new JoystickButton(m_ButtonBoard, 8);
   //private JoystickButton unlock = new JoystickButton(m_ButtonBoard, 9);
   //elevator
   private JoystickButton Floor1 = new JoystickButton(m_ButtonBoard, 8);
   private JoystickButton Floor2 = new JoystickButton(m_ButtonBoard, 1);
   private JoystickButton Floor3 = new JoystickButton(m_ButtonBoard, 2);
   private JoystickButton Floor4 = new JoystickButton(m_ButtonBoard, 13);
   private JoystickButton ManUp = new JoystickButton(m_ButtonBoard, 12);
   private JoystickButton ManDown = new JoystickButton(m_ButtonBoard, 3);
   //private JoystickButton Accention = new JoystickButton(m_ButtonBoard, 1);
   //intake
  private JoystickButton up =new JoystickButton(m_ButtonBoard, 4);
  private JoystickButton down =new JoystickButton(m_ButtonBoard, 7);
  private JoystickButton tiltu =new JoystickButton(m_ButtonBoard, 5);
  private JoystickButton tiltm =new JoystickButton(m_ButtonBoard, 6);
  private JoystickButton tiltd =new JoystickButton(m_ButtonBoard, 9);


  /** The conta iner for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Limit= false;
   // togg = false;
    // Configure the trigger bindings
    configureBindings();
    //Configure driving default
    /*
    m_robotDrive.setDefaultCommand(
      // Forward motion controls x speed (forward), sideways motion controls y speed (sideways).
        new RunCommand (  
          () -> m_robotDrive.drive(
            -MathUtil.applyDeadband(m_StickOfHope.getY(), DriveConstants.kDriveDeadband),
            -MathUtil.applyDeadband(-m_StickOfHope.getX(), DriveConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_StickOfHope.getZ(), DriveConstants.kDriveDeadbandZ),
            DriveConstants.kTeleField), m_robotDrive)
               
        );
      // Configure the trigger bindings
      configureBindings();*/
      //Configure driving default
      m_robotDrive.setDefaultCommand(
        // Forward motion controls x speed (forward), sideways motion controls y speed (sideways).
          new RunCommand (  
            () -> m_robotDrive.drive(
              -MathUtil.applyDeadband(-m_driverController.getLeftY(), DriveConstants.kDriveDeadband),
              -MathUtil.applyDeadband(-m_driverController.getLeftX(), DriveConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getRightX(), DriveConstants.kDriveDeadbandZ),
              DriveConstants.kTeleField), m_robotDrive)
                 
          );
  }
  
 
 
   
  
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
   //double right_Axis = m_Controly.getRightTriggerAxis();
   
    //if(right_Axis>0.5){
      
   // };
   m_driverController.y().toggleOnTrue(new StartEndCommand(m_Stab::out, m_Stab::in, m_Stab));
   if (Limit==false){
    m_driverController.a().toggleOnTrue(new StartEndCommand(m_CP::up, m_CP::down, m_CP));
   }
   else if(m_driverController.a().getAsBoolean()==true){
       m_climb.Reset();
   }
   m_driverController.rightTrigger().whileTrue(m_push_out);
   if(m_driverController.rightTrigger().getAsBoolean()){
    m_driverController.setRumble(GenericHID.RumbleType.kRightRumble, 1);
   }
   else{
    m_driverController.setRumble(GenericHID.RumbleType.kRightRumble, 0);
   }
   m_driverController.leftTrigger().whileTrue(m_pull_in);
   if(m_driverController.leftTrigger().getAsBoolean()){
    m_driverController.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
   }
   else{
    m_driverController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
   }
   up.whileTrue(m_rotate_up);
   down.whileTrue(m_rotate_down);
    m_driverController.x().whileTrue( m_Clamp);
    m_driverController.b().whileTrue(m_LetGo);
    ManUp.whileTrue(m_EUp);
    ManDown.whileTrue(m_EDown);
   // Accention.onTrue(m_PTog);
   if(Limit==false){
   tiltu.onTrue(new StartEndCommand(m_Rotate_rollor::hieR,m_Rotate_rollor::stay, m_Rotate_rollor));

   tiltm.onTrue(new StartEndCommand(m_Rotate_rollor::midR,m_Rotate_rollor::stay, m_Rotate_rollor));

   tiltd.onTrue(new StartEndCommand(m_Rotate_rollor::lowR,m_Rotate_rollor::stay, m_Rotate_rollor));
   /* if (tiltm.getAsBoolean()==true){
    m_Rotate_rollor.Rotate(90);
   }
   else if (tiltd.getAsBoolean()==true){
    m_Rotate_rollor.Rotate(45);
   } 
  }
  else if(Limit==true){
   if(tiltd.getAsBoolean()==true){
    m_Rotate_rollor.Reset();
  }
    */
  }
   //call differant hights, if the limit is true then Floor1 is the only one enabled and changed to reset
   if(Limit==false){

   Floor1.onTrue(new StartEndCommand(m_Elevator::DownH, m_Elevator::stop, m_Elevator));

   Floor2.onTrue(new StartEndCommand(m_Elevator::LowH, m_Elevator::stop, m_Elevator));

   Floor3.onTrue(new StartEndCommand(m_Elevator::medH, m_Elevator::stop, m_Elevator));

   Floor4.onTrue(new StartEndCommand(m_Elevator::HieH, m_Elevator::stop, m_Elevator));
   
  }
   if(Limit==true){
    Floor1.onTrue(new StartEndCommand(m_Elevator::Reset, m_Elevator::stop, m_Elevator));
 }
 m_driverController.back().toggleOnTrue(new StartEndCommand(m_Cal::activate, m_Cal::normal, m_Cal));
   /*if(m_driverController.back().getAsBoolean()==true){
     //revearsed ideas of true and false
    if(togg==false){
        if(Limit == false){
          Limit=true;
        }
        else{
          Limit = false;
        }
        togg=true;
      }
    }
     else if(m_driverController.back().getAsBoolean()==false){
        togg=false;
      }
      */
       
      
   
   // To save values for relitave encoders at the end of a match
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
