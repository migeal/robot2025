// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//commands
import frc.robot.commands.EUp;
import frc.robot.commands.EDown;
import frc.robot.commands.Clamp;
import frc.robot.commands.letGo;
import frc.robot.commands.PistonTog;

//subsystems
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.climbPistons;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.motorConstants;

import edu.wpi.first.wpilibj2.command.RunCommand;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
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
  // joystick 
  private final CommandJoystick m_StickOfHope = new CommandJoystick(0);
  private final Joystick m_ButtonBoard = new Joystick(1);
  private final XboxController m_gamerTime = new XboxController(0);
  //commands
  private final EUp m_EUp = new EUp(m_Elevator);
  private final EDown m_EDown = new EDown(m_Elevator);
  private final Clamp m_Clamp = new Clamp(m_climb);
  private final letGo m_LetGo = new letGo(m_climb);
  private final PistonTog m_PTog = new PistonTog(m_CP);

  //buttons 
   private JoystickButton lock = new JoystickButton(m_ButtonBoard, 8);
   private JoystickButton unlock = new JoystickButton(m_ButtonBoard, 9);
   private JoystickButton Floor1 = new JoystickButton(m_ButtonBoard, 2);
   private JoystickButton Floor2 = new JoystickButton(m_ButtonBoard, 3);
   private JoystickButton Floor3 = new JoystickButton(m_ButtonBoard, 13);
   private JoystickButton Floor4 = new JoystickButton(m_ButtonBoard, 12);
   private JoystickButton ManUp = new JoystickButton(m_ButtonBoard, 5);
   private JoystickButton ManDown = new JoystickButton(m_ButtonBoard, 6);
   private JoystickButton Accention = new JoystickButton(m_ButtonBoard, 1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
 
    // Configure the trigger bindings
    configureBindings();
    //Configure driving default
    m_robotDrive.setDefaultCommand(
      // Forward motion controls x speed (forward), sideways motion controls y speed (sideways).
        new RunCommand (  
          () -> m_robotDrive.drive( 
          m_StickOfHope.getX(),
          m_StickOfHope.getY(), 
          m_StickOfHope.getZ(),
          DriveConstants.kTeleField),m_robotDrive)
               
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
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    lock.whileTrue( m_Clamp);
    unlock.whileTrue(m_LetGo);
    ManUp.whileTrue(m_EUp);
    ManDown.whileTrue(m_EDown);
    Accention.onTrue(m_PTog);
   if (Floor1.getAsBoolean()==true){
    m_Elevator.Hight(2);
   }
   if (Floor2.getAsBoolean()==true){
    m_Elevator.Hight(4);
   }
   if (Floor3.getAsBoolean()==true){
    m_Elevator.Hight(6);
   }
   if (Floor4.getAsBoolean()==true){
    m_Elevator.Hight(8);
   }
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
