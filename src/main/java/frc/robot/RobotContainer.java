/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GearShift;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.MoveTowerBall;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DrivetrainSubsystem m_DrivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final TowerSubsystem m_TheRisenOne= new TowerSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  XboxController Controller1 = new XboxController(0);
  XboxController Controller2 = new XboxController(1);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //Add default commands here
    m_DrivetrainSubsystem.setDefaultCommand(
        new RunCommand(() -> m_DrivetrainSubsystem
          .arcadeDrive(Controller1.getY(GenericHID.Hand.kLeft),
              Controller1.getX(GenericHID.Hand.kRight)), m_DrivetrainSubsystem));
              


    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton (Controller1, Button.kBumperRight.value)
      .whenPressed(() -> m_DrivetrainSubsystem.setMaxOutput(Constants.DriveConstants.kLowSpeedRatio))
      .whenReleased(() -> m_DrivetrainSubsystem.setMaxOutput(Constants.DriveConstants.kHighSpeedRatio));

    //Intake Commands
    new JoystickButton (Controller1, Button.kBumperLeft.value)
      .whenPressed(new IntakeIn(m_IntakeSubsystem));
      

      new JoystickButton (Controller1, Button.kX.value)
      .whenHeld(new GearShift(m_DrivetrainSubsystem));

    //Tower Commands
    new JoystickButton (Controller1, Button.kY.value)
    .whenHeld(new MoveTowerBall(m_TheRisenOne));
      
    



  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
