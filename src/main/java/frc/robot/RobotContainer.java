/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/*

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;




import com.qualcomm.robotcore.eventloop.opmode.Disabled;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;







import org.firstinspires.ftc.robotcore.external.Telemetry;


import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;
*/
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoFarLeftShoot5;
import frc.robot.commands.AutoMidDefense;
import frc.robot.commands.AutoMidShoot;
import frc.robot.commands.AutoReverse;
import frc.robot.commands.AutoShoot3Position;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.GearShift;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.RaiseElevator;
import frc.robot.commands.ShootBall;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
  private final DrivetrainSubsystem m_DrivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem m_TheClimb= new ClimberSubsystem();
  private final SendableChooser<Command> auto = new SendableChooser<Command>();
  private final SendableChooser<String> autou = new SendableChooser<String>();


  XboxController Controller1 = new XboxController(0);
  XboxController Controller2 = new XboxController(1);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //Add default commands here
    m_DrivetrainSubsystem.setDefaultCommand(new RunCommand(() -> m_DrivetrainSubsystem.arcadeDrive(
            Controller1.getY(GenericHID.Hand.kLeft),
            -Controller1.getX(GenericHID.Hand.kRight)), m_DrivetrainSubsystem));
              
    // Configure the button bindings
    configureButtonBindings();

    //Autonomous procedures
    auto.addOption("AutoDrive", new AutoDrive(0.5, m_DrivetrainSubsystem));
    auto.addOption("FarLeftShoot 5 ", new AutoFarLeftShoot5(m_DrivetrainSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_VisionSubsystem));
    auto.addOption("Mid Shoot", new AutoMidShoot(m_DrivetrainSubsystem, m_ShooterSubsystem, m_IntakeSubsystem));
    auto.addOption("Reverse", new AutoReverse(m_DrivetrainSubsystem));
    auto.addOption("Shoot 3 then position", new AutoShoot3Position(m_DrivetrainSubsystem, m_ShooterSubsystem, m_IntakeSubsystem));
    auto.addOption("Auto Turn", new AutoTurn(0, m_DrivetrainSubsystem));
    auto.addOption("Straight mid block shooters", new AutoMidDefense(m_DrivetrainSubsystem));
    //Autonomous positions
    autou.addOption("Middle", "Middle");
    //middle, mid l/r, far l/r

    SmartDashboard.putData("Auto Chooser", auto);
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

    //Drivetrain Commands
    new JoystickButton (Controller1, Button.kX.value)
    .whenHeld(new GearShift(m_DrivetrainSubsystem));
      
    //Intake Commands (go out )
    new JoystickButton (Controller1, Button.kBumperLeft.value)
      .whenHeld(new IntakeIn(m_IntakeSubsystem));
      
    //Tower Commands
    new JoystickButton (Controller1, Button.kY.value)
    .whenPressed(() -> m_IntakeSubsystem.conveyorMotorOn(1))
    .whenReleased(() -> m_IntakeSubsystem.conveyorMotorOff());

    //Shooter Commands
    new JoystickButton (Controller1, Button.kA.value)
    .whenHeld(new ShootBall(m_ShooterSubsystem,m_IntakeSubsystem));
      
    //Climber Commands
    new JoystickButton(Controller1, Button.kB.value)
    .whenPressed(() -> m_IntakeSubsystem.compressorOn())
    .whenReleased(() -> m_IntakeSubsystem.compressorOff());
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    
    
    
    return auto.getSelected();
  }
}
