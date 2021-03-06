/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class RaiseElevator extends CommandBase {
  final ClimberSubsystem m_TheClimb;
  /**
   * Creates a new RaiseElevator.
   */
  public RaiseElevator(ClimberSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_TheClimb = subsystem;
    addRequirements(subsystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Climber Position: " + m_TheClimb.getClimberheight());
    m_TheClimb.setTheclimb(Constants.ClimberConstants.kClimberPwrUp);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_TheClimb.stopTheclimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
