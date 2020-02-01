/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TowerSubsystem;

public class MoveTowerBall extends CommandBase {
  /**
   * Creates a new MoveTowerBall.
   */
  final TowerSubsystem TheRisenOne;
  public MoveTowerBall(TowerSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    TheRisenOne = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TheRisenOne.moveBall(-1);
  

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    TheRisenOne.moveBall(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
