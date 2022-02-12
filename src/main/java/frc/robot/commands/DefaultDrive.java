/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDrive extends CommandBase {
  /**
   * Creates a new DefaultDrive.
   */
  private final DrivetrainSubsystem m_drivetrain;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;

  public DefaultDrive(DoubleSupplier left, DoubleSupplier right, DrivetrainSubsystem drivetrain) {
    m_drivetrain = drivetrain;
    m_left = left;
    m_right = right;

    addRequirements(m_drivetrain);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_drivetrain.cougarDrive(m_left.getAsDouble(), m_right.getAsDouble());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.cougarDrive(0, 0);
  }
}