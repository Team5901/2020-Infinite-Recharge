/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TowerSubsystem extends SubsystemBase {
  /**
   * Creates a new TowerSubsystem.
   */
  private final WPI_TalonSRX TowerControl = new WPI_TalonSRX(6);

  public TowerSubsystem() {

  }
  public void moveBall(double power){
    TowerControl.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
