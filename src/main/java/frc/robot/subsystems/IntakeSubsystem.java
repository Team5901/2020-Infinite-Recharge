/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsystem.
   */
  private final WPI_TalonSRX boss = new WPI_TalonSRX(5);
  private final Compressor joe = new Compressor(0);
  private final Solenoid phil = new Solenoid(7);
  
  public IntakeSubsystem() {
    boss.setInverted(true);

  }

  public void intakeMotorOn(double power){
    boss.set(power);

  }

  public void intakeMotorOff(){
    boss.stopMotor();
  }
  
  public void compressorOn(){
    joe.setClosedLoopControl(true);
  }

  public void compressorOff(){
    joe.setClosedLoopControl(false);

  }

  public void solenoidOn(){
    phil.set(true);

  }

  public void solenoidOff(){
    phil.set(false);

  }


  



  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
