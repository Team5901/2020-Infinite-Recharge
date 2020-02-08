/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private final WPI_TalonFX mav = new WPI_TalonFX(10);
 
   public ShooterSubsystem() {
    /* Config the peak and nominal outputs */
    //mav.configSelectedFeedbackSensor(10, 0, ShooterConstants.kTimeoutMs)
    //mav.configSelectedFeedbackSensor(10, 0, ShooterConstants.kTimeoutMs);
    mav.configNominalOutputForward(0, ShooterConstants.kTimeoutMs);
    mav.configNominalOutputReverse(0, ShooterConstants.kTimeoutMs);
    mav.configPeakOutputForward(1, ShooterConstants.kTimeoutMs);
    mav.configPeakOutputReverse(-1, ShooterConstants.kTimeoutMs);
    mav.setInverted(true);
    mav.config_kF(ShooterConstants.kPIDLoopIdx, ShooterConstants.kF, ShooterConstants.kTimeoutMs);
    mav.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.kP, ShooterConstants.kTimeoutMs);
    mav.config_kI(ShooterConstants.kPIDLoopIdx, ShooterConstants.kI, ShooterConstants.kTimeoutMs);
    mav.config_kD(ShooterConstants.kPIDLoopIdx, ShooterConstants.kD, ShooterConstants.kTimeoutMs);

  }

  public void shooterSpeedUp(double RPM){
      //double targetVelocity_UnitsPer100ms = RPM * 4096/2 / 600;
      //mav.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
      mav.set(0.9);
  }

  public void stopShooter(){
    mav.stopMotor();
  }


  @Override
  public void periodic() {
    //This method will be called once per scheduler run
  }
}
