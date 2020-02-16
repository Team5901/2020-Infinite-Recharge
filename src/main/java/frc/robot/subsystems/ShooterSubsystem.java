/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;



public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private final WPI_TalonFX mav = new WPI_TalonFX(10);

   public ShooterSubsystem() {

    /* Config the peak and nominal outputs */
    mav.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    /* Config the peak and nominal outputs */
    mav.configNominalOutputForward(0, ShooterConstants.kTimeoutMs);
    mav.configNominalOutputReverse(0, ShooterConstants.kTimeoutMs);
    mav.configPeakOutputForward(1, ShooterConstants.kTimeoutMs);
    mav.configPeakOutputReverse(-1, ShooterConstants.kTimeoutMs);

    // Reverse motor direction
    mav.setInverted(true);

    //Set PID values
    mav.config_kF(ShooterConstants.kPIDLoopIdx, 0.09, 0);
    mav.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.kP, 0);
    mav.config_kI(ShooterConstants.kPIDLoopIdx, ShooterConstants.kI, 0);
    mav.config_kD(ShooterConstants.kPIDLoopIdx, ShooterConstants.kD, 0);
    mav.configClosedloopRamp(1);
    mav.setSensorPhase(true);
  }

  public void shooterSpeedUp(double RPM){
      double targetVelocity_UnitsPer100ms = 2375 * 2048 / 600;
      mav.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
      System.out.println(mav.getSelectedSensorVelocity()/2048*600);
      //mav.set(0.9);
  }

  public void stopShooter(){
    mav.stopMotor();
  }

  @Override
  public void periodic() {
    //This method will be called once per scheduler run
  }

}
