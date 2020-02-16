/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotPorts;

public class DrivetrainSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX leftFrontDriveMotor = new WPI_TalonFX(RobotPorts.kLeftFrontMotor);
  private final WPI_TalonFX leftRearDriveMotor = new WPI_TalonFX(RobotPorts.kLeftFrontMotor);
  private final WPI_TalonFX rightFrontDriveMotor = new WPI_TalonFX(RobotPorts.kLeftFrontMotor);
  private final WPI_TalonFX rightRearDriveMotor = new WPI_TalonFX(RobotPorts.kLeftFrontMotor);

  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(leftFrontDriveMotor, leftRearDriveMotor);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(rightFrontDriveMotor,rightRearDriveMotor);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The Gyro
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Shifter Solenoid
  private final Solenoid SolarNoise = new Solenoid(RobotPorts.kShifterSolenoid);


  /**
   * Creates a new DriveSubsystem.
   */
  public DrivetrainSubsystem() {
    // Sets the distance per pulse for the encoders
    leftFrontDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(Math.pow(fwd,3),Math.pow(rot,3));
  }
  
  /*public void AutoDroive(double distance) {
    //find absolute error
    double error = Math.abs(distance - getAverageEncoderDistance());

    //if distance is positive and error is greater than 
    if(getAverageEncoderDistance() > 0 && error > Constants.DriveConstants.kAutoDistanceError){      
      arcadeDrive(Constants.DriveConstants.kAutoSpeedRatio, -m_gyro.getAngle()*Constants.DriveConstants.kAutoTurnRatio);
    }
    else if (getAverageEncoderDistance() < 0 && error > Constants.DriveConstants.kAutoDistanceError) {
      arcadeDrive(Constants.DriveConstants.kAutoSpeedRatio, -m_gyro.getAngle()*Constants.DriveConstants.kAutoTurnRatio);
    }
    else {
      arcadeDrive(0,0);
    }
  }*/
  
  public void turnControl(double angleTarget){
    double angle = m_gyro.getAngle();
    double target = angle + angleTarget;

    if(Math.abs(target) < Constants.DriveConstants.kAutoAngleError){
      arcadeDrive(0, -Math.signum(target)*(Math.abs(target)*Constants.DriveConstants.kAutoTurnRatio) + Constants.DriveConstants.kAutoMinRotRatio);
    }
    else {
      arcadeDrive(0, 0);
    }
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    //m_leftEncoder.reset();
    //m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  /*public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }*/

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoder() {
    return leftFrontDriveMotor.getSelectedSensorPosition();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoder() {
    return rightFrontDriveMotor.getSelectedSensorPosition();
  }

  public double getAngle() {
    return m_gyro.getAngle();
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

//shiftIn pulls in
  public void shiftIn() {
    SolarNoise.set(false);
  
  }
  //shiftOut pushes Out
  public void shiftOut(){
    SolarNoise.set(true);

  }
  
  public boolean shiftStatus(){
    return SolarNoise.get();
  }


}