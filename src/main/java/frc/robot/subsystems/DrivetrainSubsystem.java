/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotPorts;

public class DrivetrainSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX leftFrontDriveMotor = new WPI_TalonFX(RobotPorts.kLeftFrontMotor);
  private final WPI_TalonFX leftRearDriveMotor = new WPI_TalonFX(RobotPorts.kLeftRearMotor);
  private final WPI_TalonFX rightFrontDriveMotor = new WPI_TalonFX(RobotPorts.kRightFrontMotor);
  private final WPI_TalonFX rightRearDriveMotor = new WPI_TalonFX(RobotPorts.kRightRearMotor);

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

  public DrivetrainSubsystem() {
    // Sets the distance per pulse for the encoders
    leftFrontDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    rightFrontDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
  }

  public void arcadeDrive(double fwd, double rot) {

    double x = Math.pow(fwd,3);
    double y = Math.pow(rot,3);
    m_drive.arcadeDrive(Math.max(-0.8,Math.min(0.8,x)),Math.max(-0.6,Math.min(0.6,y)));
  }
  
  public void AutoDroive(double distance) {
    //find absolute error
    double error = Math.abs(distance - getAverageEncoderDistance());

    //if distance is positive and error is greater than 
    if(error > Constants.DriveConstants.kAutoDistanceError){      
      arcadeDrive(Constants.DriveConstants.kAutoSpeedRatio, -m_gyro.getAngle()*Constants.DriveConstants.kAutoTurnRatio);
    }  
    else {
      arcadeDrive(0,0);
    }
  }
  
  public void TurnControl(double AngleTarget){
    double angle = m_gyro.getAngle();
    double target = angle + AngleTarget;

    if(Math.abs(target) > Constants.DriveConstants.kAutoAngleError){
      arcadeDrive(0, -Math.signum(target)*(Math.abs(target)*Constants.DriveConstants.kAutoTurnRatio) + Constants.DriveConstants.kAutoMinRotRatio);
    }
    else {
      arcadeDrive(0, 0);
    }
  }

  public void resetEncoders() {
    leftFrontDriveMotor.setSelectedSensorPosition(0);
    rightFrontDriveMotor.setSelectedSensorPosition(0);
  }

  public double getAverageEncoderDistance() {
    double distancePerUnit=6*Math.PI/2048;
    return (getLeftEncoder()+getRightEncoder())*distancePerUnit / 2.0;
  }

  public double getLeftEncoder() {
    return leftFrontDriveMotor.getSelectedSensorPosition();
  }

  public double getRightEncoder() {
    return rightFrontDriveMotor.getSelectedSensorPosition();
  }

  public double getAngle() {
    return m_gyro.getAngle();
  }

  public void resetAngle(){
    m_gyro.reset();
  }

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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distance Traveled",getAverageEncoderDistance());
    SmartDashboard.putNumber("Angle Heading",getAngle());
    //This method will be called once per scheduler run
  }

}