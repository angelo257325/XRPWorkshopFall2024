// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPServo;

public class Robot extends TimedRobot {

  //Giving the DifferentalDrive a name
  public DifferentialDrive m_myRobot;

  //Giveing the motors a name and telling it which port it is connected to 
  public XRPMotor m_lefMotor = new XRPMotor(0); 
  public XRPMotor m_rightMotor = new XRPMotor(1);

  //Giving The Servo a name and Telling it which port it is connected to
  //Please note that Servo 1 is DeviceNum 4
  public XRPServo m_arm = new XRPServo(4);

  //Giving the Xbox Controller a name and telling it which port the controller should be connected to
  private XboxController driverController = new XboxController(0);

  @Override
  public void robotInit() {

  //Since the right motor is filped on the other side it must be inverted in order for it to turn in the same direction as the left motor
  m_rightMotor.setInverted(true);

  //In order to use the differental drive commmand you must tell it which motor is the right motor and which motor is the left motor
  m_myRobot = new DifferentialDrive(m_lefMotor, m_rightMotor);

//Since we are using the differentalDrive command the motors do not update their values enough for wpilib so you must disable the mtor safety function
  m_myRobot.setSafetyEnabled(false);

  }

 @Override
  public void testPeriodic() {

    //Contorls the robot via the tank configuration
    m_myRobot.tankDrive(driverController.getLeftY(), -driverController.getRightY());

    //Setting up a variable so that when the tirgger is pulled the arm will move
    double armAngle = driverController.getLeftTriggerAxis()*180;

    //tells the arm to move 
    m_arm.setAngle(armAngle);

  }
}
