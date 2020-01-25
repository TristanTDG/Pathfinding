//Conner Taylor
//FRC 2019 Robot Competitive BOM


/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.ecommons.Constants;
import frc.ecommons.RobotMap;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;


public class Robot extends TimedRobot {

  private DriveTrain m_DriveTrain = new DriveTrain();
  
  Joystick m_driveJoy;
  
  Boolean compLoop = false;

  AnalogInput pressure;
  double pressureDouble;

  ShuffleboardTab camera = Shuffleboard.getTab("Camera Addresses");
  NetworkTableEntry addressEntry = camera.add("Server Address", "NULL")
                                         .getEntry();
  NetworkTableEntry portEntry = camera.add("Server Port", 0)
                                      .getEntry();

  ShuffleboardTab tab = Shuffleboard.getTab("Beginning Game");
  NetworkTableEntry pressureEntry = tab.add("Pressure", 0) 
                                       .withWidget(BuiltInWidgets.kDial)
                                       .withPosition(9, 0)
                                       .withSize(1, 1)
                                       .getEntry();
  NetworkTableEntry compressorEntry = tab.add("Compressor", false)
                                         .withPosition(10, 0)
                                         .withSize(1, 1)
                                         .withWidget(BuiltInWidgets.kBooleanBox)
                                         .getEntry();




  @Override
  public void robotInit() {
    Shuffleboard.selectTab("Beginning Game");
    m_driveJoy = new Joystick(RobotMap.driveJoy);
    m_DriveTrain.robotInit(m_driveJoy);
  }

 
  @Override
  public void robotPeriodic() {
    m_DriveTrain.report();
  }


  @Override
  public void autonomousInit() {
    m_DriveTrain.autonomousInit();
  }


  @Override
  public void autonomousPeriodic() {
    m_DriveTrain.autonomousPeriodic();
  
  }

  @Override
  public void teleopInit() {
    m_DriveTrain.teleopInit();
  }


  @Override
  public void teleopPeriodic() {
    m_DriveTrain.teleopPeriodic();
  }

  @Override
  public void testInit() {
    m_DriveTrain.testInit();
  }

  @Override
  public void testPeriodic() {
    m_DriveTrain.testPeriodic();
    
  }

}
