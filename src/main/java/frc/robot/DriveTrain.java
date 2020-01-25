//Conner Taylor
//Drive Train of Robot

package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.ecommons.Constants;
import frc.ecommons.RobotMap;
import com.kauailabs.navx.*;
import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.wpilibj.shuffleboard.BuiltInTypes;

// import edu.wpi.first.wpilibj.shuffleboard;

public class DriveTrain {

  // Joysticks/Controllers
  Joystick m_joy;

  // Talons
  WPI_TalonSRX m_rMaster;
  WPI_TalonSRX m_lMaster;

  // Encoders
  Encoder m_leftEncoder;
  Encoder m_rightEncoder;

  // Victors
  WPI_VictorSPX m_rSlave1;
  WPI_VictorSPX m_lSlave1;

  // NavX Board
  AHRS navX;

  // Solenoids

  // Loops
  boolean dgLoop = false;
  boolean driveTestLoop = false;
  public static boolean camSwitch = false;
  boolean camLoop = false;

  double driveSpeed = 0.5;

  DifferentialDrive m_drive;
  DifferentialDriveKinematics kDriveKinematics; // Odometry class for tracking robot pose
  DifferentialDriveOdometry m_odometry;
  // Create config for trajectory
  TrajectoryConfig config;

  // Create a voltage constraint to ensure we don't accelerate too fast

  // m_rMaster.getSelectedSensorPosition()
  // m_lMaster.getSelectedSensorPosition()
  // two test trajectories
  Trajectory exampleTrajectory;
  Trajectory trajectory;
  RamseteController ramseteController;
  Trajectory.State goal;
  ChassisSpeeds adjustedSpeeds;

  Timer run;
  ShuffleboardTab testMode = Shuffleboard.getTab("Test Mode");
  NetworkTableEntry testDriveEntry = testMode.add("Drive Test", false)
                                             .withWidget(BuiltInWidgets.kToggleButton)
                                             .getEntry();
  NetworkTableEntry testTimerEntry = testMode.add("Timer", 0)
                                             .withWidget(BuiltInWidgets.kTextView)
                                             .getEntry();
  
  ShuffleboardTab tab = Shuffleboard.getTab("Beginning Game");

  ShuffleboardTab motor = Shuffleboard.getTab("Motors");
  
  NetworkTableEntry txEntry = testMode.add("Limelight tx", 0)
                                  .withPosition(0, 3)
                                  .withWidget(BuiltInWidgets.kTextView)
                                  .getEntry();

  NetworkTableEntry steeringEntry = testMode.add("Steering Adjust Output", 0)
                                  .withPosition(1, 3)
                                  .withSize(2,1)
                                  .withWidget(BuiltInWidgets.kTextView)
                                  .getEntry();

  NetworkTableEntry rightEncoderEntry = tab.add("Right Encoder", 0)
                                           .withSize(1, 1)
                                           .withPosition(1, 1) 
                                           .getEntry();

  NetworkTableEntry leftEncoderEntry = tab.add("Left Encoder", 0)
                                          .withSize(1, 1)
                                          .withPosition(0, 4)
                                          .getEntry();

  NetworkTableEntry rightVelocity = tab.add("Right Speed", 0)
                                          .withSize(1, 1)
                                          .withPosition(0, 5)
                                          .getEntry();


  NetworkTableEntry leftVelocity = tab.add("Left Speed", 0)
                                          .withSize(1, 1)
                                          .withPosition(0, 6)
                                          .getEntry();

  NetworkTableEntry rightPathGoal = tab.add("Right Path Goal", 0)
                                          .withSize(1, 1)
                                          .withPosition(0, 7)
                                          .getEntry();


  NetworkTableEntry leftPathGoal = tab.add("Left Path Goal", 0)
                                          .withSize(1, 1)
                                          .withPosition(0, 8)
                                          .getEntry();





  final double pi = 3.1415926535897932384626433832;

  public void TalonConfig() {
    //Configs Talon to default
    m_rMaster.configFactoryDefault();
    m_lMaster.configFactoryDefault();
    m_rSlave1.configFactoryDefault();
    m_lSlave1.configFactoryDefault();
    
    m_rSlave1.follow(m_rMaster);
    m_lSlave1.follow(m_lMaster);
    //Motors go right way
    m_rMaster.setSensorPhase(false);
    m_lMaster.setSensorPhase(false);


    m_rMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    m_lMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

    m_rMaster.setInverted(false);
    m_rSlave1.setInverted(InvertType.FollowMaster);
   
    m_lMaster.setInverted(false);
    m_lSlave1.setInverted(InvertType.FollowMaster);

    m_rMaster.setNeutralMode(NeutralMode.Brake);
    m_lMaster.setNeutralMode(NeutralMode.Brake);
    m_lSlave1.setNeutralMode(NeutralMode.Brake);
    m_rSlave1.setNeutralMode(NeutralMode.Brake);
    
  
    //ARGS (Slot, Value)
    m_rMaster.config_kF(0, 0.0);
    m_rMaster.config_kP(0, 0);
    m_rMaster.config_kI(0, 0);
    m_rMaster.config_kD(0, 0);
    m_rMaster.config_IntegralZone(0, 1000);

    m_lMaster.config_kF(0, 0.00);
    m_lMaster.config_kP(0, 0);
    m_lMaster.config_kI(0, 0);
    m_lMaster.config_kD(0, 0);
    m_lMaster.config_IntegralZone(0, 1000);


    int sensorUnitsPer100ms = 8000;
    m_rMaster.configMotionCruiseVelocity(sensorUnitsPer100ms);
    m_lMaster.configMotionCruiseVelocity(sensorUnitsPer100ms);

    int sensorUnitsPer100ms_per_sec = 10240;
    m_rMaster.configMotionAcceleration(sensorUnitsPer100ms_per_sec);
    m_lMaster.configMotionAcceleration(sensorUnitsPer100ms_per_sec);


  }


  public void robotInit(Joystick j) {
    run = new Timer();
    //Joysticks
    m_joy = j;

    //Talons - IDS found in ecommons.RobotMap
    m_rMaster = new WPI_TalonSRX(RobotMap.rMaster);
    m_lMaster = new WPI_TalonSRX(RobotMap.lMaster);

    //Victors - IDS found in ecommons.RobotMap
    m_rSlave1 = new WPI_VictorSPX(RobotMap.rSlave1);
    m_lSlave1 = new WPI_VictorSPX(RobotMap.lSlave1);

    //Encoders - IDS found in ecommons.RobotMap (I hope...)
    m_rightEncoder = new Encoder(0, 1);
    m_leftEncoder = new Encoder(2, 3);
    //tics per 100ms to meters per second need to clean up

    kDriveKinematics = new DifferentialDriveKinematics(Constants.kTrackWidthMeters);
    m_drive = new DifferentialDrive(m_lMaster, m_rMaster); //who knows if this is gonna work, not me looool
    ramseteController = new RamseteController();
    goal = trajectory.sample(3.4);

    TalonConfig();


    
  }

  public void autonomousInit() {
    //Finds ratios for hard coded paths
  }
  public void autonomousPeriodic() {
    teleopPeriodic();
  }



  /**
   * This function is called periodically during operator control.
   */
  
  public void teleopInit() {
  }

    public void teleopPeriodic() {

      if (m_joy.getPOV() == 90 && !camLoop) {
        camLoop = true;
        camSwitch = !camSwitch;
      } else if (m_joy.getPOV() != 90) {
        camLoop = false;
      }

    //Equation for ARCADE DRIVE
    double xAxis, yAxis;
    xAxis = 0.35 * m_joy.getRawAxis(Constants.xAxis);
    // * -1 to correct axis sign
    yAxis = -1*m_joy.getRawAxis(Constants.yAxis);
    
    //Equation for Arcade Drive
    double leftSide, rightSide;
    rightSide = -(yAxis - xAxis);
    leftSide = yAxis + xAxis; 

    
    
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getDistance(m_lMaster), getDistance(m_rMaster));                                            //FIX??
    ramseteController.calculate(getPose(), goal);
    adjustedSpeeds = ramseteController.calculate(getPose(), goal);
  }

   public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(((m_lMaster.getSelectedSensorVelocity(0)*10*0.1524*pi)/(4096*6)), ((m_rMaster.getSelectedSensorVelocity(0)*10*0.1524*pi)/(4096*6)));
  } //tics per 100ms to meters per second // used in place of encoder.getRate() method


  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(navX.getAngle(), 360);
  }

  public double getDistance(WPI_TalonSRX master) {
    return master.getSelectedSensorPosition();
  }
  

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_rMaster.setSelectedSensorPosition(0);
    m_lMaster.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getDistance(m_rMaster) + getDistance(m_lMaster)) / 2.0;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navX.getRate();
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    navX.reset();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_lMaster.setVoltage(leftVolts);
    m_rMaster.setVoltage(-rightVolts);
    m_drive.feed();
  }

  public void report() {
    
    rightEncoderEntry.setDouble(m_rMaster.getSelectedSensorPosition());
    leftEncoderEntry.setDouble(m_lMaster.getSelectedSensorPosition());

    
  }

  /**
   * This function is called periodically during test mode.
   */
  public void testInit() {
    run.reset();
    testDriveEntry.setBoolean(false);
    
  }
  public void testPeriodic() {
    m_lSlave1.follow(m_lMaster);
    m_rSlave1.follow(m_rMaster);

    //Test to see if driving works
    boolean driveTestCheck = testDriveEntry.getBoolean(false);
    testTimerEntry.setDouble(run.get());
    if (driveTestCheck && !driveTestLoop) {
      driveTestLoop = true;
      run.reset();
      run.start();

    }
    if (run.get() < 3 && run.get() > 0.1) {
      m_rMaster.set(ControlMode.PercentOutput, 0.7);
      m_lMaster.set(ControlMode.PercentOutput, 0.7);
    } else if (run.get() > 3 && run.get() < 6) {
      m_rMaster.set(ControlMode.PercentOutput, -0.7);
      m_lMaster.set(ControlMode.PercentOutput, -0.7);
    } else if (run.get() >= 6) {
      m_lMaster.set(ControlMode.PercentOutput, 0);
      m_rMaster.set(ControlMode.PercentOutput, 0);
      testDriveEntry.setBoolean(false);
    }
    if (!driveTestCheck) {
      driveTestLoop = false;
      m_lMaster.set(ControlMode.PercentOutput, 0);
      m_rMaster.set(ControlMode.PercentOutput, 0);
      run.stop();
      run.reset();
     }

  }
}
