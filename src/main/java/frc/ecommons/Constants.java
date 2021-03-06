package frc.ecommons;


public class Constants {

// *** Drive Joystick Axis ***
    // Left stick Y
    // drive fwd, bkwd
    public static int yAxis = 1;

    // Triggers Manipulator. Left = up. Right = Down
    public static int manipulatorUp = 2;
    public static int manipulatorDown = 3;

    public static int manipCargoPreset = 3;

    // Right stick X
    // turns
    public static int xAxis = 4;
 
// *** Drive Joystick Buttons ***
    public static int motorTest = 0;

    //B button
    //Toggle the disc grabber
    public static int discGrabber = 2;
    
    //A Button
    //Dual Action Pneumatic
    public static int dualActionGrabber = 1;

    // X
    // limelight turn
    public static int limelightAutoTurn = 7;

    // Y
    // limelight led on
    public static int limelightLEDon = 4;

    // toggles manipulator motor
    public static int manipWheelBackToggle = 5;
    public static int manipWheelForToggle = 6;

    public static int onPushDelivery = 4;

    // Select button (small black button just left of the center of the controller)
    // resets the drive shaft encoders' position and the elevator encoder's position to 0
    public static int encoderReset = 7;

    // Menu button (small black button just right of the center of the controller)
    // toggles the compressor on/off
    public static int compressor = 8;



// *** elevator joystick ***
// axis
    // left trigger
    // drive elevator down
    public static int elevatorDown = 2;
    // right trigger
    // drive elevator up
    public static int elevatorUp = 3;

    public static int elevMode = 6;


// buttons
    // Y button
    // elevator closed loop control
    // goes to position selected by index of heights array
    public static int elevatorToPositionButton = 4;
    
    //ELEVATOR DRIVE
    public static int elevatorAdd = 6;
    public static int elevatorSub = 5;

    //Preset Encoder Pos
    public static int elevatorBot = 1;
    public static int elevatorMid = 3;
    public static int elevatorTop = 4;
    public static int elevatorCargo = 2;

// *** gurney joystick axis ***
    // Y axis
    public static int gDrive = 1;

    // Right Stick Y axis
    // winds front gurney motor
    public static int gUpFront = 5;

// *** gurney joystick buttons ***
    // go down
    public static int gurneyGoDown = 2;

    // enable auto up with leveling
    public static int gurneyGoUp = 1;

    // raise back when on the floor
    public static int raiseBackGurney = 3;

    // lower back gurney when on floor
    public static int lowerBackGurney = 5;

    // //unused
    public static int gurneyManualDrive = 6;

    public static int gurneyEncoderReset = 7;

    // unused
    public static int gurneyHoldEnable = 8;

// *** gurney balance calibration ***
    //P factor of back motor
    public static double frontDriveP = 8.5;

    //Current at which the robot will lower safely
    public static double gSafteySpeed = .075;

     //Current at which the gurneys will rise while getting on the platform
     public static double gDriveUp = .7;

     //Height at which to stop giving motors current when lowering
     public static int gSafeHeight = 400;
 
     //Amount of change in degrees allowed between tests when checking if the robot is staying stable with a gurney raised
     public static int gPitchTolerance = 1;
 
     //Amount of time in milliseconds between pitch readings when the robot checks if it can raise a gurney
     public static int gWaitTime = 400;
 
     //Needed change in the gurney after gWaitTime milliseconds to classify the robot as 'not lodged'
     public static int gValueChange = 100;

     //Encoder value to pull the back gurney up to
     public static int gBackUp = 100;
 
 // *** automatic turning constants ***
     
     //Distance from robot wheels to robot center
     //TODO: Get the real offset in feet
    public static double wheelOffset = 0;

    //Radius of robot wheels
    //TODO: Get the real wheel radiuses
    public static double wheelRadius = 0;

    // Ticks per rotation of the wheels
    //TODO: Get real value for this
    public static double ticksPerRotation = 0;

 // *** gurney subroutine buttons ***
 
     //Continues the subroutine when the driver is set up
     //TODO: Get the real button indice
     //public static int gContinueRoutine = 0;
 
     //Stops the climbing routine with a safe lower
     //TODO: Get the real button indice
     //public static int gStopRoutine = 0;
 
     //Reverts robot to drive-while-raised mode
     //TODO: Get the real button indice
    // public static int gRevertStage = 0;

     //Drives up the front gurney when climing
     //TODO: Get the real button indice
     //public static int gFrontUp = 0;

// *** other ***
    //Percent Output
    public static double gForwardRate = 0.3;
    public static double gBackwardRate = -0.3;
    
    //The angle at which the RoboRio and NavX board are tilted measured counterclockwise from the right, back, and top
    public static double rioYaw = 0;
    public static double rioPitch = 34;
    public static double rioRoll = 0;

//Distance measurements for Limelight
    //Limelight
    public static double camHeight = 0.0;
    public static double targetHeight = 0.0;
    public static double camAngle = 0.0;



//Robot Characterization Constants
    public static final double ksVolts = 1.31;
    public static final double kvVoltSecondsPerMeter = 0.0426;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0129;
    public static final double kTrackWidthMeters = 0.69; //needs fixing

    public static final double kPDriveVel = 0.624;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static double kEncoderDistancePerPulse; // NEEDS VALUE

}