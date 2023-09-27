// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// The code in this file is based off of https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-4/creating-benchtop-test-program-cpp-java.html
// hi-there
package frc.robot;

//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax.IdleMode;
/**
 * The VM is configured to automatically run this class. If you change the name
 * of this class or the package after creating this project, you must also
 * update the build.gradle file in the project.
 */
// public class Robot extends RobotBase { // old extension.
public class Robot extends TimedRobot {

  // define and initialize some variables for the sample robot

  // Define left drivetrain Motors
  MotorControllerGroup m_left;

  // define right drivetrain motors
  MotorControllerGroup m_right;

  // define strafe motor
 MotorControllerGroup m_strafe;




  // define the drive
  DifferentialDrive m_drive;

  DifferentialDrive m_strafeDrive;
  // define the external encoders used for the drivetrain


  // define the joystick and xbox controller used to controll the robot
  Joystick m_joyStick;
  XboxController m_Controller_1;
  XboxController m_Controller_2;

  // define the camera
  UsbCamera camera1;
  UsbCamera camera2;
  // NetworkTableEntry cameraSelection;
  VideoSink server;

  //Auton chooser on smart dashboard


private static final String testAuto1 = "wave two times test";
private static final String testAuto2 = "wave one time test"; 

private String m_autoSelected;
private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // define timer
  Timer m_timer = new Timer();


  //define CAN
  //potential swerve drive stuff
  // CANSparkMax leftFrontSteeringMotor;
  // CANSparkMax leftFrontDriveMotor;
  // CANSparkMax leftRearSteeringMotor;
  // CANSparkMax leftRearDriveMotor;
  // CANSparkMax rightFrontSteeringMotor;
  // CANSparkMax rightFrontDriveMotor;
  // CANSparkMax rightRearSteeringMotor;
  // CANSparkMax rightRearDriveMotor;
  
  //define drive train under can
  CANSparkMax m_frontLeft;
  CANSparkMax m_rearLeft;
  CANSparkMax m_frontRight;
  CANSparkMax m_rearRight;

  CANSparkMax m_rightStrafeMotor;
  CANSparkMax m_leftStafeMotor;
  boolean strafeMode;
  CANSparkMax m_armMotor;
  CANSparkMax m_extendoArm;



 

 //state machine initialization
 double state = 0;




  public void robotInit() {
    // setup the camera
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);
    server = CameraServer.getServer();
    //camera1.setResolution(160, 120);
    //camera2.setResolution(160, 120);
    camera1.setVideoMode(PixelFormat.kYUYV, 160, 120, 30);
    camera2.setVideoMode(PixelFormat.kYUYV, 160, 120, 30);

    //set up auton chooser

   
    m_chooser.addOption("Wave one time test auto", testAuto2);
    m_chooser.addOption("Wave two times test auto", testAuto1);
    
    SmartDashboard.putData("Auto choices", m_chooser);

    // 4 motors on 0, 1, 2, 3
    // 2 (back) and 0 (front) are right side drive
    // 1 (front) and 3 (back) are left side drive
    // the drivetrain motors are controlled through RevRobotics Spark Controllers.
    // initialize the motors defined above


    //Changed to can to test something
    //m_frontLeft = new Spark(1);
    m_frontLeft = new CANSparkMax(2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    m_rearLeft = new CANSparkMax(1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);
    m_frontRight = new CANSparkMax(4, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    m_rearRight = new CANSparkMax(3, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    m_right = new MotorControllerGroup(m_frontRight, m_rearRight);
    
    /*
    Base motor controller initiallization for drive train via CAN
    leftFrontSteeringMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushless);
    leftFrontDriveMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushless);
    */
    // init the drive
    m_drive = new DifferentialDrive(m_left, m_right);
   
 
    // init the controllers
    m_Controller_1 = new XboxController(0);
    m_Controller_2 = new XboxController(1);

  }


  // These two methods are for autonomous control

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    // encoder1.setMaxPeriod(5);
    // encoder2.setMaxPeriod(5);
    // encoder1.reset();
    // encoder2.reset();
    //
    m_timer.reset();
    m_timer.start();
    m_rearLeft.setIdleMode(IdleMode.kBrake);
    m_rearRight.setIdleMode(IdleMode.kBrake);
    m_frontLeft.setIdleMode(IdleMode.kBrake);
    m_frontRight.setIdleMode(IdleMode.kBrake);
    state = 1;
   
    //auto selector
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected" + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  
    //m_time is the current value of the timer
    double m_time = m_timer.get();
  

  
    
    switch (m_autoSelected) {
    case testAuto1:
       
        break;

        case testAuto2: 
      
        break;
        
        

       

      
    }
   
    
   
}

  

  // these next two methods are for user control

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    CameraServer.startAutomaticCapture();
   
    m_rearLeft.setIdleMode(IdleMode.kCoast);
    m_rearRight.setIdleMode(IdleMode.kCoast);
    m_frontLeft.setIdleMode(IdleMode.kCoast);
    m_frontRight.setIdleMode(IdleMode.kCoast);
   
   
  }

  @Override
  public void teleopPeriodic() {
m_timer.start();

    //the code below controls scotts favorite thing the boost
    double drive_multiplier;
    if (m_Controller_1.getAButton() == true) {
      drive_multiplier = 1;
    } else {
      drive_multiplier = .5;
    }
    double m_drive_x = MathUtil.applyDeadband(m_Controller_1.getRawAxis(0), 0.1) * drive_multiplier;
    double m_drive_y = MathUtil.applyDeadband(m_Controller_1.getRawAxis(1), .1) * drive_multiplier;
    
  }

  // these two methods are for test. used to test robot code.

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    CameraServer.startAutomaticCapture();
    m_timer.reset();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }
}
