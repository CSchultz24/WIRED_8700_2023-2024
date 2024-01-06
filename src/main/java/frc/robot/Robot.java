// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// The code in this file is based off of https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-4/creating-benchtop-test-program-cpp-java.html
// hi-there
package frc.robot;

//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class. If you change the name
 * of this class or the package after creating this project, you must also
 * update the build.gradle file in the project.
 */
// public class Robot extends RobotBase { // old extension.
public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;

  private double m_disableStartTime;

  private boolean driveIsBraked;

  public static int lpctra;

  private int loopCtr;

  private boolean lastOKState;

  private boolean firstScan = true;

  private boolean autoHasRun;
  private double m_startDelay;
  private double startTime;
  Command autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    if (RobotBase.isReal()) {
      DataLogManager.start("/media/sda1");
      // Instantiate our RobotContainer.
      // Record both DS control and joystick data
      DriverStation.startDataLog(DataLogManager.getLog());
      if (isReal())
        Timer.delay(10);// allow navx to init Chief Delphi post fix intermiitent trajectory failures
                        // after start up
    }
    m_robotContainer = new RobotContainer();

  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    loopCtr++;

    if (loopCtr >= 100) {

      m_robotContainer.m_drive.ALL_CANOK = m_robotContainer.m_drive.checkCANOK();
        

      loopCtr = 0;
    }
    
    SmartDashboard.putNumber("LP", lpctra++);

    // m_loop.poll();
    if (RobotBase.isSimulation())
      m_robotContainer.m_fieldSim.periodic();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_disableStartTime = 0;
    firstScan = true;

    autoHasRun = false;
    // CommandScheduler.getInstance().cancelAll();
    //
  }

  @Override
  public void disabledPeriodic() {

    if (!m_robotContainer.m_drive.isbraked()) {
      if (m_disableStartTime == 0)
        m_disableStartTime = Timer.getFPGATimestamp();

      if (m_disableStartTime != 0 && Timer.getFPGATimestamp() > m_disableStartTime + 3) {
        m_robotContainer.m_drive.setIdleMode(false);
      }
    }

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    autoHasRun = false;

    Shuffleboard.startRecording();

    m_robotContainer.m_drive.gyroStartPitch = m_robotContainer.m_drive.getGyroPitch();

    m_robotContainer.m_drive.m_fieldOriented = false;

    m_robotContainer.m_drive.setIdleMode(true);

   
    m_robotContainer.m_drive.setClosedLoop(true);


  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (RobotBase.isSimulation())
      m_robotContainer.m_fieldSim.periodic();

    if (!autoHasRun && Timer.getFPGATimestamp() > startTime + m_startDelay
        && autonomousCommand != null) {
      autonomousCommand.schedule();
      autoHasRun = true;
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the a
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (!autoHasRun)
      m_robotContainer.m_drive.gyroStartPitch = m_robotContainer.m_drive.getGyroPitch();

    if (autoHasRun)

      m_robotContainer.m_drive.fieldOrientOffset = 180;

    m_robotContainer.m_drive.setClosedLoop(false);

    autoHasRun = false;

    m_robotContainer.m_drive.resetGyro();


    m_robotContainer.m_drive.setIdleMode(true);


    m_robotContainer.m_drive.m_fieldOriented = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (RobotBase.isSimulation())
      m_robotContainer.m_fieldSim.periodic();

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    m_robotContainer.m_drive.setIdleMode(true);

    CommandScheduler.getInstance().cancelAll();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }

  @Override
  public void simulationPeriodic() {
    m_robotContainer.m_fieldSim.periodic();
    // m_robotContainer.simulationPeriodic();
  }

}
