// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pref;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.PDPConstants;
import frc.robot.Constants.PPConstants;
import frc.robot.utils.ShuffleboardContent;

public class DriveSubsystem extends SubsystemBase {

  public SwerveDriveKinematics m_kinematics = DriveConstants.m_kinematics;

  public boolean isOpenLoop = true;// RobotBase.isSimulation() && !DriverStation.isAutonomousEnabled();

  public final SwerveModuleSM m_frontLeft = new SwerveModuleSM(
      IDConstants.FRONT_LEFT_LOCATION,
      CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
      CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
      CanConstants.FRONT_LEFT_MODULE_STEER_CANCODER,
      DriveConstants.kFrontLeftDriveMotorReversed,
      DriveConstants.kFrontLeftTurningMotorReversed,
      PDPConstants.FRONT_LEFT_DRIVE_CHANNEL,
      PDPConstants.FRONT_LEFT_TURN_CHANNEL,
      isOpenLoop,
      CanConstants.FRONT_LEFT_MODULE_STEER_OFFSET);

  public final SwerveModuleSM m_frontRight = new SwerveModuleSM(
      IDConstants.FRONT_RIGHT_LOCATION,
      CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
      CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
      CanConstants.FRONT_RIGHT_MODULE_STEER_CANCODER,
      DriveConstants.kFrontRightDriveMotorReversed,
      DriveConstants.kFrontRightTurningMotorReversed,
      PDPConstants.FRONT_RIGHT_DRIVE_CHANNEL,
      PDPConstants.FRONT_RIGHT_TURN_CHANNEL,
      isOpenLoop,
      CanConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);

  public final SwerveModuleSM m_backLeft = new SwerveModuleSM(
      IDConstants.REAR_LEFT_LOCATION,
      CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
      CanConstants.BACK_LEFT_MODULE_STEER_MOTOR,
      CanConstants.BACK_LEFT_MODULE_STEER_CANCODER,
      DriveConstants.kBackLeftDriveMotorReversed,
      DriveConstants.kBackLeftTurningMotorReversed,
      PDPConstants.BACK_LEFT_DRIVE_CHANNEL,
      PDPConstants.BACK_LEFT_TURN_CHANNEL,
      isOpenLoop,
      CanConstants.BACK_LEFT_MODULE_STEER_OFFSET);

  public final SwerveModuleSM m_backRight = new SwerveModuleSM(
      IDConstants.REAR_RIGHT_LOCATION,
      CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
      CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
      CanConstants.BACK_RIGHT_MODULE_STEER_CANCODER,
      DriveConstants.kBackRightDriveMotorReversed,
      DriveConstants.kBackRightTurningMotorReversed,
      PDPConstants.BACK_LEFT_DRIVE_CHANNEL,
      PDPConstants.BACK_LEFT_TURN_CHANNEL,
      isOpenLoop,
      CanConstants.BACK_RIGHT_MODULE_STEER_OFFSET);

  // The gyro sensor

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

  /*
   * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
   * The numbers used
   * below are robot specific, and should be tuned.
   */
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      },
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  public boolean showOnShuffleboard;// set in RobotContainer

  private SimDouble m_simAngle;// navx sim

  public double throttleValue;

  public double targetAngle;

  public boolean m_fieldOriented = false;

  public boolean useVisionOdometry = true;

  private double startTime;

  private double positionStart;

  double positionChange;

  private Pose2d visionPoseEstimatedData;

  private double latencyMs;

  public boolean visionDataAvailable;

  private PIDController xPID = new PIDController(
      PPConstants.kPXController, PPConstants.kIXController, PPConstants.kIXController); // X

  private PIDController yPID = new PIDController(PPConstants.kPYController, PPConstants.kIYController,
      PPConstants.kDYController);

  private PIDController thetaPID = new PIDController(PPConstants.kPThetaController, PPConstants.kIThetaController,
      PPConstants.kDThetaController);

  public boolean runTrajectory;

  private final EventLoop m_drloop = new EventLoop();

  // private SwerveModuleDisplay m_smd = new SwerveModuleDisplay(this);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    // SmartDashboard.putData("SM", m_smd);

    m_gyro.reset();

    resetModuleEncoders();

    setIdleMode(true);

    m_fieldOriented = false;

    if (RobotBase.isSimulation()) {

      var dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

      m_simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")));
    }

    if (showOnShuffleboard)

      ShuffleboardContent.initMisc(this);

  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  /*
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * 
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * 
   * @param rot Angular rate of the robot.
   * 
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   * field.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        this.m_fieldOriented
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    SmartDashboard.putString("TRROB", getTranslation().toString());

    updateOdometry();

    // SmartDashboard.putNumber("Xpos", getX());
    // SmartDashboard.putNumber("Ypos", getY());
    // SmartDashboard.putNumber("GyroAngle", getHeadingDegrees());

    // SmartDashboard.putNumber("GyroPitch", getGyroPitch());

    // SmartDashboard.putNumber("GyroRoll", getGyroRoll());

    if (startTime == 0) {

      startTime = Timer.getFPGATimestamp();

      positionStart = getEstimatedPose().getX();

    }

    if (Timer.getFPGATimestamp() > startTime + 5) {

      positionChange = getEstimatedPose().getX() - positionStart;

      startTime = 0;

      positionStart = getEstimatedPose().getX();

    }

    if (Pref.getPref("XTune") == 1)
      tuneXPIDGains();
    if (Pref.getPref("YTune") == 1)
      tuneYPIDGains();
    if (Pref.getPref("ThetaTune") == 1)
      tuneThetaPIDGains();

    m_drloop.poll();

  }

  public void updateOdometry() {

    // SmartDashboard.putString(("EstPose"), getEstimatedPose().toString());
    // SmartDashboard.putString(("FLModPos"), m_frontLeft.getPosition().toString());
    // SmartDashboard.putNumber(("FLDrPos"), m_frontLeft.getDrivePosition());

    if (checkCANOK()) {

      /** Updates the field relative position of the robot. */

      m_poseEstimator.update(
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_backLeft.getPosition(),
              m_backRight.getPosition()
          });

      if (visionDataAvailable) {

        m_poseEstimator.addVisionMeasurement(

            visionPoseEstimatedData,

            Timer.getFPGATimestamp() - latencyMs);
      }
    }
  }

  public Pose2d getEstimatedPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void getVisionCorrection(Pose2d pose, double latency) {
    visionPoseEstimatedData = pose;
    latencyMs = latency;
  }

  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(getHeadingRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition() },
        pose);
    m_gyro.reset();

  }

  public void setAngleAdjustment(double adjustment) {
    m_gyro.setAngleAdjustment(adjustment);
  }

  public double getAngleAdjustment() {
    return m_gyro.getAngleAdjustment();
  }

  public double getHeadingDegrees() {
    return -Math.IEEEremainder((m_gyro.getAngle()), 360);

  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public float getGyroPitch() {
    return m_gyro.getPitch();
  }

  public float getGyroRoll() {
    return m_gyro.getRoll();
  }

  public boolean checkCANOK() {
    return RobotBase.isSimulation() ||
        m_frontLeft.checkCAN()
            && m_frontRight.checkCAN()
            && m_backLeft.checkCAN()
            && m_backLeft.checkCAN();

  }

  public void resetModuleEncoders() {
    m_frontLeft.resetAngleToAbsolute();
    m_frontRight.resetAngleToAbsolute();
    m_backLeft.resetAngleToAbsolute();
    m_backRight.resetAngleToAbsolute();
  }

  /** Zeroes the heading of the robot. */
  public void resetGyro() {
    m_gyro.reset();
    // m_gyro.setAngleAdjustment(0);

  }

  public Translation2d getTranslation() {
    return getEstimatedPose().getTranslation();
  }

  public double getX() {
    return getTranslation().getX();
  }

  public double getY() {
    return getTranslation().getY();
  }

  public double reduceRes(double value, int numPlaces) {
    double n = Math.pow(10, numPlaces);
    return Math.round(value * n) / n;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // public double getTurnRate() {
  // return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  // }

  public void setIdleMode(boolean brake) {

    m_frontLeft.setDriveBrakeMode(brake);
    m_frontLeft.setTurnBrakeMode(brake);
    m_frontRight.setDriveBrakeMode(brake);
    m_frontRight.setTurnBrakeMode(brake);
    m_backLeft.setDriveBrakeMode(brake);
    m_backLeft.setTurnBrakeMode(brake);
    m_backRight.setDriveBrakeMode(brake);
    m_backRight.setTurnBrakeMode(brake);

  }

  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  @Override
  public void simulationPeriodic() {

    ChassisSpeeds chassisSpeedSim = m_kinematics.toChassisSpeeds(

        new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        });
    // want to simulate navX gyro changing as robot turns
    // information available is radians per second and this happens every 20ms
    // radians/2pi = 360 degrees so 1 degree per second is radians / 2pi
    // increment is made every 20 ms so radian adder would be (rads/sec) *(20/1000)
    // degree adder would be radian adder * 360/2pi
    // so degree increment multiplier is 360/100pi = 1.1459

    double temp = chassisSpeedSim.omegaRadiansPerSecond * 1.1459155;
    SmartDashboard.putNumber("CHSSM", chassisSpeedSim.omegaRadiansPerSecond);
    temp += m_simAngle.get();

    m_simAngle.set(temp);

    Unmanaged.feedEnable(20);
  }

  public void jogTurnModule(SwerveModuleSM i, double speed) {
    i.turnMotorMove(speed);
  }

  public void positionTurnModule(SwerveModuleSM i, double angle) {
    i.positionTurn(angle);
  }

  public void driveModule(SwerveModuleSM i, double speed) {
    i.driveMotorMove(speed);
  }

  public boolean getTurnInPosition(SwerveModuleSM i, double targetAngle) {
    return i.turnInPosition(targetAngle);
  }

  public double getAnglefromThrottle() {

    return 180 * throttleValue;
  }

  public PIDController getXPID() {
    return xPID;
  }

  public PIDController getYPID() {
    return yPID;
  }

  public PIDController getThetaPID() {
    return thetaPID;
  }

  public void setRunTrajectory() {
    runTrajectory = true;
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            resetOdometry(traj.getInitialHolonomicPose());
          }
        }),
        new PPSwerveControllerCommand(

            traj,

            this::getEstimatedPose, // Pose supplier

            m_kinematics, // SwerveDriveKinematics

            getXPID(),

            getYPID(),

            getThetaPID(),

            this::setModuleStates, // Module states consumer

            this // Requires this drive subsystem
        ),

        new InstantCommand(() -> stopModules()));
  }

  public void tuneXPIDGains() {

    if (xPID.getP() != Pref.getPref("PPXkP"))
      xPID.setP(Pref.getPref("PPXkP"));

    if (xPID.getI() != Pref.getPref("PPXkI"))
      xPID.setI(Pref.getPref("PPXkI"));

    if (xPID.getD() != Pref.getPref("PPXkD"))
      xPID.setD(Pref.getPref("PPXkD"));

  }

  public void tuneYPIDGains() {

    if (yPID.getP() != Pref.getPref("PPYkP"))
      yPID.setP(Pref.getPref("PPYkP"));

    if (yPID.getI() != Pref.getPref("PPYkI"))
      yPID.setI(Pref.getPref("PPYkI"));

    if (yPID.getD() != Pref.getPref("PPYkD"))
      yPID.setD(Pref.getPref("PPYkD"));

  }

  public void tuneThetaPIDGains() {

    if (thetaPID.getP() != Pref.getPref("PPThetakP"))
      thetaPID.setP(Pref.getPref("PPThetakP"));

    if (thetaPID.getI() != Pref.getPref("PPThetakI"))
      thetaPID.setI(Pref.getPref("PPThetakI"));

    if (thetaPID.getP() != Pref.getPref("PPThetakD"))
      thetaPID.setD(Pref.getPref("PPThetakD"));

  }

}