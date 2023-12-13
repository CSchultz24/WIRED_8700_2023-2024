// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.PDPConstants;
import frc.robot.Constants.PPConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.Pref;
public class DriveSubsystem extends SubsystemBase {

  public SwerveDriveKinematics m_kinematics = DriveConstants.m_kinematics;

  public boolean isOpenLoop = true;// RobotBase.isSimulation() && !DriverStation.isAutonomousEnabled();

  final PowerDistribution m_pdp = new PowerDistribution();
 
  public final SwerveModuleSM m_frontLeft = new SwerveModuleSM(
      IDConstants.FRONT_LEFT_LOCATION,
      CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
      CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
      CanConstants.FRONT_LEFT_MODULE_STEER_CANCODER,
      DriveConstants.kFrontLeftDriveMotorReversed,
      DriveConstants.kFrontLeftTurningMotorReversed,
      PDPConstants.FRONT_LEFT_DRIVE_CHANNEL,
      PDPConstants.FRONT_LEFT_TURN_CHANNEL,
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
      CanConstants.BACK_RIGHT_MODULE_STEER_OFFSET);

  // The gyro sensor

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 100);

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

  private boolean showOnShuffleboard;// set in RobotContainer using set function

  public SimDouble m_simAngle;// navx sim

  public double throttleValue;

  public double targetAngle;

  public boolean m_fieldOriented = false;

  public Pose2d activeTagPose = new Pose2d();

  public SimpleMotorFeedforward rotsff = new SimpleMotorFeedforward(.06, .2);

  public PIDController rotatePID = new PIDController(DriveConstants.kTurnP,
      DriveConstants.kTurnI, DriveConstants.kTurnD);

  public PIDController xPID = new PIDController(
      PPConstants.kPXController, PPConstants.kIXController, PPConstants.kIXController); // X

  public PIDController yPID = new PIDController(PPConstants.kPYController, PPConstants.kIYController,
      PPConstants.kDYController);

  public PIDController thetaPID = new PIDController(PPConstants.kPThetaController, PPConstants.kIThetaController,
      PPConstants.kDThetaController);

  public boolean runTrajectory;

  /**
   * All vision related variables
   * 
   */

  public boolean inhibitVision;

  private double lastVisionUpdateTime;

  LimelightResults llresults;

  public Pose2d botPose = new Pose2d();

  public int numberTags;
  public boolean hasTag;
  public int fiducialID;

  public int tagid1;
  public int tagid2;

  public int numberTargets;
  public boolean hasTarget;
  public double tx;
  public double ty;
  public double ts;
  public double targetArea;
  public double latencyCaptureMs;
  public double latencyPipelineMs;
  public double boundBoxHeight;;
  public double boundingBoxWidth;
  public double boundingBoxLongestSide;
  public double boundingBoxShortestSide;

  public boolean allianceBlue;

  private final double visionTimeSet = 1;

  private final double visRobDiagErrLimit = .25;

  private final double xyErrLimit = .05;

  private final double rotErrLimit = .5;

  private double endYTarget;

  private double endXTarget;

  public boolean isPipe;

  public Field2d m_Field2d;

  public boolean isRotating;

  public boolean trajectoryRunning;

  public boolean limelightExists;

  public double neuraClassID;

  public double neuralClassID;

  public boolean coneFound;

  public boolean cubeFound;

  public boolean ALL_CANOK;

  public double fieldOrientOffset = 180;

  public float gyroStartPitch;;

  public int moduleFaultSeen;

  public int moduleStickyFaultSeen;

  private boolean firstCorrection;

  public double pitchRateOfChange;

  public double[] measuredStates = { 0, 0, 0, 0, 0, 0, 0, 0 };

  public double[] desiredStates = { 0, 0, 0, 0, 0, 0, 0, 0 };

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    m_gyro.reset();

    resetModuleEncoders();

    setIdleMode(true);

    // m_fieldOriented = true;

    setInhibitVisionCorrection(false);

    if (RobotBase.isSimulation()) {

      var dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

      m_simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")));

      rotatePID.setP(.004);

      thetaPID.setP(.005);

    }
    resetEncoders();
    // m_Field2d = new Field2d();

    // SmartDashboard.putData(m_Field2d);

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
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, get180Rotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

  }

  /**
   * Sets swerve module states using Chassis Speeds.
   *
   * @param chassisSpeeds The desired Chassis Speeds
   */
  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = DriveConstants.m_kinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(swerveModuleStates);
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

  public void setClosedLoop(boolean on) {
    m_frontLeft.m_isOpenLoop = !on;
    m_frontRight.m_isOpenLoop = !on;
    m_backLeft.m_isOpenLoop = !on;
    m_backRight.m_isOpenLoop = !on;
    isOpenLoop = !on;
  }

  public void setAngleCorrection(double value) {
    m_frontLeft.angleCorrection = value;
    m_frontRight.angleCorrection = -value;
    m_backLeft.angleCorrection = -value;
    m_backRight.angleCorrection = value;
  }

  public void clearAngleCorrection() {
    m_frontLeft.angleCorrection = 0;
    m_frontRight.angleCorrection = 0;
    m_backLeft.angleCorrection = 0;
    m_backRight.angleCorrection = 0;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block

    SmartDashboard.putBoolean("TMCO", m_frontLeft.turnCoderConnected);


    // SmartDashboard.putNumber("TurnFL", m_frontLeft.angleCorrection);
    // SmartDashboard.putNumber("TurnFR", m_frontRight.angleCorrection);

    updateOdometry();

    if (DriverStation.isEnabled()) {

      desiredStates[0] = m_frontLeft.getDesiredState()[0];
      desiredStates[1] = m_frontLeft.getDesiredState()[1];
      desiredStates[2] = m_frontRight.getDesiredState()[0];
      desiredStates[3] = m_frontRight.getDesiredState()[1];
      desiredStates[4] = m_backLeft.getDesiredState()[0];
      desiredStates[5] = m_backLeft.getDesiredState()[1];
      desiredStates[6] = m_backRight.getDesiredState()[0];
      desiredStates[7] = m_backRight.getDesiredState()[1];

      measuredStates[0] = m_frontLeft.getMeasuredState()[0];
      measuredStates[1] = m_frontLeft.getMeasuredState()[1];
      measuredStates[2] = m_frontRight.getMeasuredState()[0];
      measuredStates[3] = m_frontRight.getMeasuredState()[1];
      measuredStates[4] = m_backLeft.getMeasuredState()[0];
      measuredStates[5] = m_backLeft.getMeasuredState()[1];
      measuredStates[6] = m_backRight.getMeasuredState()[0];
      measuredStates[7] = m_backRight.getMeasuredState()[1];

    }
    SmartDashboard.putNumberArray("swerve/desiredStates", desiredStates);
    SmartDashboard.putNumberArray("swerve/measuredStates", measuredStates);

    if (moduleFaultSeen == 0) {
      moduleFaultSeen = m_frontLeft.getFaults() + m_frontRight.getFaults() + m_backLeft.getFaults()
          + m_backRight.getFaults();
      if (moduleStickyFaultSeen == 0) {
        moduleStickyFaultSeen = m_frontLeft.getStickyFaults() + m_frontRight.getStickyFaults()
            + m_backLeft.getStickyFaults() + m_backRight.getStickyFaults();
      }
    }
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /*
   * the pitch values may be robot dependent. Someone suggested the slow speed ~1
   * ft/s.
   * 
   * For us the fast speed was ~7 ft/s.
   * 
   * Also, you should zero your heading in auto init.
   * 
   * 
   * 
   */
  public CommandBase autoBalance() {
    return Commands.race(
        Commands.sequence(
            Commands.run(
                () -> this.drive(Pref.getPref("balancerate") * DriveConstants.kMaxSpeedMetersPerSecond,
                    0, 0),
                this).until(() -> this.getCompedGyroPitch() >= Pref.getPref("balancehigh")),
            Commands.run(
                () -> this.drive(0.2 * DriveConstants.kMaxSpeedMetersPerSecond,
                    0, 0),
                this).until(() -> this.getCompedGyroPitch() <= Pref.getPref("balancelow")),
            Commands.run(this::setX, this)),
        Commands.waitSeconds(15));
    // Commands.run(
    // ()->this.drive(0,0,0,true,true),this));
  }

  public double[] getPoseAsDoubles(Pose2d pose) {
    return r2dToArray(pose);
  }

  public void updateOdometry() {

    if (true) {

      /** Updates the field relative position of the robot. */

      m_poseEstimator.update(
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_backLeft.getPosition(),
              m_backRight.getPosition()
          });

      if (RobotBase.isReal() && !inhibitVision && numberTags > 0

          && (getAddVisionMeasurementByTime(visionTimeSet, visRobDiagErrLimit))

          && botPose != new Pose2d()) {

        m_poseEstimator.addVisionMeasurement(

            botPose,

            Timer.getFPGATimestamp() - latencyPipelineMs);
      }
    }
  }

  /**
   * 
   * ll vision related methods
   * 
   * @param on
   */
  public void setInhibitVisionCorrection(boolean on) {
    inhibitVision = on;
  }

  public boolean getInhibitVisionCorrection() {
    return inhibitVision;
  }

  public double getDiagVisionDifference() {
    return Math.hypot(getX() - botPose.getX(), getY() - botPose.getY());
  }

  // run vision correction on set seconds
  public boolean getAddVisionMeasurementByTime(double len, double diagDistLimit) {
    if ((getAddVisionMeasurementByDistance(diagDistLimit)) && Timer.getFPGATimestamp() > lastVisionUpdateTime + len) {
      lastVisionUpdateTime = Timer.getFPGATimestamp();
      firstCorrection = false;
    }
    return true;
  }

  public boolean getAddVisionMeasurementByDistance(double diagDistLimit) {
    return getDiagVisionDifference() > diagDistLimit;
  }

  public Pose2d getEstimatedPosition() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_gyro.reset();
    m_poseEstimator.resetPosition(getHeadingRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition() },
        pose);

  }

  public void resetEncoders() {

    m_frontLeft.resetEncoders();

    m_frontRight.resetEncoders();

    m_backLeft.resetEncoders();

    m_backRight.resetEncoders();

  }

  public boolean getAllianceBlue() {
    return (DriverStation.getAlliance() == Alliance.Blue);
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

  public Rotation2d get180Rotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees() - fieldOrientOffset);
  }

  public float getGyroPitch() {
    return m_gyro.getPitch();
  }

  public float getCompedGyroPitch() {
    return m_gyro.getPitch() - gyroStartPitch;
  }

  public float getGyroRoll() {
    return m_gyro.getRoll();
  }

  public Rotation2d getGyroR2d() {
    //
    return m_gyro.getRotation2d();

  }

  public boolean checkCANOK() {

    return RobotBase.isSimulation()

        || m_frontLeft.checkCAN()
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
    return getEstimatedPosition().getTranslation();
  }

  public double getX() {
    return getEstimatedPosition().getX();
  }

  public double getY() {
    return getEstimatedPosition().getY();
  }

  public double reduceRes(double value, int numPlaces) {
    double n = Math.pow(10, numPlaces);
    return Math.round(value * n) / n;
  }

  public void setActiveTagPose(Pose2d pose) {
    activeTagPose = pose;
  }

  public Pose2d getActiveTagPose() {
    return activeTagPose;

  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // public double getTurnRate() {
  // return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  // }

  public boolean isbraked() {
    return m_frontLeft.m_driveMotor.getIdleMode() == IdleMode.kBrake;
  }

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

  public void setSMIsRotating() {
    m_frontLeft.isRotating = true;
    m_frontRight.isRotating = true;
    m_backLeft.isRotating = true;
    m_backRight.isRotating = true;
  }

  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public Command clearFaults() {
    moduleFaultSeen = 0;
    moduleStickyFaultSeen = 0;
    m_pdp.clearStickyFaults();
    return Commands.sequence(Commands.runOnce(() -> m_frontLeft.clearFaults()),
        Commands.runOnce(() -> m_frontRight.clearFaults()),
        Commands.runOnce(() -> m_backLeft.clearFaults()),
        Commands.runOnce(() -> m_backRight.clearFaults()));
  }

  public double[] r2dToArray(Pose2d pose) {
    double[] temp = { 0, 0, 0 };
    temp[0] = pose.getX();
    temp[1] = pose.getY();
    temp[2] = pose.getRotation().getDegrees();
    return temp;
  }

  public boolean posesCompare(double[] resulta, double[] resultb, double[] resultc) {
    double[] tempa = resulta;
    double[] tempb = resultb;
    double[] tempc = resultc;

    boolean resultx = Math.abs(tempa[0] - tempb[0]) < xyErrLimit;
    boolean resulty = Math.abs(tempa[1] - tempb[1]) < xyErrLimit;
    boolean resultR = Math.abs(tempa[2] - tempb[2]) < rotErrLimit;

    // SmartDashboard.putBoolean("OKX", resultx);
    // SmartDashboard.putBoolean("OKY", resulty);
    // SmartDashboard.putBoolean("OKr", resultR);

    if (resultx && resulty && resultR) {
      resultx = Math.abs(tempa[0] - tempc[0]) < xyErrLimit;
      resulty = Math.abs(tempa[1] - tempc[1]) < xyErrLimit;
      resultR = Math.abs(tempa[2] - tempc[2]) < rotErrLimit;

    }

    return resultx && resulty && resultR;
  }

  public Pose2d getAvePose(Pose2d[] poses) {
    Pose2d[] temp = new Pose2d[3];
    temp = poses;
    double[] tempa = r2dToArray(temp[0]);
    double[] tempb = r2dToArray(temp[1]);
    double[] tempc = r2dToArray(temp[2]);

    double aveX = (tempa[0] + tempb[0] + tempc[0]) / 3;
    double aveY = (tempa[1] + tempb[1] + tempc[1]) / 3;
    double aveR = (tempa[2] + tempb[1] + tempc[1]) / 3;

    temp[0] = new Pose2d(aveX, aveY, new Rotation2d(Units.degreesToRadians(aveR)));

    return temp[0];
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

    /**
     * Need to find the degree change in 20 ms from angular radians per second
     * 
     * So radsper20ms = radspersec/50
     * degrees per rad = 360/2PI=57.3
     * degrees per 20ms = radsper20ms * degrees per rad
     * conversion is 57.3/50=114.6/100=1.15
     */

    double temp = chassisSpeedSim.omegaRadiansPerSecond * 1.15;
    // SmartDashboard.putNumber("CHSSM", chassisSpeedSim.omegaRadiansPerSecond);
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

  // public void driveModule(SwerveModuleSM i, double speed) {
  // i.driveMotorMove(speed);
  // }

  public boolean getTurnInPosition(SwerveModuleSM i, double targetAngle) {
    return i.turnInPosition(targetAngle);
  }

  public PIDController getRotatePID() {
    return rotatePID;
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

  public void setEndX(double xval) {
    endXTarget = xval;
  }

  public double getEndX() {
    return endXTarget;

  }

  public void setEndY(double yval) {
    endYTarget = yval;
  }

  public double getEndY() {
    return endYTarget;
  }

  public void setShowOnShuffleboard(boolean on) {
    showOnShuffleboard = on;
  }

  public void setIsRotating(boolean on) {
    isRotating = on;
  }

  public void tuneYPIDGains() {

    if (yPID.getP() != Pref.getPref("PPYkP"))
      yPID.setP(Pref.getPref("PPYkP"));

    if (yPID.getI() != Pref.getPref("PPYkI"))
      yPID.setI(Pref.getPref("PPYkI"));

    if (yPID.getD() != Pref.getPref("PPYkD"))
      yPID.setD(Pref.getPref("PPYkD"));

  }

  public void tuneXPIDGains() {

    if (xPID.getP() != Pref.getPref("PPXkP"))
      xPID.setP(Pref.getPref("PPXkP"));

    if (xPID.getI() != Pref.getPref("PPXkI"))
      xPID.setI(Pref.getPref("PPXkI"));

    if (xPID.getD() != Pref.getPref("PPXkD"))
      xPID.setD(Pref.getPref("PPXkD"));

  }

  public void tuneThetaPIDGains() {

    if (thetaPID.getP() != Pref.getPref("PPThetakP"))
      thetaPID.setP(Pref.getPref("PPThetakP"));

    if (thetaPID.getI() != Pref.getPref("PPThetakI"))
      thetaPID.setI(Pref.getPref("PPThetakI"));

    if (thetaPID.getP() != Pref.getPref("PPThetakD"))
      thetaPID.setD(Pref.getPref("PPThetakD"));

  }

  public void tuneRotatePIDGains() {

    if (rotatePID.getP() != Pref.getPref("PPRotatekP"))
      rotatePID.setP(Pref.getPref("PPThetakP"));

    if (rotatePID.getI() != Pref.getPref("PPRotatekI"))
      rotatePID.setI(Pref.getPref("PPThetakI"));

    if (rotatePID.getP() != Pref.getPref("PPRotatekD"))
      rotatePID.setD(Pref.getPref("PPThetakD"));

  }

  public double getAnglefromThrottle() {
    return 0;
  }

  public double getGyrocompedPitch() {
    return 0;
  }
}