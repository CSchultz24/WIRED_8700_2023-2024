package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModuleSM;

public class FieldSim {
    private final DriveSubsystem m_swerveDrive;

    private SwerveModuleSM[] modules = new SwerveModuleSM[4];
  
    private final Field2d m_field2d = new Field2d();
  
    private final Pose2d[] m_swerveModulePoses = new Pose2d[4];
  
    private double xVal;
  
    private double yVal;
  
    private Rotation2d r2d;
  
    public FieldSim(DriveSubsystem swerveDrive) {
  
      m_swerveDrive = swerveDrive;
  
  
      modules[0] = m_swerveDrive.m_frontLeft;
      modules[1] = m_swerveDrive.m_frontRight;
      modules[2] = m_swerveDrive.m_backLeft;
      modules[3] = m_swerveDrive.m_backRight;
  
    }
  
    public void initSim() {
      SmartDashboard.putData("Field2d", m_field2d);
    }
  
    public Field2d getField2d() {
  
      return m_field2d;
    }
  
    private void updateRobotPoses() {
  
      m_field2d.setRobotPose(m_swerveDrive.getEstimatedPosition());
  
      for (int i = 0; i < 4; i++) {
  
        Translation2d updatedPositions = DriveConstants.kModuleTranslations[i]
  
            .rotateBy(m_swerveDrive.getEstimatedPosition().getRotation())
  
            .plus(m_swerveDrive.getEstimatedPosition().getTranslation());
  
        m_swerveModulePoses[i] = new Pose2d(
            updatedPositions,
            modules[i]
                .getHeadingRotation2d()
                .plus(m_swerveDrive.getHeadingRotation2d()));
  
      }
  
      m_field2d.getObject("Swerve Modules")
  
          .setPoses(m_swerveModulePoses);
  
      if (DriverStation.getAlliance() == Alliance.Blue) {
  
        m_field2d.getObject("AprilTags")
            .setPoses(SimConstants.Tags.aprilTagsBlue);
  
      }
  
      if (DriverStation.getAlliance() == Alliance.Red) {
  
        m_field2d.getObject("AprilTags")
            .setPoses(SimConstants.Tags.aprilTagsRed);
      }
  
    }
  
    public void periodic() {
  
      updateRobotPoses();
  
      // simulationPeriodic();
  
      // SmartDashboard.putData("Field2d", m_field2d);
    }
  
    public void simulationPeriodic() {
    }  
}
