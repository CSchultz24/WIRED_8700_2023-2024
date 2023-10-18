package frc.robot.simulation;

import java.util.Collection;
import java.util.Iterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants2023;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModuleSM;

public class FieldSim {
    private final DriveSubsystem m_swerveDrive;

  private SwerveModuleSM[] modules = new SwerveModuleSM[4];

  private final Field2d m_field2d = new Field2d();

  private final Pose2d[] m_swerveModulePoses = new Pose2d[4];

  private final Pose2d[] aprilTags = new Pose2d[8];

  private double xVal;

  private double yVal;

  private Rotation2d r2d;

  private Pose3d[] tagArray;

  public FieldSim(DriveSubsystem swerveDrive) {

    m_swerveDrive = swerveDrive;

    modules[0] = m_swerveDrive.m_frontLeft;
    modules[1] = m_swerveDrive.m_frontRight;
    modules[2] = m_swerveDrive.m_backLeft;
    modules[3] = m_swerveDrive.m_backRight;

    Collection<Pose3d> values = FieldConstants2023.aprilTags.values();

    tagArray = values.toArray(new Pose3d[0]);

    for (int n = 0; n < 8; n++) {
      double xVal = tagArray[n].getX();
      double yVal = tagArray[n].getY();
      Rotation2d r2d = tagArray[n].getRotation().toRotation2d();
      aprilTags[n] = new Pose2d(xVal, yVal, r2d);

    }

  }

  public void initSim() {
  }

  public Field2d getField2d() {

    return m_field2d;
  }

  private void updateRobotPoses() {

    Pose2d testing = m_swerveDrive.getEstimatedPose();

    m_field2d.setRobotPose(m_swerveDrive.getEstimatedPose());

    for (int i = 0; i < 4; i++) {

      Translation2d updatedPositions = DriveConstants.kModuleTranslations[i]

          .rotateBy(m_swerveDrive.getEstimatedPose().getRotation())

          .plus(m_swerveDrive.getEstimatedPose().getTranslation());

      m_swerveModulePoses[i] = new Pose2d(
          updatedPositions,
          modules[i]
              .getHeadingRotation2d()
              .plus(m_swerveDrive.getHeadingRotation2d()));

    }

    m_field2d.getObject("Swerve Modules")

        .setPoses(m_swerveModulePoses);

    m_field2d.getObject("AprilTags")
        .setPoses(aprilTags);

  }

  public void periodic() {

    updateRobotPoses();

    if (RobotBase.isSimulation())

      simulationPeriodic();

    SmartDashboard.putData("Field2d", m_field2d);
  }

  public void simulationPeriodic() {
  }
}
