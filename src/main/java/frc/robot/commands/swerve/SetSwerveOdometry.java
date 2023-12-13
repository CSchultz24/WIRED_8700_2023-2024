package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;

public class SetSwerveOdometry extends CommandBase{
    
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_swerveDrive;

 // private final FieldSim m_fieldSim;
  private final Pose2d m_pose2d;

  /**
   * Sets the robot's position
   *
   * @param swerveDrive Swerve's odometry is set
   * @param pose2d position to set odometry to
   */
  public SetSwerveOdometry(DriveSubsystem swerveDrive, Pose2d pose2d) {
    this(swerveDrive, null, pose2d);
  }

  /**
   * Sets the robot's position
   *
   * @param swerveDrive Swerve's odometry is set
   * @param fieldSim fieldSim to set robot's position if we're simulating the robot
   * @param pose2d position to set odometry to
   */
  public SetSwerveOdometry(DriveSubsystem swerveDrive, FieldSim fieldSim, Pose2d pose2d) {
    // if (RobotBase.isSimulation() && fieldSim == null)
    //   System.out.println(
    //       "SetOdometry Command Error: Robot is in Simulation, but you did not add FieldSim to the argument");

    m_swerveDrive = swerveDrive;
   // m_fieldSim = fieldSim;
    m_pose2d = pose2d;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    m_swerveDrive.resetOdometry(m_pose2d);
    // m_driveTrain.setNavXOffset(m_pose2d.getRotation().getDegrees());
    // if (RobotBase.isSimulation()) m_fieldSim.resetRobotPose(m_pose2d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
