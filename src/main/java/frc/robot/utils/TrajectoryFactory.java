package frc.robot.utils;

import java.io.File;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryFactory {
    
    private DriveSubsystem m_drive;

    private FieldSim m_fs;

    private boolean tune;

    public boolean run;

    public boolean runTagTraj;

    public Pose2d activeTagPose;

    public Pose2d endPose = new Pose2d();

    public double loadStationYOffset = .25;

    public Translation2d leftTranslation = new Translation2d(1.25, loadStationYOffset);

    public Translation2d rightTranslation = new Translation2d(1.25, -loadStationYOffset);

    private Translation2d activeTranslation;

    public Rotation2d endRotation = Rotation2d.fromDegrees(180);

    public boolean showSelected;

    public boolean showLoadTraj;

    public Pose2d startLoadPose;

    

    public TrajectoryFactory(DriveSubsystem drive, FieldSim fs) {

        m_drive = drive;
        m_fs = fs;



    }



    public PathPlannerTrajectory getPathPlannerTrajectory(String pathName, double maxvel, double maxaccel,
            boolean reversed) {
        // checkFileExists(pathName);
        PathPlannerTrajectory ppTrajectory = PathPlanner.loadPath(pathName, maxvel, maxaccel, reversed);
        return ppTrajectory;
    }

    public List<PathPlannerTrajectory> getPathPlannerTrajectoryGroup(String pathName, double maxvel, double maxaccel,
            boolean reversed) {
        List<PathPlannerTrajectory> paths = PathPlanner.loadPathGroup(pathName, maxvel, maxaccel, reversed);
        return paths;
    }

    public boolean checkFileExists(String name) {
        File deployDirectory = Filesystem.getDeployDirectory();
        String pathName = deployDirectory.toString() + "\\" + name + ".path";

        // SmartDashboard.putString("Name", pathName);
        File f = new File(pathName);

        // SmartDashboard.putBoolean("Exists", f.exists());// && !f.isDirectory()));
        return f.exists() && !f.isDirectory();
    }

    /**
     * 
     * Yes. It’s a bit confusing but heading refers to the direction of travel and
     * the holonomic rotation is the direction you want the robot to face. When not
     * using holonomic mode, heading refers to both since they can’t be different.
     * 
     * 
     * 
     * @param traj
     * @param isFirstPath
     * @return
     */
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {

                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        PathPlannerTrajectory transformed = PathPlannerTrajectory.transformTrajectoryForAlliance(traj,
                                DriverStation.getAlliance());
                        // m_drive.resetGyro();
                        m_drive.resetOdometry(transformed.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(

                        traj,

                        m_drive::getEstimatedPosition, // Pose supplier

                        m_drive.m_kinematics, // SwerveDriveKinematics

                        m_drive.getXPID(),

                        m_drive.getYPID(),

                        m_drive.getThetaPID(),

                        m_drive::setModuleStates, // Module states consumer

                        true,

                        m_drive // Requires this drive subsystem
                ),

                new InstantCommand(() -> m_drive.stopModules()),

                new ParallelCommandGroup(

                        new InstantCommand(() -> run = false),

                        new InstantCommand(() -> m_drive.setInhibitVisionCorrection(false)),

                        new InstantCommand(() -> runTagTraj = false)));
    }

    public void setActiveTagPose(Pose2d pose) {

        activeTagPose = pose;
    }

    public void clearTrajectory() {
        PathPlannerTrajectory traj = new PathPlannerTrajectory();
        m_fs.getField2d().getObject("Traj").setTrajectory(traj);

    }

    public void periodic() {

    }

    public PathPlannerTrajectory getSimpleTraj() {

        // Simple path without holonomic rotation. Stationary start/end. Max velocity of
        // 4 m/s and max accel of 3 m/s^2

        PathPlannerTrajectory traj1 = PathPlanner.generatePath(

                new PathConstraints(2, 2),

                new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0)), // position, heading

                new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45)) // position, heading
        );
        return traj1;
    }

    public PathPlannerTrajectory getSimpleTrajWithHolonomic() {

        // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4
        // m/s and max accel of 3 m/s^2
        PathPlannerTrajectory traj2 = PathPlanner.generatePath(

                new PathConstraints(3, 3),

                new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position,

                new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-90)) // position,

        );

        return traj2;
    }

    // More complex path with holonomic rotation. Non-zero starting velocity of 2
    // m/s. Max velocity of 4 m/s and max accel of 3 m/s^2

    public PathPlannerTrajectory getNonZeroStartTraj() {

        PathPlannerTrajectory traj3 = PathPlanner.generatePath(

                new PathConstraints(4, 3),

                new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 2), // position,

                new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-90)), // position,

                new PathPoint(new Translation2d(5.0, 3.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-30)) // position,

        );
        return traj3;
    }

    public boolean getAllianceBlue() {
        return (DriverStation.getAlliance() == Alliance.Blue);
    }

    public Pose2d getTrajectoryStartPose(String name, boolean reversed) {
        PathPlannerTrajectory ppTrajectory = PathPlanner.loadPath(name, 2, 1, reversed);
        return ppTrajectory.getInitialPose();

    }
}
