package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends CommandBase{
    private DriveSubsystem swerve;
    private boolean isRelative;
    private double goal;
    private HolonomicDriveController holonomicDriveController = new HolonomicDriveController(new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            new ProfiledPIDController(Constants.SwerveTransformPID.PID_TKP / 2,
                    Constants.SwerveTransformPID.PID_TKI, Constants.SwerveTransformPID.PID_TKD,
                    new TrapezoidProfile.Constraints(Constants.SwerveTransformPID.MAX_ANGULAR_VELOCITY,
                            Constants.SwerveTransformPID.MAX_ANGULAR_ACCELERATION)));
    private Pose2d startPos = new Pose2d();
    private Pose2d targetPose2d = new Pose2d();
    private int finishCounter = 0;

    /**
     * Turns robot to specified angle. Uses absolute rotation on field.
     *
     * @param swerve     Swerve subsystem
     * @param angle      Requested angle to turn to
     * @param isRelative Whether the angle is relative to the current angle: true =
     *                   relative, false
     *                   = absolute
     */
    public TurnToAngle(DriveSubsystem swerve, double angle, boolean isRelative) {
        addRequirements(swerve);
        this.swerve = swerve;
        this.goal = angle;
        this.isRelative = isRelative;
        holonomicDriveController.setTolerance(new Pose2d(1, 1, Rotation2d.fromDegrees(1)));

    }

    @Override
    public void initialize() {
        startPos = swerve.getEstimatedPosition();
        if (isRelative) {
            targetPose2d = new Pose2d(startPos.getTranslation(),
                    startPos.getRotation().rotateBy(Rotation2d.fromDegrees(goal)));
        } else {
            targetPose2d = new Pose2d(startPos.getTranslation(), Rotation2d.fromDegrees(goal));
        }
        // if (DriverStation.getAlliance() == Alliance.Red) {
        //     targetPose2d = new Pose2d(targetPose2d.getTranslation(),
        //             targetPose2d.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        // }
    }

    @Override
    public void execute() {
        Pose2d currPose2d = swerve.getEstimatedPosition();
        ChassisSpeeds chassisSpeeds = this.holonomicDriveController.calculate(currPose2d,
                targetPose2d, 0, targetPose2d.getRotation());
        swerve.setModuleStates(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupt) {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        if (holonomicDriveController.atReference()) {
            finishCounter++;
        } else {
            finishCounter = 0;
        }
        return finishCounter > 2;
    }
}
