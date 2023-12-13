package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TeleopRoutines.RotateToAngle;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.commands.swerve.SetSwerveDriveSlow;
import frc.robot.commands.swerve.ToggleFieldOriented;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.TrajectoryFactory;

public class RobotContainer {
    
        // The robot's subsystems
        final DriveSubsystem m_drive;

        public TrajectoryFactory m_tf;

        public FieldSim m_fieldSim = null;

        // The driver, codriver and arm controllers

        public CommandXboxController m_driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                m_drive = new DriveSubsystem();

                Pref.deleteUnused();

                Pref.addMissing();

                SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

                m_fieldSim = new FieldSim(m_drive);

                m_fieldSim.initSim();

                setDefaultCommands();

                configDriverButtons();

        }

        private void setDefaultCommands() {

                m_drive.setDefaultCommand(getDriveCommand());

        }

        void configDriverButtons() {

                m_driverController.leftTrigger()
                                .whileTrue(getSlowDriveCommand());

                // m_driverController.rightTrigger().whileTrue(new
                // IntakePieceStopMotor(m_intake, 11));

                // m_driverController.rightBumper().whileTrue(new EjectPieceFromIntake(m_intake,
                // 10));

                // m_driverController.leftBumper().whileTrue(new EjectPieceFromIntake(m_intake,
                // 5));

                // m_driverController.a()

                // m_driverController.b().onTrue(
                // deliverPositionsCommand(2).withTimeout(15));

                m_driverController.x().onTrue(new ToggleFieldOriented(m_drive));

                // m_driverController.y()
                // .onTrue(new LoadStationPositions(m_liftArm, m_wrist, m_extendArm, m_intake)
                // .withTimeout(10))
                // .onTrue(new IntakePieceStopMotor(m_intake, 11));

                m_driverController.back()
                                .onTrue(new InstantCommand(() -> m_drive.resetGyro()));

                // m_driverController.start()
                // .onTrue(new RetractWristExtendLiftHome(m_liftArm, m_extendArm, m_wrist));

                // m_driverController.back()

                // m_driverController.povUp().onTrue(
                // Commands.runOnce(() -> m_liftArm.incGoal(.25)));

                // m_driverController.povDown()
                // .onTrue(Commands.runOnce(() -> m_liftArm.incGoal(-.25)));

                // m_driverController.povLeft()
                // .onTrue(Commands.runOnce(() -> m_extendArm.incGoal(.25)));

                // m_driverController.povRight()
                // .onTrue(Commands.runOnce(() -> m_extendArm.incGoal(-.25)));

        }

        public Command getDriveCommand() {
                return new SetSwerveDrive(m_drive,
                                () -> m_driverController.getRawAxis(1),
                                () -> m_driverController.getRawAxis(0),
                                () -> m_driverController.getRawAxis(4));

        }

        public Command getSlowDriveCommand() {
                return new SetSwerveDriveSlow(m_drive,
                                () -> m_driverController.getRawAxis(1),
                                () -> m_driverController.getRawAxis(0),
                                () -> m_driverController.getRawAxis(4));

        }

        public Command getRotateCommand(double angle) {

                return new RotateToAngle(m_drive, angle);
        }

        public void simulationPeriodic() {

                m_fieldSim.periodic();
        }

        public void periodic() {
                m_fieldSim.periodic();
                // m_pt.update();

        }
    
}