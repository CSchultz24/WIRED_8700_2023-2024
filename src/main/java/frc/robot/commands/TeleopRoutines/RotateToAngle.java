package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToAngle extends PIDCommand {
    private DriveSubsystem m_drive;

  public RotateToAngle(DriveSubsystem drive, double angle) {

    super(
        // The controller that the command will use
        drive.getRotatePID(),
        // This should return the measurement
        () -> drive.getHeadingDegrees(),
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          drive.drive(0, 0, output * 3);
        }, drive);
    m_drive = drive;

    super.m_controller.setTolerance(2);

    super.m_controller.enableContinuousInput(-180, 180);

    // this number could be changed

    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setIsRotating(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.m_controller.atSetpoint();
  }
}
