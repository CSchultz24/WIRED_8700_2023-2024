package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ToggleFieldOriented extends InstantCommand {
    private DriveSubsystem m_drive;

  public ToggleFieldOriented(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.m_fieldOriented = !m_drive.m_fieldOriented;
  }
}
