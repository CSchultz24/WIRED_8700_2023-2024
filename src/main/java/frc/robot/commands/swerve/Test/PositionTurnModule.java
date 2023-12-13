package frc.robot.commands.swerve.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModuleSM;

public class PositionTurnModule extends CommandBase{
    
    private DriveSubsystem m_drive;

    private SwerveModuleSM m_n;
  
    /** Creates a new PositionTurnModule. */
    public PositionTurnModule(DriveSubsystem drive, SwerveModuleSM n) {
      // Use addRequirements() here to declare subsystem dependencies.
      m_drive = drive;
  
      m_n = n;
  
      addRequirements(m_drive);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
  
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      m_drive.targetAngle = m_drive.getAnglefromThrottle();
      //SmartDashboard.putNumber("TargetAngle", m_drive.targetAngle);
      m_n.positionTurn(m_drive.targetAngle);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_n.positionTurn(0.);
  
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return m_drive.getTurnInPosition(m_n, m_drive.targetAngle);
    }
}
