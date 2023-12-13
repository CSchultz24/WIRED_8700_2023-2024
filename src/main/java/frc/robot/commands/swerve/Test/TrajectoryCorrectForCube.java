package frc.robot.commands.swerve.Test;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryCorrectForCube extends CommandBase{
    private DriveSubsystem m_drive;
    private PIDController m_pidController;
  
    public TrajectoryCorrectForCube(DriveSubsystem drive) {
      // Use addRequirements() here to declare subsystem dependencies.
      m_drive = drive;
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
  
      m_drive.clearAngleCorrection();
  
    //  m_drive.m_fieldOriented = false;
  
      m_pidController = new PIDController(1, 0, 0);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
  
      if (!m_drive.cubeFound)
  
        m_drive.clearAngleCorrection();
  
      else {
  
        double xError = m_pidController.calculate(m_drive.tx, -1.25);
  
       // SmartDashboard.putNumber("XCUBERR", xError);
  
        double temp = xError;
  
        m_drive.setAngleCorrection(temp);
  
      }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_drive.clearAngleCorrection();
      //m_drive.m_fieldOriented = true;
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
