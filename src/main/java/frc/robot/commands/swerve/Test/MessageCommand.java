package frc.robot.commands.swerve.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class MessageCommand extends InstantCommand{
    private String m_message;

    public MessageCommand(String message) {
      // Use addRequirements() here to declare subsystem dependencies.
      m_message = message;
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    //  SmartDashboard.putString("ActiveDrop", m_message);
    }
}
