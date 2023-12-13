package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
public class RumbleCommand extends CommandBase{
    
     /** Creates a new RumbleCommand. */
  private XboxController m_controller;
  private RumbleType m_Type;
  private double m_level;
  private int m_time;
  private int loopCtr;

  public RumbleCommand(CommandXboxController controller, RumbleType type, double level, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controller = controller.getHID();
    m_Type = type;
    m_level = level;
    m_time = (int) (time * 50);// 20ms in 1 sec
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopCtr = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    loopCtr++;
    m_controller.setRumble(m_Type, m_level);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controller.setRumble(m_Type, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return loopCtr > m_time;
  }
}
