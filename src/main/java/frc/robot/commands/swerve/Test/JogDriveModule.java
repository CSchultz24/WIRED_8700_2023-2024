package frc.robot.commands.swerve.Test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class JogDriveModule extends CommandBase{
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_swerveDrive;
  private boolean m_individual;
  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput, m_testInput;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public JogDriveModule(
      DriveSubsystem swerveDriveSubsystem,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput,
      DoubleSupplier rotationInput, DoubleSupplier testInput, boolean individual) {
    m_swerveDrive = swerveDriveSubsystem;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;
    m_testInput = testInput;
    m_individual = individual;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = MathUtil.applyDeadband(Math.abs(m_throttleInput.getAsDouble()), 0.05)
        * Math.signum(m_throttleInput.getAsDouble());
    double strafe = MathUtil.applyDeadband(Math.abs(m_strafeInput.getAsDouble()), 0.05)
        * Math.signum(m_strafeInput.getAsDouble());
    double rotation = MathUtil.applyDeadband(Math.abs(m_rotationInput.getAsDouble()), 0.05)
        * Math.signum(m_rotationInput.getAsDouble());
    double test = MathUtil.applyDeadband(Math.abs(m_testInput.getAsDouble()), 0.05)
        * Math.signum(m_testInput.getAsDouble());

    throttle *= .5;
    strafe *= .5;
    rotation *= .5;
    test *= .5;

    if (!m_individual) {

      m_swerveDrive.m_frontLeft.driveMotorMoveOpenLoop(throttle);
      m_swerveDrive.m_frontRight.driveMotorMoveOpenLoop(throttle);
      m_swerveDrive.m_backLeft.driveMotorMoveOpenLoop(throttle);
      m_swerveDrive.m_backRight.driveMotorMoveOpenLoop(throttle);

    } else {

      m_swerveDrive.m_frontLeft.driveMotorMoveOpenLoop(throttle);

      m_swerveDrive.m_frontRight.driveMotorMoveOpenLoop(strafe);

      m_swerveDrive.m_backLeft.driveMotorMoveOpenLoop(rotation);

      m_swerveDrive.m_backRight.driveMotorMoveOpenLoop(test);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.m_frontLeft.driveMotorMoveOpenLoop(0);
    m_swerveDrive.m_frontRight.driveMotorMoveOpenLoop(0);
    m_swerveDrive.m_backLeft.driveMotorMoveOpenLoop(0);
    m_swerveDrive.m_backRight.driveMotorMoveOpenLoop(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
