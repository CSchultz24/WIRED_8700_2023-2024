package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SetSwerveDriveSlow extends CommandBase{
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_swerveDrive;
  private final SlewRateLimiter m_slewX = new SlewRateLimiter(DriverConstants.kTranslationSlew, -10000, 0);
  private final SlewRateLimiter m_slewY = new SlewRateLimiter(DriverConstants.kTranslationSlew, -10000, 0);
  private final SlewRateLimiter m_slewRot = new SlewRateLimiter(DriverConstants.kRotationSlew, -10000, 0);

  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;
  private double throttle;
  private double strafe;
  private double rotation;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public SetSwerveDriveSlow(
      DriveSubsystem swerveDriveSubsystem, DoubleSupplier throttleInput, DoubleSupplier strafeInput,
      DoubleSupplier rotationInput) {

    m_swerveDrive = swerveDriveSubsystem;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveSubsystem);
  }

  double mult = .2;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // https://www.chiefdelphi.com/t/swerve-controller-joystick/392544/5
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    throttle = MathUtil.applyDeadband(Math.abs(m_throttleInput.getAsDouble()),
        DriverConstants.kControllerDeadband)
        * Math.signum(m_throttleInput.getAsDouble());
    strafe = MathUtil.applyDeadband(Math.abs(m_strafeInput.getAsDouble()),
        DriverConstants.kControllerDeadband)
        * Math.signum(m_strafeInput.getAsDouble());
    rotation = MathUtil.applyDeadband(Math.abs(m_rotationInput.getAsDouble()),
        DriverConstants.kControllerRotDeadband)
        * Math.signum(m_rotationInput.getAsDouble());

    // square values after deadband while keeping original sign

    throttle = Math.signum(throttle) * Math.pow(throttle, 2);
    strafe = Math.signum(strafe) * Math.pow(strafe, 2);
    rotation = Math.signum(rotation) * Math.pow(rotation, 2);

    throttle *= -DriveConstants.kMaxSpeedMetersPerSecond;
    strafe *= -DriveConstants.kMaxSpeedMetersPerSecond;
    rotation *= DriveConstants.kMaxRotationRadiansPerSecond;

    throttle *= mult;
    strafe *= mult;
    rotation *= mult;

    if (Math.abs(rotation) < DriverConstants.kControllerRotDeadband)
      rotation = 0;

    double throttle_sl = m_slewX.calculate(throttle);
    double strafe_sl = m_slewY.calculate(strafe);
    double rotation_sl = m_slewRot.calculate(rotation);
    // SlewRateLimiter rotation set with very fast rate
    m_swerveDrive.drive(throttle_sl, strafe_sl, -rotation);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
