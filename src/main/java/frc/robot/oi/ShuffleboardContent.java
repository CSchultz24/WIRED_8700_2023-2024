package frc.robot.oi;

import java.util.Map;

import com.ctre.phoenix.sensors.CANCoderFaults;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModuleSM;
import frc.robot.CTRECanCoder;

public class ShuffleboardContent {
    
    private int m_moduleNumber;

        static ShuffleboardLayout boolsLayout;

        public ShuffleboardContent() {

        }

        public static void initBooleanShuffleboard(SwerveModuleSM m_sm) {

                int m_moduleNumber = m_sm.m_locationIndex;
                String abrev = m_sm.modAbrev[m_moduleNumber];

                ShuffleboardTab x = Shuffleboard.getTab("Drivetrain");

                x.addBoolean("DriveCAN" + abrev, () -> m_sm.driveMotorConnected)
                                .withPosition(8, m_moduleNumber);
                x.addBoolean("TurnCAN" + abrev, () -> m_sm.turnMotorConnected)
                                .withPosition(9, m_moduleNumber);

        }

        public static void initDriveShuffleboard(SwerveModuleSM m_sm) {

                int m_moduleNumber = m_sm.m_locationIndex;
                String abrev = m_sm.modAbrev[m_moduleNumber];
                String driveLayout = m_sm.getModuleName(m_moduleNumber) + " Drive";
                ShuffleboardLayout drLayout = Shuffleboard.getTab("Drivetrain")
                                .getLayout(driveLayout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 0)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                drLayout.addNumber("Drive Speed MPS " + abrev, () -> m_sm.getDriveVelocity());

                drLayout.addNumber("Drive Position " + abrev, () -> m_sm.getDrivePosition());

                drLayout.addNumber("App Output " + abrev,
                                () -> m_sm.m_driveMotor.getAppliedOutput());

                drLayout.addNumber("Current Amps " + abrev,
                                () -> m_sm.getDriveCurrent());

                drLayout.addNumber("Firmware" + abrev,
                                () -> m_sm.m_driveMotor.getFirmwareVersion());

        }

        public static void initTurnShuffleboard(SwerveModuleSM m_sm) {
                int m_moduleNumber = m_sm.m_locationIndex;
                String abrev = m_sm.modAbrev[m_moduleNumber];
                String turnLayout = m_sm.getModuleName(m_moduleNumber) + " Turn";

                ShuffleboardLayout tuLayout = Shuffleboard.getTab("Drivetrain")
                                .getLayout(turnLayout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 2)
                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                tuLayout.addNumber("Turn Setpoint Deg " + abrev, () -> m_sm.angle);

                tuLayout.addNumber("Turn Enc Pos " + abrev,
                                () -> m_sm.getTurnAngleDegs() % 360);

                tuLayout.addNumber("Act Ang Deg " + abrev,
                                () -> m_sm.getTurnAngleDegs());

                tuLayout.addNumber("TurnAngleOut" + abrev,
                                () -> m_sm.m_turnMotor.getAppliedOutput());

                tuLayout.addNumber("Position" + abrev, () -> m_sm.m_turnCANcoder.getPosition());

                tuLayout.addNumber("Current Amps" + abrev, () -> m_sm.getTurnCurrent());

                tuLayout.addNumber("Abs Offset" + abrev, () -> m_sm.m_turnEncoderOffset);

                tuLayout.addNumber("Firmware" + abrev,
                                () -> m_sm.m_turnMotor.getFirmwareVersion());

        }

        public static void initCoderBooleanShuffleboard(SwerveModuleSM m_sm) {

                int m_moduleNumber = m_sm.m_locationIndex;
                String abrev = m_sm.modAbrev[m_moduleNumber];

                ShuffleboardTab x = Shuffleboard.getTab("CanCoders");
                String error;
                x.addBoolean("CANOK" + abrev, () -> m_sm.turnCoderConnected  )               
                      .withPosition(8, m_moduleNumber);
                // x.addBoolean("Fault" + abrev, () -> m_sm.m_turnCANcoder.getFaults()
                // .withPosition(9, m_moduleNumber));

        }

        public static void initCANCoderShuffleboard(SwerveModuleSM m_sm) {
                int m_moduleNumber = m_sm.m_locationIndex;
                ;
                String abrev = m_sm.modAbrev[m_moduleNumber];
                String canCoderLayout = m_sm.getModuleName(m_moduleNumber) + " CanCoder";

                ShuffleboardLayout coderLayout = Shuffleboard.getTab("CanCoders")
                                .getLayout(canCoderLayout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 0)
                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                coderLayout.addNumber("Position" + abrev,
                                () -> m_sm.m_turnCANcoder.getPosition());
                coderLayout.addNumber("Abs Position" + abrev,
                                () -> m_sm.m_turnCANcoder.getAbsolutePosition());
                coderLayout.addNumber("Velocity" + abrev,
                                () -> m_sm.m_turnCANcoder.getVelocity());
                coderLayout.addString(" MagField " + abrev,
                                () -> m_sm.m_turnCANcoder.getMagnetFieldStrength().toString());

                coderLayout.addNumber("Bus Volts" + abrev,
                                () -> m_sm.m_turnCANcoder.getBusVoltage());

                coderLayout.addNumber("Abs Offset" + abrev, () -> m_sm.m_turnEncoderOffset);

                coderLayout.addNumber("Firmware#" + abrev,
                                () -> m_sm.m_turnCANcoder.getFirmwareVersion());

        }

        public static void initMisc(DriveSubsystem drive) {

                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Drivetrain");

                drLayout1.addBoolean("FieldOr", () -> drive.m_fieldOriented).withPosition(8, 4)

                                .withSize(1, 1);
                drLayout1.addNumber("GyroYaw", () -> drive.getHeadingDegrees()).withPosition(9, 4)

                                .withSize(1, 1);
        }
}
