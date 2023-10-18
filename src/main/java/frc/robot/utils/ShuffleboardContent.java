// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModuleSM;

/** Add your docs here. */
public class ShuffleboardContent {

        private static int locationIndex;

        // static ShuffleboardLayout boolsLayout;

        public ShuffleboardContent() {

        }

        public static void initBooleanShuffleboard(SwerveModuleSM m_sm) {

                locationIndex = m_sm.m_locationIndex;

                String abrev = m_sm.modAbrev[locationIndex];

                ShuffleboardTab x = Shuffleboard.getTab("Drivetrain");

                x.addBoolean("DriveCAN" + abrev, () -> m_sm.driveMotorConnected)
                                .withPosition(8, locationIndex);
                x.addBoolean("TurnCAN" + abrev, () -> m_sm.turnMotorConnected)
                                .withPosition(9, locationIndex);

        }

        public static void initDriveShuffleboard(SwerveModuleSM m_sm) {

                int locationIndex = m_sm.m_locationIndex;
                String abrev = m_sm.modAbrev[locationIndex];
                String driveLayout = abrev + " Drive";
                ShuffleboardLayout drLayout = Shuffleboard.getTab("Drivetrain")
                                .getLayout(driveLayout, BuiltInLayouts.kList).withPosition(locationIndex * 2, 0)
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

                int locationIndex = m_sm.m_locationIndex;
                String abrev = m_sm.modAbrev[locationIndex];
                String turnLayout = abrev + " Turn";

                ShuffleboardLayout tuLayout = Shuffleboard.getTab("Drivetrain")
                                .getLayout(turnLayout, BuiltInLayouts.kList).withPosition(locationIndex * 2, 2)
                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                tuLayout.addNumber("Turn Setpoint Deg " + abrev, () -> m_sm.angle);

                tuLayout.addNumber("Turn Enc Pos " + abrev,
                                () -> m_sm.getTurnAngleDegs());

                tuLayout.addNumber("TurnOutput" + abrev,
                                () -> m_sm.m_turnMotor.getAppliedOutput());

                tuLayout.addNumber("CancoderPosition" + abrev, () -> m_sm.m_turnCANcoder.getMyPosition());

                tuLayout.addNumber("Current Amps" + abrev, () -> m_sm.getTurnCurrent());

                tuLayout.addNumber("Abs Offset" + abrev, () -> m_sm.m_turnEncoderOffset);

                tuLayout.addNumber("Firmware" + abrev,
                                () -> m_sm.m_turnMotor.getFirmwareVersion());

        }

        public static void initCoderBooleanShuffleboard(SwerveModuleSM m_sm) {

                int locationIndex = m_sm.m_locationIndex;
                String abrev = m_sm.modAbrev[locationIndex];
                ShuffleboardTab x = Shuffleboard.getTab("CanCoders");

                x.addBoolean("CANOK" + abrev, () -> m_sm.turnCoderConnected)
                                .withPosition(8, locationIndex);
                x.addBoolean("Fault" + abrev, () -> m_sm.m_turnCANcoder.getFaulted())
                                .withPosition(9, locationIndex);

        }

        public static void initCANCoderShuffleboard(SwerveModuleSM m_sm) {
                int locationIndex = m_sm.m_locationIndex;
                String abrev = m_sm.modAbrev[locationIndex];
                String canCoderLayout = abrev + " CanCoder";

                ShuffleboardLayout coderLayout = Shuffleboard.getTab("CanCoders")
                                .getLayout(canCoderLayout, BuiltInLayouts.kList).withPosition(locationIndex * 2, 0)
                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                coderLayout.addNumber("Position" + abrev,
                                () -> m_sm.m_turnCANcoder.getMyPosition());
                coderLayout.addNumber("Abs Position" + abrev,
                                () -> m_sm.m_turnCANcoder.getAbsolutePosition());
                coderLayout.addNumber("Velocity" + abrev,
                                () -> m_sm.m_turnCANcoder.getVelValue());
                coderLayout.addString(" MagField " + abrev,
                                () -> m_sm.m_turnCANcoder.getMagnetFieldStrength().toString());
                coderLayout.addNumber("Battery Volts" + abrev,
                                () -> m_sm.m_turnCANcoder.getBatValue());
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

                drLayout1.addBoolean("OpenLoop", () -> drive.isOpenLoop).withPosition(8, 4)

                                .withSize(1, 1);
                drLayout1.addNumber("GyroYaw", () -> drive.getHeadingDegrees()).withPosition(9, 4)

                                .withSize(1, 1);
        }
}