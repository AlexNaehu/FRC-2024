package frc.robot.Robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); //guess
    public static final double kDriveMotorGearRatio = 1 / 5.8462; //guess
    public static final double kTurningMotorGearRatio = 1 / 18.0; //guess
    public static final double kDriveEncoderRot2Meter = kTurningMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2MeterPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5; //guess
    
    public static final double ABSOLUTE_ENC_MAX_VOLTAGE     = 5.0; //4.784, (Technicall the pins on RoboRio are rated at 5V)

    public static final double INVALID_ANGLE = 245.0;
    public static final double CONTROLLER_INPUT_WAIT_TIME = 0.005;
    public static final int NAVX_RESET_WAIT_TIME = 1;

    //DriveConstants
    public static final double kTrackWidth = Units.inchesToMeters(21);
        // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(25.5);
        // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //FL SwerveModule Position (relative to physical center of robot)
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //FR SwerveModule Position (relative to physical center of robot)
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //BL SwerveModule Position (relative to physical center of robot)
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); //BR SwerveModule Position (relative to physical center of robot)
        

    //Front Left Swerve Module
    public static int kFLDriveMotorPort = 0; //DIO Port
    public static int kFLTurningMotorPort = 1; //DIO Port
    public static boolean kFLDriveEncoderReversed;
    public static boolean kFLTurningEncoderReversed;
    public static int kFLAbsoluteEncoderPort = 0; //analog input port on RoboRio
    public static double kFLAbsoluteEncoderOffsetRad = 0.0;
    public static boolean kFLAbsoluteEncoderReversed;

    //Front Right Swerve Module
    public static int kFRDriveMotorPort = 2;
    public static int kFRTurningMotorPort = 3;
    public static boolean kFRDriveEncoderReversed;
    public static boolean kFRTurningEncoderReversed;
    public static int kFRAbsoluteEncoderPort = 1;
    public static double kFRAbsoluteEncoderOffsetRad = 0.0;
    public static boolean kFRAbsoluteEncoderReversed;

    //Back Left Swerve Module
    public static int kBLDriveMotorPort = 4;
    public static int kBLTurningMotorPort = 5;
    public static boolean kBLDriveEncoderReversed;
    public static boolean kBLTurningEncoderReversed;
    public static int kBLAbsoluteEncoderPort = 2;
    public static double kBLAbsoluteEncoderOffsetRad = 0.0;
    public static boolean kBLAbsoluteEncoderReversed;

    //Back Right Swerve Module
    public static int kBRDriveMotorPort = 6;
    public static int kBRTurningMotorPort = 7;
    public static boolean kBRDriveEncoderReversed;
    public static boolean kBRTurningEncoderReversed;
    public static int kBRAbsoluteEncoderPort = 3;
    public static double kBRAbsoluteEncoderOffsetRad = 0.0;
    public static boolean kBRAbsoluteEncoderReversed;

    //SwerveDrive Deadband
    public static final double kDeadband = 0.05;

    //Velocity Limiters
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5.0;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

    //Acceleration Slew Limiters (SwerveDrive)
    public static final double MaxAccelerationUnitsPerSecond = 3;
    public static final double MaxAngularAccelerationUnitsPerSecond = 3;
}
