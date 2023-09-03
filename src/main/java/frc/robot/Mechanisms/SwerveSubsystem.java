package frc.robot.Mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot.Constants;
import frc.robot.Robot.Robot;

public class SwerveSubsystem {
    
    private final SwerveModule frontLeft = new SwerveModule(
    Constants.kFLDriveMotorPort, 
    Constants.kFLTurningMotorPort, 
    Constants.kFLDriveEncoderReversed, 
    Constants.kFLTurningEncoderReversed,
    Constants.kFLAbsoluteEncoderPort, 
    Constants.kFLAbsoluteEncoderOffsetRad, 
    Constants.kFLAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
    Constants.kFRDriveMotorPort, 
    Constants.kFRTurningMotorPort, 
    Constants.kFRDriveEncoderReversed, 
    Constants.kFRTurningEncoderReversed,
    Constants.kFRAbsoluteEncoderPort, 
    Constants.kFRAbsoluteEncoderOffsetRad, 
    Constants.kFRAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
    Constants.kBLDriveMotorPort, 
    Constants.kBLTurningMotorPort, 
    Constants.kBLDriveEncoderReversed, 
    Constants.kBLTurningEncoderReversed,
    Constants.kBLAbsoluteEncoderPort, 
    Constants.kBLAbsoluteEncoderOffsetRad, 
    Constants.kBLAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
    Constants.kBRDriveMotorPort, 
    Constants.kBRTurningMotorPort, 
    Constants.kBRDriveEncoderReversed, 
    Constants.kBRTurningEncoderReversed,
    Constants.kBRAbsoluteEncoderPort, 
    Constants.kBRAbsoluteEncoderOffsetRad, 
    Constants.kBRAbsoluteEncoderReversed);

    public static double getHeading(){
        return Math.IEEEremainder(Robot.navx.getAngle(), 360);
    }

    public Rotation2d geRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    
}
