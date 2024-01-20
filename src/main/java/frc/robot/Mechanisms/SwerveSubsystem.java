package frc.robot.Mechanisms;


import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
//import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot.Constants;
//import frc.robot.Robot.Robot;
import frc.robot.Robot.Robot;

public class SwerveSubsystem {
    
    public final static SwerveModule frontLeft = new SwerveModule(
    Constants.kFLDriveMotorPort, 
    Constants.kFLTurningMotorPort, 
    Constants.kFLDriveEncoderReversed, 
    Constants.kFLTurningEncoderReversed,
    Constants.kFLAbsoluteEncoderPort, 
    Constants.kFLAbsoluteEncoderOffsetRad, 
    Constants.kFLAbsoluteEncoderReversed);

    public final static SwerveModule frontRight = new SwerveModule(
    Constants.kFRDriveMotorPort, 
    Constants.kFRTurningMotorPort, 
    Constants.kFRDriveEncoderReversed, 
    Constants.kFRTurningEncoderReversed,
    Constants.kFRAbsoluteEncoderPort, 
    Constants.kFRAbsoluteEncoderOffsetRad, 
    Constants.kFRAbsoluteEncoderReversed);

    public final static SwerveModule backLeft = new SwerveModule(
    Constants.kBLDriveMotorPort, 
    Constants.kBLTurningMotorPort, 
    Constants.kBLDriveEncoderReversed, 
    Constants.kBLTurningEncoderReversed,
    Constants.kBLAbsoluteEncoderPort, 
    Constants.kBLAbsoluteEncoderOffsetRad, 
    Constants.kBLAbsoluteEncoderReversed);

    public final static SwerveModule backRight = new SwerveModule(
    Constants.kBRDriveMotorPort, 
    Constants.kBRTurningMotorPort, 
    Constants.kBRDriveEncoderReversed, 
    Constants.kBRTurningEncoderReversed,
    Constants.kBRAbsoluteEncoderPort, 
    Constants.kBRAbsoluteEncoderOffsetRad, 
    Constants.kBRAbsoluteEncoderReversed);

    
    
    public SwerveSubsystem(){
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
                } 
            catch (Exception e){}
        }).start();

    }

    public void zeroHeading(){
        Robot.navx.reset();
    }

    public static double getHeading(){
        return Math.IEEEremainder(Robot.navx.getAngle(), 360);
    }
    
    public static Rotation2d getRotation2d(){
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
