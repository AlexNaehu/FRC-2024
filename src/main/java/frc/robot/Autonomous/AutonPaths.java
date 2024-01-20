package frc.robot.Autonomous;

import java.util.List;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Mechanisms.SwerveSubsystem;
import frc.robot.Robot.Constants;
import frc.robot.Robot.Robot;

public class AutonPaths {

    private static TrajectoryConfig trajectoryConfig;
    private static Trajectory trajectory;
    private static boolean isFollowing = false;
    private static Pose2d currentPose;

    private static double errorX;
    private static double errorY;
    private static double errorTheta;

    private static double pidOutputX;
    private static double pidOutputY;
    private static double pidOutputTheta;

    private static SlewRateLimiter pidLimiterX;
    private static SlewRateLimiter pidLimiterY;
    private static SlewRateLimiter pidLimiterTheta;

    private static double pidSpeedX;
    private static double pidSpeedY;
    private static double pidSpeedTheta;

    private static Timer swerveTimer;
    private static double elapsedTime;
    private static double expectedDuration;

    

    public static void leftScoreMob(){

        
        if(!isFollowing){
            // Start Timer
            swerveTimer.reset();
            swerveTimer.start();

            // Initialize Slew Rate Limiters for Acceleration Smoothening
            pidLimiterX = new SlewRateLimiter(Constants.MaxAccelerationUnitsPerSecond);
            pidLimiterY = new SlewRateLimiter(Constants.MaxAccelerationUnitsPerSecond);
            pidLimiterTheta = new SlewRateLimiter(Constants.MaxAngularAccelerationUnitsPerSecond);
            
            // Initialize trajectory settings
            trajectoryConfig = new TrajectoryConfig(
            Constants.kPhysicalMaxSpeedMetersPerSecond,
            Constants.MaxAccelerationUnitsPerSecond)
                    .setKinematics(Constants.kDriveKinematics);

            // Generate trajectory
            trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                    new Translation2d(1, 0),
                    new Translation2d(1, -1)),
                    new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            trajectoryConfig);

            expectedDuration = trajectory.getTotalTimeSeconds();

            isFollowing = true;
        }

        while (isFollowing){

        currentPose = Robot.getPose();

        // Calculate errors in x, y, and theta... calcualtes a distance in whole meters
        errorX = trajectory.getStates().get(0).poseMeters.getX() - currentPose.getX();
        errorY = trajectory.getStates().get(0).poseMeters.getY() - currentPose.getY();
        errorTheta = trajectory.getStates().get(0).poseMeters.getRotation().getRadians() - currentPose.getRotation().getRadians();

        // Calculate PID outputs for X
        pidOutputX = Constants.kPXController * errorX;
        pidSpeedX = pidLimiterX.calculate(pidOutputX);

        // Calculate PID outputs for Y
        pidOutputY = Constants.kPYController * errorY;
        pidSpeedY = pidLimiterY.calculate(pidOutputY);

        // Calculate PID outputs for Theta
        pidOutputTheta = Constants.kPTurning * errorTheta;
        pidSpeedTheta = pidLimiterTheta.calculate(pidOutputTheta);

        // Use the PID outputs to update motor outputs and robot's pose

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(pidSpeedX, pidSpeedY, pidSpeedTheta, SwerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        Robot.swerveSubsystem.setModuleStates(moduleStates); //desaturates the speeds for me and assigns the states to each module

        elapsedTime = swerveTimer.get();

        if (elapsedTime >= expectedDuration || elapsedTime >= 14.95) {
            isFollowing = false;
            swerveTimer.stop();
            swerveTimer.reset();
            break;
        }

        }
            
    }

    public static void rightScoreMob(){

    }

    public static void midScorePark(){

    }

    public static double getExpectedDuration(){
        return expectedDuration;
    }
}
