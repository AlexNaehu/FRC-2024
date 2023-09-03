package frc.robot.Autonomous;

import java.util.List;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Mechanisms.SwerveSubsystem;
import frc.robot.Robot.Constants;

public class AutonPaths {

    public static void rightScoreMob(){
            // 1. Create trajectory settings
            TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                Constants.kTeleDriveMaxSpeedMetersPerSecond, 
                Constants.MaxAngularAccelerationUnitsPerSecond)
                .setKinematics(Constants.kDriveKinematics);

            // 2. Generate trajectory
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)), //initial point
                List.of(
                        new Translation2d(1,0), //intermediary points
                        new Translation2d(1,-1)   //intermediary points
                        ),
                        new Pose2d(2, -1, Rotation2d.fromDegrees(180)), //final point plus its final orientation
                        trajectoryConfig
                );

            // 3. Define PID controllers for tracking trajectory
            PIDController xController = new PIDController(Constants.kPXController, 0, 0);
            PIDController yController = new PIDController(Constants.kPYController, 0, 0);
            ProfiledPIDController rotController = new ProfiledPIDController(Constants.kPRotController, 0, 0, Constants.kRotControllerConstraints);

            rotController.enableContinuousInput(-Math.PI, Math.PI);

            // 4. Contruct command to follow trajectory
            
            
    }

    public static void leftScoreMob(){

    }

    public static void midScorePark(){

    }
}
