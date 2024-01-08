package frc.robot.Robot;


//Utilizes all mechanisms for the robot and runs their programs to perform each function


//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.hal.ThreadsJNI;
//import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.net.PortForwarder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;


//import frc.robot.Autonomous.DriveStraight;
//import frc.robot.Autonomous.Turn;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autonomous.AutonPaths;
import frc.robot.Mechanisms.Arm;
import frc.robot.Mechanisms.Hook;
import frc.robot.Mechanisms.Intake;
import frc.robot.Mechanisms.SwerveSubsystem;
//import frc.robot.Mechanisms.DriveTrain;
import frc.robot.Vision.SensorObject;




/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  public static Arm         arm;
  //public static DriveTrain  driveTrain;
  public static Intake        Intake;
  public static SensorObject    sensor;
  public static Hook hook;

  private UsbCamera Cam;

  //private static final double LEFT_DEADBAND_THRESHOLD = 0.15;
  //private static final double RIGHT_DEADBAND_THRESHOLD = 0.15;
  double pThr = 0.0;

  //private boolean armPIDState = false;

  private SwerveSubsystem swerveSubsystem;

  public static AHRS navx;
  
  public static XboxController controller1;
  public static XboxController controller2;

  
  private static final int XBOX_PORT_1 = 0;
  private static final int XBOX_PORT_2 = 1;

  private static final String LeftScoreMob = "LeftScoreMob";
  private static final String MidScorePark = "MidScorePark";
  private static final String RightScoreMob = "RightScoreMob";
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static Timer timer;
  public static Timer autonClock;

  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter rotLimiter;

  private SwerveDriveOdometry odometer;
  private SwerveModulePosition[] positions;

  private boolean aimbotEnabled;
  private double kAimP = -0.000005f;  //may need to calibrate kAimP or min_command if aiming causes occilation
  private double min_command = 0.000005f;
  private double heading_error;
  private double steering_adjust;

  //private DifferentialDriveOdometry odometry;

  @Override
  public void robotInit() 
  {

    /*--------------------------------------------------------------------------
    *  Initialize Drive Cameras
    *-------------------------------------------------------------------------*/

    Cam = CameraServer.startAutomaticCapture();
    Cam.setFPS(15);
    Cam.setResolution(320, 240);
    Cam.setPixelFormat(PixelFormat.kMJPEG);

    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);
   
   
    /*--------------------------------------------------------------------------
    *  Initialize Mechanisms & Drive Controllers
    *-------------------------------------------------------------------------*/
    arm        = new Arm();
    //driveTrain = new DriveTrain();
    Intake = new Intake();
  
    sensor = new SensorObject();

    hook = new Hook();

    navx = new AHRS();

    controller1    = new XboxController(XBOX_PORT_1);
    controller2    = new XboxController(XBOX_PORT_2);

    swerveSubsystem = new SwerveSubsystem();

    positions = new SwerveModulePosition[] {
      SwerveSubsystem.frontLeft.getPosition(), 
      SwerveSubsystem.frontRight.getPosition(), 
      SwerveSubsystem.backLeft.getPosition(), 
      SwerveSubsystem.backRight.getPosition()
    }; //UNSURE}

    odometer = new SwerveDriveOdometry(Constants.kDriveKinematics, new Rotation2d(0), positions);
    
    xLimiter = new SlewRateLimiter(Constants.MaxAccelerationUnitsPerSecond);
    yLimiter = new SlewRateLimiter(Constants.MaxAccelerationUnitsPerSecond);
    rotLimiter = new SlewRateLimiter(Constants.MaxAngularAccelerationUnitsPerSecond);

    /*--------------------------------------------------------------------------
    *  Initialize Auton
    *-------------------------------------------------------------------------*/
    m_chooser.setDefaultOption("RightScoreMob", RightScoreMob);
    m_chooser.addOption("MidScorePark", MidScorePark);
    m_chooser.addOption("LeftScoreMob", LeftScoreMob);
   
    SmartDashboard.putData("Auto choices", m_chooser);

    /*--------------------------------------------------------------------------
    *  Engage Mechanical Brakes, Set Target Angles to Current Angles & Start
    *  PID Threads
    *-------------------------------------------------------------------------*/
    
    navx.reset();
    
    //arm.pivotPID();

    Intake.flywheelPID();
    
                                                                                                                                                                                                                                                                                                                                                                                       
    //driveTrain.coneAimPID();
    //driveTrain.cubeAimPID(); //prolly not going to use this bc of USB load on roboRio CPU
    //sensor.sensorObject(); //def cant use bc of image processing load on roboRio load


    timer = new Timer();
    timer.start();

    autonClock = new Timer(); //starts in autonInit()
    
  }


  @Override
  public void robotPeriodic() 
  {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    //NetworkTableInstance table = NetworkTableInstance.getDefault();

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double v = tv.getDouble(0.0);
    
    //Limelight
    SmartDashboard.putNumber("Limelight X", x); //(x,y) from the crosshair on the camera stream in pixle units
    SmartDashboard.putNumber("Limelight Y", y);
    SmartDashboard.putNumber("Limelight Area", area); //area of FOV that the target takes up
    SmartDashboard.putNumber("Limelight Valid Target", v);//0 for no valid target, 1 for valid target

    //USB Camera
    Cam.setFPS(15);
    Cam.setResolution(320, 240);
    
    //Sensors
    SmartDashboard.putNumber("Roll", navx.getRoll());
   
    
    
    //DriveBase
    /*
    SmartDashboard.putBoolean("Cone Aim PID State", DriveTrain.coneAimPIDState);
    SmartDashboard.putBoolean("Cube Aim PID State", DriveTrain.cubeAimPIDState);
    SmartDashboard.putNumber("FR Motor Temperature", driveTrain.getMotorTemperature(21));
    SmartDashboard.putNumber("BR Motor Temperature", driveTrain.getMotorTemperature(22));
    SmartDashboard.putNumber("FL Motor Temperature", driveTrain.getMotorTemperature(20));
    SmartDashboard.putNumber("BL Motor Temperature", driveTrain.getMotorTemperature(23));
    SmartDashboard.putNumber("Cone Left Command", driveTrain.cone_left_command);
    SmartDashboard.putNumber("Cone Right Command", driveTrain.cone_right_command);
    SmartDashboard.putNumber("Cube Left Command", driveTrain.cube_left_command);
    SmartDashboard.putNumber("Cube Right Command", driveTrain.cube_right_command);
    */
    SmartDashboard.putBoolean("Aimbot Enabled", aimbotEnabled);
    SmartDashboard.putNumber("Aimbot Heading Error", heading_error);
    SmartDashboard.putNumber("Aimbot PID Power", steering_adjust);


    //Arm          

    
    SmartDashboard.putBoolean("Arm Target Hit", arm.armTargetHit);
    //SmartDashboard.putNumber("PIVOT: Target Angle", arm.getPivotTargetAngle());
    SmartDashboard.putNumber("PIVOT: Encoder Voltage", arm.armPivotEnc.getVoltage());
    SmartDashboard.putNumber("PIVOT: Encoder Angle", arm.getPivotAngle());
    

    //Intake
    SmartDashboard.putBoolean("Intake: Open State", Intake.isIntakeOpen());
    SmartDashboard.putNumber("Intake: Left Power", Intake.getLeftRollerPower());
    SmartDashboard.putNumber("Intake: Right Power", Intake.getRightRollerPower());

  
    //Odometer
    odometer.update(getRotation2d(), new SwerveModulePosition[] {
      SwerveSubsystem.frontLeft.getPosition(), SwerveSubsystem.frontRight.getPosition(),
      SwerveSubsystem.backLeft.getPosition(), SwerveSubsystem.backRight.getPosition()
    });
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

    //double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    //double limelightMountAngleDegrees = 25.0;
  





    // distance from the center of the Limelight lens to the floor
    //double limelightLensHeightInches = 35.5;//WILL CHANGE THIS BECAUSE WE ARE USING A NEW ROBOT
  
    // distance from the target to the floor
    //double goalHeightInches = 104.0;//WILL CHANGE BECAUSE THE GOALS ARE NOW DIFFERENT (MIGHT NOT EVEN NEED HEIGHT SINCE WE WORK IN INCHES)
  
    //double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;//AGAIN WE MAY NOT NEED THIS
    //double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);//DOUBLE CHECK IF WE ARE GOING TO BE DOING IT THIS WAY)
  
    //calculate distance
    //double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians); //(COULD POTENTIALLY USE THIS TO TELL
                                                                                                                            //THE ROBOT WHEN TO STOP IN FRONT OF
                                                                                                                            //THE GOAL TO BEGIN RAISING THE ARM,
                                                                                                                            //THEN TO MOVE CLOSER TO SCORE THE GOAL
  }

  @Override
  public void autonomousInit() 
  {

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    //initializeRobotPositions();

    autonClock.reset();
    autonClock.start();

  }

  @Override
  public void autonomousPeriodic() //periodically calls one of three battle plans >:D
  {
    
    //Moved to autonomousInit();
    //autonClock.reset();
    //autonClock.start();

    switch (m_autoSelected) {

      case LeftScoreMob:
      AutonPaths.leftScoreMob();
        // Put custom auto code here
        break;
      case MidScorePark:
      AutonPaths.midScorePark();
        // Put custom auto code here
        break;
      case RightScoreMob:
        AutonPaths.rightScoreMob();
        // Put default auto code here
        break;
    }
    

    
  }
    
  @Override
  public void teleopInit() 
  {
    autonClock.stop();
    //initializeRobotPositions();

  }

  @Override
  public void teleopPeriodic() 
  {
    
    robotControls();
    
  }


  private void robotControls()
  {
    
    // AIMBOT TOGGLE // 1
    if (aimbotEnabled) {
      heading_error = -((SmartDashboard.getNumber("Center X", 0.0))+320);
      steering_adjust = 0.0;

      if (Math.abs(heading_error) > 1.0) {
          steering_adjust = kAimP * heading_error + min_command;
      }


    /*--------------------------------------------------------------------------
    *  DriveBase Movement - Manual/ Aimbot Control (1)
    *-------------------------------------------------------------------------*/
    
        // 1. Get real-time joystick inputs
        double xSpeed = controller1.getLeftX();
        double ySpeed = controller1.getLeftY();
        double rotSpeed = controller1.getRightY();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > Constants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.kDeadband ? ySpeed : 0.0;
        rotSpeed = Math.abs(rotSpeed) > Constants.kDeadband ? rotSpeed : 0.0;

        // 3. Apply acceleration smoothening
        xSpeed = xLimiter.calculate(xSpeed) * Constants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.kTeleDriveMaxSpeedMetersPerSecond;
        rotSpeed = rotLimiter.calculate(rotSpeed) * Constants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getRotation2d());
        //(Field's Perspective, "North" is the field's North)

        if (aimbotEnabled){
          // Adjust only the rotational component for swerve drive
          chassisSpeeds = new ChassisSpeeds(0, 0, steering_adjust);
        }

        //5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        //6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);




    /*--------------------------------------------------------------------------
    *  DriveBase Movement - AimBot (1)
    *-------------------------------------------------------------------------*/
    
    

      
  }


    /*--------------------------------------------------------------------------
    *  Intake - Manual Control (1)
    *-------------------------------------------------------------------------*/
 
    if(controller1.getLeftBumper() && controller1.getRightTriggerAxis()<0.05) //suck in
    {
      Intake.setIntake(0.3); // LEFT BUMPER // 1
    }
    else if(controller1.getRightBumper() && controller1.getRightTriggerAxis()<0.05) //shoot out (not really shoot just drop)
    {
      Intake.setIntake(-0.3); // RIGHT BUMPER // 1
    }
    else{
      Intake.setIntake(0.0);//Stop Motors
    }

    /*--------------------------------------------------------------------------
    *  Output (Flywheel) - Manual Control (1)
    *-------------------------------------------------------------------------*/
    
    if(controller1.getLeftTriggerAxis()<0.05 && controller1.getRightTriggerAxis()>0.05)
    {
      Intake.setFeeder(0.3);
      Intake.setTargetSpeed(3000); // RIGHT TRIGGER // 1
    }
    else
    {
      Intake.setFeeder(0);
      Intake.setTargetSpeed(0.0); //Stop Motors
    }
    
    /*--------------------------------------------------------------------------
    *  Arm Movement - Manual Control (1)
    *-------------------------------------------------------------------------*/
    
    if (controller1.getYButton() && !(controller1.getAButton())){
      arm.armUp();
      arm.setArmTargetHit(true);
    }
    else if (controller1.getAButton() && !(controller1.getYButton())){
      arm.armDown();
      arm.setArmTargetHit(true);
    }
    else{
      arm.setArmTargetHit(false);
    }


    /*--------------------------------------------------------------------------
    *  Hook - Presets (2)
    *-------------------------------------------------------------------------*/
     
    

    /*-----------------------------------------------------------------------
    *  Aimbot - Manual Control (1)
    *----------------------------------------------------------------------*/
    
    if (controller1.getLeftTriggerAxis()>0.05){
      aimbotEnabled = true; // LEFT TRIGGER // 1
    }
    else{
      aimbotEnabled = false;
    }
    

  } //End of robot controlls




  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    odometer.resetPosition(getRotation2d(), positions, pose);
  }

  public static double getHeading(){
    return Math.IEEEremainder(Robot.navx.getAngle(), 360);
}

public static Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
}

  

}
