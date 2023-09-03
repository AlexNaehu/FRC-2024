package frc.robot.Mechanisms;




import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot.Constants;

public class SwerveModule {
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final CANEncoder driveEncoder;
    private final CANEncoder turningEncoder;

    private final PIDController turningPIDController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(
        int driveMotorID, 
        int turningMotorID, 
        boolean driveMotorReversed, 
        boolean turningMotorReversed, 
        int absoluteEncoderID, 
        double absoluteEncoderOffset, 
        boolean absoluteEncoderReversed){

            this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
            this.absoluteEncoderReversed = absoluteEncoderReversed;
            absoluteEncoder = new AnalogInput(absoluteEncoderID);

            driveMotor = new CANSparkMax(absoluteEncoderID, MotorType.kBrushless);
            turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

            driveMotor.setInverted(driveMotorReversed);
            turningMotor.setInverted(turningMotorReversed);

            driveEncoder = driveMotor.getEncoder();
            turningEncoder = turningMotor.getEncoder();
            
            driveEncoder.setPositionConversionFactor(Constants.kDriveEncoderRot2Meter);
            driveEncoder.setVelocityConversionFactor(Constants.kDriveEncoderRPM2MeterPerSec);
            turningEncoder.setPositionConversionFactor(Constants.kTurningEncoderRot2Rad);
            turningEncoder.setVelocityConversionFactor(Constants.kTurningEncoderRPM2MeterPerSec);

            turningPIDController = new PIDController(Constants.kPTurning, 0, 0);
            turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

            resetEncoders();
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

    public double getTurningPosition(){
        return turningEncoder.getPosition();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
      }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getVoltage() / Constants.ABSOLUTE_ENC_MAX_VOLTAGE;
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0); //calculates 
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond) < 0.001){ //makes sure that when letting go of joysticks, the wheels stay oriented how they were but stop moving
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }
}

