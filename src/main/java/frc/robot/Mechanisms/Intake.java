package frc.robot.Mechanisms;


//import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.XboxController;

//import com.ctre.phoenix.motorcontrol.can.BaseMotorController; //used to make VictorSPX motors follow eachother
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.IMotorController;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Robot.BananaConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot.Constants;



public class Intake {
    
    private static CANSparkMax flywheel;
    private static CANSparkMax feeder;
    private static CANSparkMax intake;
    //private static WPI_VictorSPX claw;
    private final RelativeEncoder flywheelEncoder;
    

    private int flywheelRoller_ID = 25;
    private int feederRoller_ID = 29;
    private int intake_ID = 28; //Don't know this (or any of these, double check)
    //private int claw_ID = 25;

    public boolean intakeOpen = true;

    private static volatile double targetSpeed;

    public Intake()
    {

        flywheel = new CANSparkMax(flywheelRoller_ID, MotorType.kBrushless);
        feeder = new CANSparkMax(feederRoller_ID, MotorType.kBrushless);
        intake = new CANSparkMax(intake_ID, MotorType.kBrushless);
        flywheelEncoder = flywheel.getEncoder();
        
        
    }

    public void changeClawState()
    {
        intakeOpen = !intakeOpen;
    }

    /*public void intake(double rpm) //takes 2 seconds to close around a cone from full open 
    {
        flywheel.set(-rpm);//0.55
        //rightRoller.set(power*0.8);//0.55
        //claw.set(0.5);
    }
    */

    public void output(double rpm) // takes 2 seconds to open from a cone closed position back to full open
    {   
        flywheel.set(rpm);
        //rightRoller.set(-power*0.6);
        //claw.set(-0.5);
    }

    public void setIntake(double power){
        intake.set(power);
    }

    public void setFeeder(double power){
        feeder.set(power);
    }

    public boolean isIntakeOpen()
    {
        return intakeOpen;
    }

    public double getLeftRollerPower()
    {
        return flywheel.get();
        //return claw.get();
    }

    public double getRightRollerPower()
    {
        return feeder.get();
    }
    
    public double getSpeed() 
    {   
        return flywheelEncoder.getVelocity();
    }

    public double getTargetSpeed()
    {
        return targetSpeed;
    }

    public void setTargetSpeed(double speed) //sets target rpm (3000 rpm for high score)
    {
        targetSpeed = speed;
    }

    public void flywheelPID()
    {
        Thread t = new Thread(() ->
        {
            final double ARM_PIVOT_THREAD_WAITING_TIME = 0.005;
            final double kP = 0.0;//0.018 //0.020
            final double kD = 0.0;//0.0012
            final double kI = 0.0;
            //final double kA = 0.0;//0.33//0.0077;
            //final double kF = 0.0;//-0.05;

            double power;            
            double kPpower;
            double kIpower;
            double kDpower;
            double kApower;
            double kFpower;

            Timer armTimer = new Timer();
            armTimer.start();

            double previousError = 0;
            double currentError; 
            double deltaError = 0; 

            double previousDerivative = 0;
            double currentDerivative;    
            //double filteredDerivative;  // filtered to prevent derivative jumps, and provide 
                                        //a smoother transition to a new slope of the dE v. dt graph
            
            double previousTime = 0;
            
            double deltaTime;
            
            double currentTime;
            double currentSpeed;

            double integral = 0;

            while(true)
            {
                if(targetSpeed >= Constants.INVALID_Speed)
                {
                    Timer.delay(Constants.CONTROLLER_INPUT_WAIT_TIME);
                }
                else
                {
                    //SmartDashboard.putBoolean("pivot pid state", runPivotPID);
                    currentTime  = armTimer.get();
                    currentSpeed = getSpeed();

                    currentError = targetSpeed - currentSpeed;
                    
                    /*if(Math.abs(currentError-previousError) < 1.0)
                    {
                       if((armTimer.get() - currentTime) > 0.1)
                       {
                            runPivotPID = false;
                            turnPivot(0.0);
                            setPivotTargetSpeed(Constants.INVALID_Speed);
                            Thread.currentThread().interrupt();
                       }    
                          
                    }*/

                    //if(runPivotPID == true)
                    //{
                        
                        deltaError = currentError - previousError;
                        deltaTime  = currentTime  - previousTime;

                        integral += deltaTime * currentError;

                        currentDerivative = (deltaError / deltaTime);
                        //filteredDerivative = (0.7 * currentDerivative) + (0.3 * previousDerivative);


                        kPpower = kP * currentError;
                        kIpower = kI * integral;
                        kDpower = kD * currentDerivative;//filteredDerivative;//currentDerivative;//filteredDerivative
                        kApower = 0;//159.5
                        kFpower = 0;

                        power = kPpower + kIpower + kDpower + kApower + kFpower;
                        //power = (kP * currentError) + (kA * (Math.cos(Math.toRadians(currentSpeed - 324.5))));//ka compensates for Speed of arm
                                //arm extension distance + 13 is the distance from pivot to wrist
                                //(kD * currentDerivative) + kF; //+ (kI * integral)

                        
                        output(-(power/8300)); //flywheel motor has max 8300 rpm, thus assigning power from 0->1 must be proportional to the ratio of target RPM to max RPM
                       
                    
                        previousError = currentError;
                        previousTime = currentTime;

                        previousDerivative = currentDerivative;

                        SmartDashboard.putNumber("Flywheel P RPM", kPpower/8300);//flywheel motor has max 8300 rpm, thus assigning power from 0->1 must be proportional to the ratio of target RPM to max RPM
                        SmartDashboard.putNumber("Flywheel I RPM", kIpower/8300);//flywheel motor has max 8300 rpm, thus assigning power from 0->1 must be proportional to the ratio of target RPM to max RPM
                        SmartDashboard.putNumber("Flywheel D RPM", kDpower/8300);//flywheel motor has max 8300 rpm, thus assigning power from 0->1 must be proportional to the ratio of target RPM to max RPM
                        SmartDashboard.putNumber("Total Flywheel RPM", power/8300);//flywheel motor has max 8300 rpm, thus assigning power from 0->1 must be proportional to the ratio of target RPM to max RPM
                    
                        Timer.delay(ARM_PIVOT_THREAD_WAITING_TIME);
                    //}
                }
            }
        });
        t.start();
    }


}

