/*
 *  Author : Alex Naehu
 * 
 * Functionality : controls the arm using 
 * 
 *  Methods :  controls the arm extension by the power, controls the arm pivot by the power,
 *             gets the status of each limit switch, gets the angle of the arm pivot,  
 *             moves the arm extension to the target distance, moves the arm pivot to the targetAngle
 * 
 *  Revision History : First created 1/13/23
 * 
 * 
 */
package frc.robot.Mechanisms;

//import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.XboxController;

//import javax.lang.model.util.ElementScanner14;

//import com.ctre.phoenix.motorcontrol.can.BaseMotorController; //used to make VictorSPX motors follow eachother
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.IMotorController;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import frc.robot.Robot.Constants;
import edu.wpi.first.wpilibj.AnalogInput;



public class Arm{

private static WPI_VictorSPX leftAngler = new WPI_VictorSPX(28); //made both of these static
//private static WPI_VictorSPX rightAngler = new WPI_VictorSPX(24); //check this motor's CAN ID





private Timer pivotTimer = new Timer();


private static double PIVOT_VOLTAGE_OFFSET = 0.0;//may change if the motors require higher voltage.
    // may need an offset if the motor voltage requirement is higher than the max limit of the analog input
    // in the case that an offset is used, when calculating the angle, add back the offset to value



public AnalogInput armPivotEnc;
private static final int    ARM_PIVOT_ENCODER_ANALOG_PORT = 0;
private static final double ARM_PIVOT_ENC_MAX_VOLTAGE     = 4.784;


public boolean armTargetHit = false;

//private static final double INPUT_THRESHOLD = 1.0E-3;

public Arm(){

    leftAngler = new WPI_VictorSPX(28);
    armPivotEnc = new AnalogInput(ARM_PIVOT_ENCODER_ANALOG_PORT);
    pivotTimer = new Timer();

    }

    public void armUp(){
        leftAngler.set(0.3);
        pivotTimer.start();
        if (pivotTimer.advanceIfElapsed(0.5)){ //must calibrate this without a stopping sensor
            leftAngler.set(0);
        }
        pivotTimer.stop();
        pivotTimer.reset();
    }

    public void armDown(){
        leftAngler.set(-0.1); //gravity will make it fall faster down than resisting gravity upward (requires less power down)
        pivotTimer.start();
        if (pivotTimer.advanceIfElapsed(0.5)){ //must calibrate this without a stopping sensor
            leftAngler.set(0);
        }
        pivotTimer.stop();
        pivotTimer.reset();
    }

    //KEEPING ANGLER GETTER AND ANGLE SENSOR FOR SAFETY IF THE PHYSICAL BREAK BREAKS
    public double getPivotAngle() 
    {   
        return (((armPivotEnc.getVoltage() + PIVOT_VOLTAGE_OFFSET) / ARM_PIVOT_ENC_MAX_VOLTAGE) * 360.0);
    }
  
    public void setArmTargetHit(boolean state)
    {
        armTargetHit = state;
    }

    public boolean getArmTargetHit()
    {
        return armTargetHit;
    }

}