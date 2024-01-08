package frc.robot.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Hook {
    private static CANSparkMax hook;
    private final int hook_ID = 3;

    public Hook(){
        hook = new CANSparkMax(hook_ID, MotorType.kBrushless);
    }

    public void hookDown(){
        hook.set(-0.5); //need super high gear ratio
    }

    public void hookStop(){
        hook.set(0);
    }


}
