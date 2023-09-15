package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ClawSubsystem {
    
    private final Servo claw1;
    private final Servo claw2;

    public ClawSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        claw1 = hardwareMap.servo.get(Constants.ArmConstants.CLAW1_SERVO_MOTOR_NAME);
        claw2 = hardwareMap.servo.get(Constants.ArmConstants.CLAW2_SERVO_MOTOR_NAME);
    }

    public void openClaw1()
    {
        claw1.setPosition(/*Hard code values*/);
    }

    public void openClaw2()
    {
        claw2.setPosition(/*Hard code values*/);
    }

    public void closeClaw1()
    {
        claw1.setPosition(/*Hard code values*/);
    }  

    public void closeClaw2()
    {
        claw2.setPosition(/*Hard code values*/);
    }  

    public Servo getClaw1()
    {
        return claw1;
    }

    public Servo getClaw2()
    {
        return claw2;
    }

}