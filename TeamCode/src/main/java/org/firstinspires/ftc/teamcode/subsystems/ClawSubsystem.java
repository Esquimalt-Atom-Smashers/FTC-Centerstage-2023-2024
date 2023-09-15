package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.*;

public class ClawSubsystem {

    private final Servo claw1;
    private final Servo claw2;

    public ClawSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        claw1 = hardwareMap.servo.get(CLAW1_SERVO_MOTOR_NAME);
        claw2 = hardwareMap.servo.get(CLAW2_SERVO_MOTOR_NAME);
    }

    public void openClaw1()
    {
        claw1.setPosition(OPEN_POSITION);
    }

    public void openClaw2()
    {
        claw2.setPosition(OPEN_POSITION);
    }

    public void closeClaw1()
    {
        claw1.setPosition(CLOSE_POSITION);
    }  

    public void closeClaw2()
    {
        claw2.setPosition(CLOSE_POSITION);
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