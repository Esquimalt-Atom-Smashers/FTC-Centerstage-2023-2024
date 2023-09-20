package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.ElbowConstants.*;
public class ElbowSubsystem {
    // One DcMotorEx
    // Method to spin the motor clockwise
    // Method to spin the motor counter clockwise
    // Method to set the arm to a specific position
    // Logic to limit the motor from going too high
    // Lower level method using PIDF to accurately set the position of the motor

    // TODO: Fix all of this, was written very quickly to test the elbow

    private final DcMotorEx elbowMotor;

    public  ElbowSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        elbowMotor = hardwareMap.get(DcMotorEx.class, ELBOW_DC_MOTOR_NAME);
        elbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void spin() {
        elbowMotor.setPower(1);
    }

    public void counterSpin() {
        elbowMotor.setPower(-1);
    }

    public void stop() {
        elbowMotor.setPower(0);
    }
}
