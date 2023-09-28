package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.WinchConstants.*;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WinchSubsystem {
    // One DcMotorEx

    // Method to move the motor
    // (Maybe) Method to move the motor the other way
    // Method to stop the motor

    private final DcMotorEx winchMotor;

    public WinchSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize the servo
        winchMotor = hardwareMap.get(DcMotorEx.class, WINCH_DC_MOTOR_NAME);
    }

    public void intake() {
        winchMotor.setPower(INTAKE_SPEED);
    }

    // Set the motor to outtake
    public void outtake() {
        winchMotor.setPower(OUTTAKE_SPEED);
    }

    // Stop the motor
    public void stop() { winchMotor.setPower(0); }
}
