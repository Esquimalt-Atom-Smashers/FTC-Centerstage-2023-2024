package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.WinchConstants.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WinchSubsystem {
    // One DcMotorEx

    // Method to move the motor
    // (Maybe) Method to move the motor the other way
    // Method to stop the motor

    private final DcMotorEx winchMotor;

    public WinchSubsystem(HardwareMap hardwareMap) {
        // Initialize the servo
        winchMotor = hardwareMap.get(DcMotorEx.class, WINCH_MOTOR_NAME);
    }

    public void intake() {
        winchMotor.setPower(WINCH_SPEED);
    }

    // Set the motor to outtake
    public void outtake() {
        winchMotor.setPower(UNWINCH_SPEED);
    }

    // Stop the motor
    public void stop() { winchMotor.setPower(0); }
}
