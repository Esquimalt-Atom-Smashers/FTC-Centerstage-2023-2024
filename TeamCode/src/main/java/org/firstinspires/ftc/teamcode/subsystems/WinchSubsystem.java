package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.WinchConstants.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WinchSubsystem extends SubsystemBase {
    // One DcMotorEx

    // Method to move the motor
    // (Maybe) Method to move the motor the other way
    // Method to stop the motor

    private final DcMotorEx winchMotor;

    public WinchSubsystem(HardwareMap hardwareMap) {
        // Initialize the servo
        winchMotor = hardwareMap.get(DcMotorEx.class, WINCH_MOTOR_NAME);

        winchMotor.setDirection(WINCH_MOTOR_DIRECTION);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void winch() {
        winchMotor.setPower(WINCH_SPEED);
    }

    public void unwinch() {
        winchMotor.setPower(UNWINCH_SPEED);
    }

    // Stop the motor
    public void stop() { winchMotor.setPower(0); }
}
