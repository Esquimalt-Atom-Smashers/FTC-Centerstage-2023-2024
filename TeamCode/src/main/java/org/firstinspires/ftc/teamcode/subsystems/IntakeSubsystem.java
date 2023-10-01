package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.*;

public class IntakeSubsystem {
    Telemetry telemetry;
    private final DcMotorEx intakeMotor;
    private final ServoEx intakeServo;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        // Initialize the servo
        intakeServo = new SimpleServo(hardwareMap, INTAKE_SERVO_NAME, MIN_ANGLE, MAX_ANGLE);
        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);

        // Initialize the motor
        intakeMotor.setDirection(INTAKE_MOTOR_DIRECTION);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Set the servo to the down position
    public void downPosition() {
        intakeServo.turnToAngle(INTAKE_DOWN_POSITION);
    }

    public void mediumPosition() {
        intakeServo.turnToAngle(INTAKE_DRIVING_POSITION);
    }

    // Set the servo to the up position
    public void upPosition() {
        intakeServo.turnToAngle(INTAKE_UP_POSITION);
    }

    // Set the motor to intake
    public void intake() {
        intakeMotor.setPower(INTAKE_SPEED);
    }

    // Set the motor to outtake
    public void outtake() {
        intakeMotor.setPower(OUTTAKE_SPEED);
    }

    // Stop the motor
    public void stop() {
        intakeMotor.setPower(0);
    }
}
