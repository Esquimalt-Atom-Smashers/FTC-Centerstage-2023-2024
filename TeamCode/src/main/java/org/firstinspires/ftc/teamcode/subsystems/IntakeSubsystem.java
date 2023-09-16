package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.*;

public class IntakeSubsystem {

    private final DcMotorEx intakeMotor;
    private final Servo intakeServo;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeServo = hardwareMap.servo.get(INTAKE_SERVO_NAME);
        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);

        intakeMotor.setDirection(INTAKE_MOTOR_DIRECTION);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void lowerIntake() {
        intakeServo.setPosition(INTAKE_DOWN_POSITION);
    }

    public void raiseIntake() {
        intakeServo.setPosition(INTAKE_UP_POSITION);
    }

    public void intake() {
        intakeMotor.setPower(INTAKE_SPEED);
    }

    public void outtake() {
        intakeMotor.setPower(OUTTAKE_SPEED);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}
