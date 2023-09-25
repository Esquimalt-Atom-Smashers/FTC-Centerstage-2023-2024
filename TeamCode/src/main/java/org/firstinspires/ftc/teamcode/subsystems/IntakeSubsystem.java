package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.*;

public class IntakeSubsystem {
    Telemetry telemetry;
    private final DcMotorEx intakeMotor;
    private final ServoEx intakeServo;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeServo = new SimpleServo(hardwareMap, INTAKE_SERVO_NAME, MIN_ANGLE, MAX_ANGLE);
        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);

        intakeMotor.setDirection(INTAKE_MOTOR_DIRECTION);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void lowerIntake() {
        intakeServo.turnToAngle(INTAKE_DOWN_POSITION);
    }

    public void raiseIntake() {
        intakeServo.turnToAngle(INTAKE_UP_POSITION);
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
