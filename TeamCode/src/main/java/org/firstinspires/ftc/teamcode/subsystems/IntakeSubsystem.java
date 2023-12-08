package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.*;

/**
 * A subsystem that represents the servo and motor that control the intake.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx intakeMotor;
    private final ServoEx intakeServo;

    /**
     * Creates a new ElbowSubsystem.
     * @param hardwareMap The hardware map of the robot
     */
    public IntakeSubsystem(HardwareMap hardwareMap) {
        // Initialize the servo
        intakeServo = new SimpleServo(hardwareMap, INTAKE_SERVO_NAME, MIN_ANGLE, MAX_ANGLE);
        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);

        // Initialize the motor
        intakeMotor.setDirection(INTAKE_MOTOR_DIRECTION);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Lower the intake by setting the servo to the down position.
     */
    public void downPosition() {
        intakeServo.turnToAngle(INTAKE_DOWN_POSITION);
    }

    /**
     * Half raise the intake by setting the servo to the medium position.
     */
    public void mediumPosition() {
        intakeServo.turnToAngle(INTAKE_DRIVING_POSITION);
    }

    /**
     * Raise the intake by setting the servo to the high position.
     */
    public void upPosition() {
        intakeServo.turnToAngle(INTAKE_UP_POSITION);
    }

    /**
     * Start intaking by moving the motor.
     */
    public void intake() {
        intakeMotor.setPower(INTAKE_SPEED);
    }

    /**
     * Start outtaking by moving the motor backwards.
     */
    public void outtake() {
        intakeMotor.setPower(OUTTAKE_SPEED);
    }

    /**
     * Stop intaking.
     */
    public void stop() {
        intakeMotor.setPower(0);
    }
}
