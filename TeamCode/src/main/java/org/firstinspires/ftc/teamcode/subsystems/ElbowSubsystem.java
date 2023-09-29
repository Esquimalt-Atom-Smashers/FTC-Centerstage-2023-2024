package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.ElbowConstants.*;

public class ElbowSubsystem {
    private final DcMotorEx elbowMotor;

    private PIDController controller;

    public static double target;
    private boolean atTarget = false;

    private final Telemetry telemetry;

    public  ElbowSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        elbowMotor = hardwareMap.get(DcMotorEx.class, ELBOW_DC_MOTOR_NAME);
        elbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        controller = new PIDController(P, I, D);

        this.telemetry = telemetry;
    }

    public void intakePosition() {
        target = INTAKE_POSITION;
    }

    public void drivingPosition() {
        target = DRIVING_POSITION;
    }

    public void levelPosition() {
        target = LEVEL_POSITION;
    }

    public void testPosition() {
        target = TEST_POSITION;
    }

    public void stop() {
        elbowMotor.setPower(0);
    }

    public void raiseManually() {
        elbowMotor.setPower(MANUAL_MOTOR_SPEED);
    }

    public void lowerManually() {
        elbowMotor.setPower(-MANUAL_MOTOR_SPEED);
    }

    public void runPID() {
        controller.setPID(P, I, D);
        int elbowPosition = elbowMotor.getCurrentPosition();
        double power = controller.calculate(elbowPosition, target);
        elbowMotor.setPower(power);
        // If the power we are setting is basically none, we are close enough to the target
        atTarget = power <= POWER_TOLERANCE;
    }

    public boolean isAtTarget() {
        return atTarget;
    }
}
