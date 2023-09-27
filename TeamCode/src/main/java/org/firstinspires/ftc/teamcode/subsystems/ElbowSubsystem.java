package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
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

    public void spinManual() {
        elbowMotor.setPower(MANUAL_MOTOR_SPEED);
    }

    public void counterSpinManual() {
        elbowMotor.setPower(-MANUAL_MOTOR_SPEED);
    }

    public void runPID() {
        controller.setPID(P, I, D);
        int elbowPosition = elbowMotor.getCurrentPosition();
        double power = controller.calculate(elbowPosition, target);
        elbowMotor.setPower(power);
//        telemetry.addData("pos ", elbowPosition);
//        telemetry.addData("Target ", target);
//        telemetry.update();
    }

    public boolean isAtPosition() {
        return Math.abs(target - elbowMotor.getCurrentPosition()) <= TOLERANCE;
    }
}
