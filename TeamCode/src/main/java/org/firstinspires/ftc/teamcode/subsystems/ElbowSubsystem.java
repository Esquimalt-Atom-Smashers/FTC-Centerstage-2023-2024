package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.ElbowConstants.*;

import java.util.Arrays;

public class ElbowSubsystem {
    private final DcMotorEx elbowMotor;
    private final Telemetry telemetry;

    private TargetRotation position = TargetRotation.DEFAULT;
    private final TargetRotation[] positions = TargetRotation.class.getEnumConstants();

    public ElbowSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        elbowMotor = hardwareMap.get(DcMotorEx.class, ELBOW_MOTOR_NAME);

        elbowMotor.setDirection(ELBOW_MOTOR_DIRECTION);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.telemetry = telemetry;
    }

    public void rotateNext() {
        if (position == TargetRotation.RIGGING) return;

        rotateTo(positions[Arrays.asList(positions).indexOf(position) + 1]);
    }

    public void rotatePrev() {
        if (position == TargetRotation.DEFAULT) return;

        rotateTo(positions[Arrays.asList(positions).indexOf(position) - 1]);
    }

    public void rotateTo(TargetRotation target) {
        while (elbowMotor.getCurrentPosition() < target.getTarget() - CONTROL_LOOP_TOLERANCE ||
                elbowMotor.getCurrentPosition() > target.getTarget() + CONTROL_LOOP_TOLERANCE) {
            elbowMotor.setPower(ELBOW_PID_CONTROLLER.calculate(target.getTarget(), elbowMotor.getCurrentPosition()));
        }
        position = target;
    }

    // Use of this method will ruin the positioning-tracking of the elbow.
    public void rotateTo(int deg) {
        double target  = deg * PULSES_PER_DEG;
        while (elbowMotor.getCurrentPosition() < target - CONTROL_LOOP_TOLERANCE ||
                elbowMotor.getCurrentPosition() > target + CONTROL_LOOP_TOLERANCE) {
            elbowMotor.setPower(ELBOW_PID_CONTROLLER.calculate(target, elbowMotor.getCurrentPosition()));
        }
    }

    public void lift() {
        elbowMotor.setPower(LIFT_POWER);
    }

    public void drop() {
        elbowMotor.setPower(DROP_POWER);
    }

    public enum TargetRotation {
        DEFAULT(0),
        LOW(-1),
        MEDIUM(-1),
        HIGH(-1),
        RIGGING(-1);

        private int deg;
        TargetRotation(int deg) {
            this.deg = deg;
        }

        int getTarget() {
            return (int) (deg * PULSES_PER_DEG);
        }
    }
}
