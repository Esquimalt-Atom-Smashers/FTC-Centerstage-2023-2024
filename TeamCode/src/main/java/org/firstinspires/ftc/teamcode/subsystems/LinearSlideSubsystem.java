package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.LinearSlideConstants.*;

import java.util.Arrays;

public class LinearSlideSubsystem {
    private final DcMotorEx slideMotor;

    private TargetPosition position = TargetPosition.DEFAULT;
    private TargetPosition[] positions = TargetPosition.class.getEnumConstants();

    public LinearSlideSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        slideMotor = hardwareMap.get(DcMotorEx.class, SLIDE_MOTOR_NAME);

        slideMotor.setDirection(SLIDE_MOTOR_DIRECTION);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void nextPosition() {
        if (position == TargetPosition.HIGH) return;
        // Set our position to the next position
        setPosition(positions[Arrays.asList(positions).indexOf(position) + 1]);
    }

    public void prevPosition() {
        if (position == TargetPosition.DEFAULT) return;
        // Set our position to the previous position
        setPosition(positions[Arrays.asList(positions).indexOf(position) - 1]);
    }

    public void setPosition(TargetPosition targetPosition) {
        // Loop until we get within the tolerance to the target
        while (slideMotor.getCurrentPosition() < targetPosition.getTarget() - TARGET_TOLERANCE ||
                slideMotor.getCurrentPosition() > targetPosition.getTarget() + TARGET_TOLERANCE) {
            slideMotor.setPower(SLIDE_PID_CONTROLLER.calculate(targetPosition.getTarget(), slideMotor.getCurrentPosition()));
        }
        position = targetPosition;
    }

    public void SetPosition(int target) {
        while (slideMotor.getCurrentPosition() < target - TARGET_TOLERANCE ||
                slideMotor.getCurrentPosition() > target + TARGET_TOLERANCE) {
            slideMotor.setPower(SLIDE_PID_CONTROLLER.calculate(target, slideMotor.getCurrentPosition()));
        }
    }

    public void extend() {
        if (isMaxExtension()) {
            stop();
            return;
        }
        slideMotor.setPower(EXTEND_POWER);
    }

    public void retract() {
        if (isMaxRetraction()) {
            stop();
            return;
        }
        slideMotor.setPower(RETRACT_POWER);
    }

    public void stop() {
        slideMotor.setPower(0);
    }

    public boolean isMaxExtension() {
        return slideMotor.getCurrentPosition() >= TargetPosition.HIGH.getTarget();
    }

    public boolean isMaxRetraction() {
        return slideMotor.getCurrentPosition() <= TargetPosition.DEFAULT.getTarget();
    }

    public enum TargetPosition {
        // The number of inches
        // TODO: Figure out the numbers for these
        DEFAULT(0),
        LOW(-1),
        MEDIUM(-1),
        HIGH(-1);

        private int inches;

        TargetPosition(int inches) {
            this.inches = inches;
        }
        int getTarget() {
            return (int) (inches * PULSES_PER_INCH);
        }
    }
}
