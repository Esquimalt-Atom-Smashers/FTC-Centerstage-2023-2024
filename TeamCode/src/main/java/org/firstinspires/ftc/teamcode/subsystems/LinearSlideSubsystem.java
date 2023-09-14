package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Constants.LinearSlideConstants.*;

import java.util.Arrays;

public class LinearSlideSubsystem {
    private final DcMotorEx slideMotor;
    private final Telemetry telemetry;

    private TargetPosition position = TargetPosition.DEFAULT;
    private TargetPosition[] positions = TargetPosition.class.getEnumConstants();

    public LinearSlideSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        slideMotor = hardwareMap.get(DcMotorEx.class, SLIDE_MOTOR_NAME);

        slideMotor.setDirection(SLIDE_MOTOR_DIRECTION);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.telemetry = telemetry;
    }

    public void moveNext() {
        if (position == TargetPosition.MAX) return;

        setPosition(positions[Arrays.asList(positions).indexOf(position) + 1]);
    }

    public void movePrev() {
        if (position == TargetPosition.DEFAULT) return;

        setPosition(positions[Arrays.asList(positions).indexOf(position) - 1]);
    }

    public void setPosition(TargetPosition target) {
        while (slideMotor.getCurrentPosition() < target.getTarget() - CONTROL_LOOP_TOLERANCE ||
                slideMotor.getCurrentPosition() > target.getTarget() + CONTROL_LOOP_TOLERANCE) {
            slideMotor.setPower(SLIDE_PID_CONTROLLER.calculate(target.getTarget(), slideMotor.getCurrentPosition()));
        }
        position = target;
    }

    public void setPosition(int target) {
        while (slideMotor.getCurrentPosition() < target - CONTROL_LOOP_TOLERANCE ||
                slideMotor.getCurrentPosition() > target + CONTROL_LOOP_TOLERANCE) {
            slideMotor.setPower(SLIDE_PID_CONTROLLER.calculate(target, slideMotor.getCurrentPosition()));
        }
    }

    public void extend() {
        if (isMaxExtension()) return;

        slideMotor.setPower(EXTEND_POWER);
    }

    public void retract() {
        if (isMaxRetraction()) return;

        slideMotor.setPower(RETRACT_POWER);
    }

    public boolean isMaxExtension() {
        return slideMotor.getCurrentPosition() >= TargetPosition.MAX.getTarget();
    }

    public boolean isMaxRetraction() {
        return slideMotor.getCurrentPosition() <= TargetPosition.DEFAULT.getTarget();
    }

    public enum TargetPosition {
        DEFAULT(0),
        QUARTER(-1),
        HALF(-1),
        THREE_QUARTER(-1),
        MAX(-1);

        private int inches;
        TargetPosition(int inches) {
            this.inches = inches;
        }

        int getTarget() {
            return (int) (inches * PULSES_PER_INCH);
        }
    }
}
