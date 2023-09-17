package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.*;
import static org.firstinspires.ftc.teamcode.Constants.LinearSlideConstants.PULSES_PER_INCH;

public class DriveSubsystem {
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx rearLeft;
    private final DcMotorEx rearRight;

    private BNO055IMU gyro;

    private double snapTarget;
    private double positionTarget;
    private double halfStepTarget;

    private AprilTagDetection tagInAlignment;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        frontLeft = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_MOTOR_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_MOTOR_NAME);
        rearLeft = hardwareMap.get(DcMotorEx.class, REAR_LEFT_MOTOR_NAME);
        rearRight = hardwareMap.get(DcMotorEx.class, REAR_RIGHT_MOTOR_NAME);

        frontLeft.setDirection(FRONT_LEFT_MOTOR_DIRECTION);
        frontRight.setDirection(FRONT_RIGHT_MOTOR_DIRECTION);
        rearLeft.setDirection(REAR_LEFT_MOTOR_DIRECTION);
        rearRight.setDirection(REAR_RIGHT_MOTOR_DIRECTION);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro.initialize(parameters);
    }

    public void drive(double forward, double strafe, double angular) {
        double gyroRads = Math.toRadians(-gyro.getAngularOrientation().firstAngle);
        double rotateY = FIELD_CENTRIC ? strafe * Math.sin(gyroRads) + forward * Math.cos(gyroRads) : forward;
        double rotateX = FIELD_CENTRIC ? strafe * Math.cos(gyroRads) - forward * Math.sin(gyroRads) : strafe;

        frontLeft.setPower(Range.clip(rotateY + rotateX + angular, -1, 1) * POWER_MULTIPLIER);
        frontRight.setPower(Range.clip(rotateY - rotateX - angular, -1, 1) * POWER_MULTIPLIER);
        rearLeft.setPower(Range.clip(rotateY - rotateX + angular, -1, 1) * POWER_MULTIPLIER);
        rearRight.setPower(Range.clip(rotateY + rotateX - angular, -1, 1) * POWER_MULTIPLIER);
    }

    public void autoSnap() {
        snapTarget = 90;
        if (getNormalizedAngle() >= -90) drive(0, 0, AUTO_SNAP_POWER);
        if (getNormalizedAngle() <= -90) drive(0, 0, -AUTO_SNAP_POWER);
    }

    public void centerWithTag(AprilTagDetection tag) {
        if (tag.ftcPose.x < 0) {
            drive(0, 0.5, 0.0);
        }
        if (tag.ftcPose.x > 0) {
            drive(0, -0.5, 0.0);
        }
        tagInAlignment = tag;
    }

    public boolean isCentered() {
        return tagInAlignment.ftcPose.x + TAG_ALIGNMENT_TOLERANCE <= 0 &&
                tagInAlignment.ftcPose.x - TAG_ALIGNMENT_TOLERANCE >= 0;
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    public void halfStepLeft() {
        setMotorPositions(TargetPosition.HALF_STEP_LEFT.getTarget());
        drive(STEPPING_POWER, 0, 0);
    }

    public void halfStepRight() {
        setMotorPositions(TargetPosition.HALF_STEP_RIGHT.getTarget());
        drive(STEPPING_POWER, 0, 0);
    }

    public int getNormalizedAngle() {
        return (int) AngleUnit.normalizeDegrees(-gyro.getAngularOrientation().firstAngle);
    }

    public boolean aprilTagIsAligned() {
        return false;
    }

    public boolean isFinishedSteppingLeft() {
        return getAverageMotorPosition() + STEPPING_TOLERANCE <= TargetPosition.HALF_STEP_LEFT.getTarget() &&
                getAverageMotorPosition() - STEPPING_TOLERANCE >= TargetPosition.HALF_STEP_LEFT.getTarget();
    }

    public boolean isFinishedSteppingRight() {
        return getAverageMotorPosition() + STEPPING_TOLERANCE <= TargetPosition.HALF_STEP_RIGHT.getTarget() &&
                getAverageMotorPosition() - STEPPING_TOLERANCE >= TargetPosition.HALF_STEP_RIGHT.getTarget();
    }

    public boolean isFinishedSnapping() {
        return getNormalizedAngle() + AUTO_SNAP_TOLERANCE_DEG <= snapTarget &&
                getNormalizedAngle() - AUTO_SNAP_TOLERANCE_DEG >= snapTarget;
    }

    private int getAverageMotorPosition() {
        return (frontLeft.getCurrentPosition() +
                frontRight.getCurrentPosition() +
                rearLeft.getCurrentPosition() +
                rearRight.getCurrentPosition()) / 4;
    }

    private void setMotorPositions(int position) {
        frontLeft.setTargetPosition(position);
        frontRight.setTargetPosition(position);
        rearLeft.setTargetPosition(position);
        rearRight.setTargetPosition(position);
    }

    public void setPositionTarget(TargetPosition position) {

    }

    enum TargetPosition {
        POSITION_1(0),
        POSITION_2(-1),
        POSITION_3(-1),
        HALF_STEP_LEFT(-1),
        HALF_STEP_RIGHT(-1);

        private int inches;
        TargetPosition(int inches) {
            this.inches = inches;
        }

        int getTarget() {
            return (int) (inches * PULSES_PER_INCH);
        }
    }


}
