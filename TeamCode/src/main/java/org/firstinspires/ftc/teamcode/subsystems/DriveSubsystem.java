package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.*;

public class DriveSubsystem {
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx rearLeft;
    private final DcMotorEx rearRight;

    private BNO055IMU gyro;

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

        // This is not a typo, this allows us to still set 'raw' power to the motors. Important for PID control.
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
}
