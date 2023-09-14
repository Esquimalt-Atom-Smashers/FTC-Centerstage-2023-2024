package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PIDController {
    private final ElapsedTime timer = new ElapsedTime();

    private final double Kp;
    private final double Ki;
    private final double Kd;
    private final boolean angleWrap;

    private double lastError = 0;
    private double integral = 0;

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        angleWrap = false;
    }

    public PIDController(double Kp, double Ki, double Kd, boolean angleWrap) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.angleWrap = angleWrap;
    }

    public double calculate(double reference, double state) {
        double error;
        double derivative;

        if (angleWrap) {
            error = AngleUnit.normalizeRadians(reference - state);
        }
        else {
            error = reference - state;
        }

        integral += error * timer.seconds();
        derivative = (error - lastError) / timer.seconds();

        double output = (error * Kp) + (integral * Ki) + (derivative * Kd);

        timer.reset();
        lastError = error;

        return output;
    }
}