package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

// TODO: Make all subsystem extend from this
public abstract class CustomSubsystemBase {

    protected HardwareMap hardwareMap;
    protected Robot robot;

    public CustomSubsystemBase(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
    }

    abstract void printData(Telemetry telemetry);
}