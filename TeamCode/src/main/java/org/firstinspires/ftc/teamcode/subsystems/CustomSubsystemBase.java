package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CustomSubsystemBase extends SubsystemBase {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;

    public CustomSubsystemBase(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void printData() {

    }
}
