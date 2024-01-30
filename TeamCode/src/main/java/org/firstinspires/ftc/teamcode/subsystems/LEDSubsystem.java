package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.LEDConstants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LEDSubsystem extends CustomSubsystemBase {

    private final RevBlinkinLedDriver ledDriver;

    public LEDSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, LED_NAME);
    }

    public void setLights(RevBlinkinLedDriver.BlinkinPattern pattern) {
        ledDriver.setPattern(pattern);
    }

    public void setBlue() {
        setLights(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
    }

    public void setRed() {
        setLights(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
    }
}
