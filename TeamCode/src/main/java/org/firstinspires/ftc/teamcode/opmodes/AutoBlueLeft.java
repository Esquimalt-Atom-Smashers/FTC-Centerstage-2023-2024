package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutonomousController;

@Autonomous(group = "auto")
public class AutoBlueLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (isStopRequested()) return;
        new AutonomousController(hardwareMap, telemetry).blueLeft();
    }
}
