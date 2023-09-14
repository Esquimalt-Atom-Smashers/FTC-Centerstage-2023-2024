package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

public class MainOpMode extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robot.run();
        }
    }
}
