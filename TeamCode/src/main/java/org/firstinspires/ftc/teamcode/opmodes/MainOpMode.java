package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * OpMode used after autonomous.
 */
@TeleOp(name="Main", group = "Real")
public class MainOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this, false, false);


        int c = 0;
        while (opModeInInit()) {
            c++;
            telemetry.addLine(c % 100 == 0 ? "-  -" : "0  0");
            telemetry.addLine(" u ");
            telemetry.update();
        }

        robot.start();
        while (opModeIsActive() && !isStopRequested()) {
            robot.run();
        }
    }
}
