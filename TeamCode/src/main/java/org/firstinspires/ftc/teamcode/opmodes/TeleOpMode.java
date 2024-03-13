package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * OpMode that isn't used after autonomous.
 */
@TeleOp(name = "Tele OpMode", group = "Real")
public class TeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this, false, true);

        telemetry.addLine("This op mode is the one of the main op mode used for controlling the robot.");
        telemetry.addLine("While enabled, use the two controllers to control the drive base, intake, elbow, slide, box, and hanging subsystems through a mixture of manual and scheduled commands.");
        telemetry.addLine("The main difference from MainOpMode is that this resets the encoders before starting.");
        telemetry.update();

        waitForStart();
        robot.start();

        while (opModeIsActive() && !isStopRequested()) {
            robot.run();
        }
    }
}
