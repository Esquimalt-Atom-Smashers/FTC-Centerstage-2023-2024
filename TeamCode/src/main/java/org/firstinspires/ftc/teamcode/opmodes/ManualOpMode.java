package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * OpMode that doesn't use commands to control the robot.
 */
@TeleOp(name = "Manual", group = "Testing")
public class ManualOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this, true, false);

        telemetry.addLine("This op mode is one of the main op modes used to control the robot.");
        telemetry.addLine("While enabled, use the two controllers to control the drive base, intake, elbow, slide, box, and hanging subsystems only using manual controls.");
        telemetry.addLine("This doesn't use the command scheduler or any PID controllers");
        telemetry.update();

        waitForStart();
        robot.startManual();

        while (opModeIsActive() && !isStopRequested()) {
            robot.runManually();
        }
    }
}
