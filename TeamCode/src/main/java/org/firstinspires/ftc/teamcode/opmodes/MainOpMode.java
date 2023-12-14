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

    /* TODO:
        Configure PID values (elbow + slide)
        Get distance value (elbow + slide)
        Check distance sensor logic
        Find servo values for intake
        Implement timeout for PID (elbow + slide)
        Configure road runner again
    */

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this, false, false);

        robot.start();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robot.run();
        }
    }
}
