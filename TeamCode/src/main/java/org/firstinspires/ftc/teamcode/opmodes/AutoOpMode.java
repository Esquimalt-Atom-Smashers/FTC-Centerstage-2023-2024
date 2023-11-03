package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
public class AutoOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public boolean canContinue() {
        return opModeIsActive() && !isStopRequested();
    }
}
