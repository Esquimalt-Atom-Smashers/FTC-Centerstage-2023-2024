package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(group = "auto")
public class TestingAutoOpMode extends LinearOpMode {

    double waitTime = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);

        waitForStart();
        driveSubsystem.turn(90);
//        driveSubsystem.drive(6);
//        waitForSeconds(waitTime);
//        driveSubsystem.strafe(6);
//        waitForSeconds(waitTime);
//        for (int i = 0; i < 4; i++) {
//            driveSubsystem.turn(-90);
//            waitForSeconds(waitTime);
//            driveSubsystem.turn(90);
//            waitForSeconds(waitTime);
//        }
//        for (int i = 0; i < 4; i++) {
//            driveSubsystem.drive(12);
//            waitForSeconds(waitTime);
//            driveSubsystem.drive(-12);
//            waitForSeconds(waitTime);
//        }
//        driveSubsystem.strafe(-6);
//        waitForSeconds(waitTime);
//        driveSubsystem.drive(6);
//        waitForSeconds(waitTime);
//        driveSubsystem.turn(90);
        driveSubsystem.stopMotors();
    }

    private void waitForSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() <= seconds) {}
    }
}
