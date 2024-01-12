package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(group = "auto")
public class TestingAutoOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);

        waitForStart();

        driveSubsystem.drive(6);
        waitForSeconds(1);
        driveSubsystem.strafe(6);
        waitForSeconds(1);
        driveSubsystem.drive(-6);
        waitForSeconds(1);
        driveSubsystem.strafe(-6);
        driveSubsystem.stopMotors();
    }

    private void waitForSeconds(float seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() <= seconds) {}
    }
}
