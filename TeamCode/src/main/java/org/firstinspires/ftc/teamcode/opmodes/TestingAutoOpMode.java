package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

@Autonomous(group = "auto")
public class TestingAutoOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(11, -59.4, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(11, -35), Math.toRadians(90))
                .turn(Math.toRadians(0))
                .waitSeconds(.5)
                .lineToSplineHeading(new Pose2d(47, -35, Math.toRadians(0)))
                .waitSeconds(.5)
                .back(0.1)
                .splineToSplineHeading(new Pose2d(0, -12, Math.toRadians(180)), Math.toRadians(180))
                // Pick up pixel
                .splineTo(new Vector2d(-58, -11.5), Math.toRadians(180))
                .waitSeconds(0.5)
                .back(1.5)
                .splineToSplineHeading(new Pose2d(0, -12, Math.toRadians(0)), Math.toRadians(0))
                        .build();

        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectorySequence(trajSeq);
    }
}
