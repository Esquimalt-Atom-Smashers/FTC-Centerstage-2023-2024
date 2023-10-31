package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;

public class AutonomousController {
    HardwareMap hardwareMap;
    Pose2d startPosition;
    TrajectorySequence pushMovement;
    TrajectorySequence driveToBackdrop;
    int aprilTagID;
    double aprilTagLateralDistance;
    double aprilTagLateralTarget = -4;
    double aprilTagForwardDistance;
    double aprilTagForwardTarget = 14;
    private final SampleMecanumDrive drive;
    private final CameraSubsystem cameraSubsystem;
    private String startPos;
    private Telemetry telemetry;

    public AutonomousController(HardwareMap hardwareMap, String startPos, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.startPos = startPos;
        this.telemetry = telemetry;
        drive = new SampleMecanumDrive(hardwareMap);
        cameraSubsystem = new CameraSubsystem(hardwareMap);
    }

    public void run() {
        // Prepping Road Runner start position
        if (startPos.equals("redLeft")){
            startPosition = new Pose2d(-35.3, -62, Math.toRadians(90));
        }
        if (startPos.equals("redRight")){
            startPosition = new Pose2d(11.5, -62, Math.toRadians(90));
        }
        if (startPos.equals("blueLeft")){
            startPosition = new Pose2d(11.5, 62, Math.toRadians(270));
        }
        if (startPos.equals("blueRight")){
            startPosition = new Pose2d(-35.3, 62, Math.toRadians(270));
        }

        // Sets robot position in Road Runner
        drive.setPoseEstimate(startPosition);

        // Finding Game Element position
        int gameElementPosition = findGameElement();

        // Building unique movement to push the purple pixel
        if (gameElementPosition == -1){
            pushMovement = drive.trajectorySequenceBuilder(startPosition)
                    .forward(30)
                    .turn(Math.toRadians(80))
                    .forward(1)
                    .back(5)
                    .strafeLeft(5)
                    .build();
        }
        if (gameElementPosition == 0){
            pushMovement = drive.trajectorySequenceBuilder(startPosition)
                    .forward(27)
                    .back(5)
                    .build();
        }
        if (gameElementPosition == 1){
            pushMovement = drive.trajectorySequenceBuilder(startPosition)
                    .forward(25)
                    .turn(Math.toRadians(-80))
                    .forward(3)
                    .back(5)
                    .strafeRight(5)
                    .build();
        }

        if (startPos.equals("redRight") || startPos.equals("blueLeft")) { // Case: Go to backdrop and score

            // Building movement to the backdrop & determine April Tag to line up with
            if (startPos.equals("redRight")) {
                // Building Movement to backdrop
                driveToBackdrop = drive.trajectorySequenceBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()))
                        .splineToSplineHeading(new Pose2d(34, -60, 0), 0)
                        .splineToConstantHeading(new Vector2d(35, -35), 0)
                        .build();

                // Determine April Tag to line up with
                if (gameElementPosition == -1){
                    aprilTagID = 4;
                }
                if (gameElementPosition == 0){
                    aprilTagID = 5;
                }
                if (gameElementPosition == 1){
                    aprilTagID = 6;
                }
            }
            if (startPos.equals("blueLeft")) {
                // Building Movement to backdrop
                driveToBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToSplineHeading(new Pose2d(35, 59, 0), 0)
                        .splineToConstantHeading(new Vector2d(25, 35), 0)
                        .build();

                // Determine April Tag to line up with
                if (gameElementPosition == -1){
                    aprilTagID = 1;
                }
                if (gameElementPosition == 0){
                    aprilTagID = 2;
                }
                if (gameElementPosition == 1){
                    aprilTagID = 3;
                }
            }

            // Commit & do drive instructions (~6s)
            drive.followTrajectorySequence(pushMovement);
            drive.followTrajectorySequence(driveToBackdrop);

            // Line up with april tag
            do {
                // Re-check lateral distance to april tag
                aprilTagLateralDistance = cameraSubsystem.getLateralDistance(aprilTagID);

                // TODO: It seems to not see any april tags even though in CameraTestingOpMode it can

                // Move robot closer to april tag
                if (aprilTagLateralDistance < 0) {
                    telemetry.addData("Should be going left", aprilTagLateralDistance);
//                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(-aprilTagLateralDistance).build());
                }
                if (aprilTagLateralDistance > 0) {
                    telemetry.addData("Should be going right", aprilTagLateralDistance);
//                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(aprilTagLateralDistance).build());
                }
                telemetry.update();
            } while (Math.abs(aprilTagLateralTarget - aprilTagLateralDistance) >= 1); // +-1" tolerance


            // Get to the correct distance away from the backdrop
            do {
                // Re-check distance to april tag
                cameraSubsystem.detect();
                aprilTagForwardDistance = cameraSubsystem.getDistance(aprilTagID);

                telemetry.addData("Distance", aprilTagForwardDistance);

                if (aprilTagForwardDistance != -1) {
                    // Move robot closer to april tag
                    if (aprilTagForwardDistance < aprilTagForwardTarget) {
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(aprilTagForwardTarget - aprilTagForwardDistance).build());
                    }
                    if (aprilTagForwardDistance > aprilTagForwardTarget) {
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(aprilTagForwardDistance - aprilTagForwardTarget).build());
                    }
                }
                telemetry.update();

            } while (Math.abs(aprilTagForwardDistance - aprilTagForwardTarget) >= 0.5); // +-1/2" tolerance
            // TODO: Now move intake, elbow and slide, drop pixel, and win!

        } else { // Case: We don't want to drive to the backdrop and score the yellow pixel
            drive.followTrajectorySequence(pushMovement);
        }
    }

    private int findGameElement(){
        // TODO; To test this auto without color/object detection, return: -1, 0, or 1 based on this guide:
        return 1; // -1: Left, 0: Center, 1: Right
    }

    private double getAprilTagLateralDistance(int aprilTagID){
        // TODO
        return -1;
    }
}
