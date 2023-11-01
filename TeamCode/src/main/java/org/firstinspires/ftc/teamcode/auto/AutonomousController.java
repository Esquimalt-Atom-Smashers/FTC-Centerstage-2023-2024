package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;

public class AutonomousController {
    private final Telemetry telemetry;

    private final SampleMecanumDrive drive;
    private final CameraSubsystem cameraSubsystem;

    Pose2d startPosition;
    TrajectorySequence pushMovement;
    TrajectorySequence driveToBackdrop;
    int gameElementPosition;
    int aprilTagID;
    double lateralDistance;
    double lateralTarget = 0;
    double forwardDistance;
    double forwardTarget = 14;
    boolean goToBackDrop;

    public AutonomousController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        drive = new SampleMecanumDrive(hardwareMap);
        cameraSubsystem = new CameraSubsystem(hardwareMap);
        gameElementPosition = findGameElement();
    }

    public void run() {
        // Sets robot position in Road Runner
        drive.setPoseEstimate(startPosition);

        // Do push movement
        setPushMovement(gameElementPosition);
        drive.followTrajectorySequence(pushMovement);

        if (goToBackDrop) {
            drive.followTrajectorySequence(driveToBackdrop);
            lineUpWithAprilTag();
        }
    }

    public void redLeft(){
        startPosition = new Pose2d(-35.3, -62, Math.toRadians(90));
        run();
    }

    public void blueRight(){
        startPosition = new Pose2d(-35.3, 62, Math.toRadians(270));
        run();
    }

    public void redRight(){
        goToBackDrop = true;
        startPosition = new Pose2d(11.5, -62, Math.toRadians(90));

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
        run();
    }

    public void blueLeft(){
        goToBackDrop = true;
        startPosition = new Pose2d(11.5, 62, Math.toRadians(270));

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
        run();
    }

    private void setPushMovement(int gameElementPosition){
        if (gameElementPosition == -1){
            pushMovement = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
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
    }

    private void lineUpWithAprilTag(){
        // Line up with april tag
        do {
            // Re-check lateral distance to april tag
            lateralDistance = cameraSubsystem.getLateralDistance(aprilTagID);
            telemetry.addData("Lateral Distance", lateralDistance);

            // TODO: It seems to not see any april tags even though in CameraTestingOpMode it can

            // Move robot closer to april tag
            if (lateralDistance < 0) {
                telemetry.addData("Should be going left", lateralDistance);
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(-lateralDistance).build());
            }
            if (lateralDistance > 0) {
                telemetry.addData("Should be going right", lateralDistance);
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(lateralDistance).build());
            }
            telemetry.update();
        } while (Math.abs(lateralTarget - lateralDistance) >= 1); // +-1" tolerance


        // Get to the correct distance away from the backdrop
        do {
            // Re-check distance to april tag
            cameraSubsystem.detect();
            forwardDistance = cameraSubsystem.getDistance(aprilTagID);

            telemetry.addData("Distance", forwardDistance);

            if (forwardDistance != -1) {
                // Move robot closer to april tag
                if (forwardDistance < forwardTarget) {
                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(forwardTarget - forwardDistance).build());
                }
                if (forwardDistance > forwardTarget) {
                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(forwardDistance - forwardTarget).build());
                }
            }
            telemetry.update();

        } while (Math.abs(forwardDistance - forwardTarget) >= 0.5); // +-1/2" tolerance
    }

    private int findGameElement(){
        // TODO; To test this auto without color/object detection, return: -1, 0, or 1 based on this guide:
        return 1; // -1: Left, 0: Center, 1: Right
    }
}
