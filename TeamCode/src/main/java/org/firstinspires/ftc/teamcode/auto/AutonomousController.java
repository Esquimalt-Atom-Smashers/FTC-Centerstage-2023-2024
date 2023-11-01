package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import static org.firstinspires.ftc.teamcode.Constants.CameraConstants.CAMERA_NAME;

public class AutonomousController {
    HardwareMap hardwareMap;
    Pose2d startPosition;
    TrajectorySequence pushMovement;
    TrajectorySequence driveToBackdrop;
    OpenCvCamera camera;
    int aprilTagID;
    int gameElementPosition;
    double aprilTagLateralDistance;
    double aprilTagLateralTarget = -4;
    double aprilTagForwardDistance;
    double aprilTagForwardTarget = 14;
    boolean goToBackDrop;
    private final SampleMecanumDrive drive;
    private final CameraSubsystem cameraSubsystem;
    private final Telemetry telemetry;
    private OpenCVPipeline openCV;

    public AutonomousController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        drive = new SampleMecanumDrive(hardwareMap);
        cameraSubsystem = new CameraSubsystem(hardwareMap);
        setUpCameraWithOpenCV();
    }

    public void run() {
        telemetry.addData("Prop position", gameElementPosition);
        telemetry.update();

//        // Sets robot position in Road Runner
//        drive.setPoseEstimate(startPosition);
//
//        // Do push movement
//        setPushMovement(gameElementPosition);
//        drive.followTrajectorySequence(pushMovement);
//
//        if (goToBackDrop) {
//            drive.followTrajectorySequence(driveToBackdrop);
//            lineUpWithAprilTag();
//        }
    }

    public void redLeft(){
        startPosition = new Pose2d(-35.3, -62, Math.toRadians(90));
        gameElementPosition = openCV.findGameElement(0);
        run();
    }

    public void blueRight(){
        startPosition = new Pose2d(-35.3, 62, Math.toRadians(270));
        gameElementPosition = openCV.findGameElement(1);
        run();
    }

    public void redRight(){
        goToBackDrop = true;
        startPosition = new Pose2d(11.5, -62, Math.toRadians(90));
        gameElementPosition = openCV.findGameElement(0);

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
        gameElementPosition = openCV.findGameElement(1);

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
        switch (gameElementPosition) {
            case -1:
                pushMovement = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .forward(30)
                        .turn(Math.toRadians(80))
                        .forward(1)
                        .back(5)
                        .strafeLeft(5)
                        .build();
            case 0:
                pushMovement = drive.trajectorySequenceBuilder(startPosition)
                        .forward(27)
                        .back(5)
                        .build();
            case 1:
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
            aprilTagLateralDistance = cameraSubsystem.getLateralDistance(aprilTagID);

            // TODO: It seems to not see any april tags even though in CameraTestingOpMode it can

            // Move robot closer to april tag
            if (aprilTagLateralDistance < 0) {
                telemetry.addData("Should be going left", aprilTagLateralDistance);
                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(-aprilTagLateralDistance).build());
            }
            if (aprilTagLateralDistance > 0) {
                telemetry.addData("Should be going right", aprilTagLateralDistance);
                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(aprilTagLateralDistance).build());
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
    }

    private void setUpCameraWithOpenCV(){
        openCV = new OpenCVPipeline();
        openCV.passTelemetry(telemetry);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, CAMERA_NAME);

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }
            @Override
            public void onError(int errorCode) {}
        });

        camera.setPipeline(openCV);
    }
}
