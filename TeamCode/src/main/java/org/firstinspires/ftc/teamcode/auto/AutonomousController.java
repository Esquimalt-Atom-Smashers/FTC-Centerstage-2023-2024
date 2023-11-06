package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.MoveElbowCommand;
import org.firstinspires.ftc.teamcode.commands.MoveSlideCommand;
import org.firstinspires.ftc.teamcode.opmodes.AutoOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvWebcam;

public class AutonomousController {
    private AutoOpMode opMode;
    HardwareMap hardwareMap;
    Pose2d startPosition;
    TrajectorySequence pushMovementPreOuttake;
    TrajectorySequence pushMovementPostOuttake;
    TrajectorySequence driveToBackdrop;
    int aprilTagID;
    int gameElementPosition;
    double aprilTagLateralDistance;
    double aprilTagLateralTarget = -4;
    double aprilTagForwardDistance;
    double aprilTagForwardTarget = 14;
    double extraMovement;
    boolean goToBackDrop;
    private final SampleMecanumDrive drive;
    private final Telemetry telemetry;
    private final ClawSubsystem claw;
    private final IntakeSubsystem intake;
    private final ElbowSubsystem elbow;
    private final LinearSlideSubsystem slide;
    private OpenCVPipeline pipeline;
    private int allianceColor;
    private OpenCvWebcam webcam;

    public AutonomousController(AutoOpMode autoOpMode) {
        this(autoOpMode.hardwareMap, autoOpMode.telemetry);
        this.opMode = autoOpMode;
    }

    public AutonomousController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        drive = new SampleMecanumDrive(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        elbow = new ElbowSubsystem(hardwareMap);
        slide = new LinearSlideSubsystem(hardwareMap);
        updateStatus("Not Ready (Starting OpenCVPipeline)");
        startOpenCV();
        while (!pipeline.cameraReady && canContinue()) updateStatus("Not Ready (Waiting for camera)");
        updateStatus("Ready");
    }

    public void run() {
        while (!pipeline.cameraReady && canContinue()) updateStatus("Waiting for camera");
        gameElementPosition = pipeline.findGameElement(allianceColor);
        webcam.closeCameraDevice();

        // Initial subsystem movements
        MoveElbowCommand elbowCommand = new MoveElbowCommand(elbow, Constants.ElbowConstants.LOW_SCORING_POSITION);
        SequentialCommandGroup autoStartCommand = new SequentialCommandGroup(
                new InstantCommand(claw::closeClawSingle, claw),
                new WaitCommand(500),
                new InstantCommand(intake::mediumPosition, intake),
                new WaitCommand(1000),
                elbowCommand
        );
        autoStartCommand.schedule();
        while (!elbowCommand.isFinished() && canContinue()) {
            CommandScheduler.getInstance().run();
        }

        // Sets robot position in Road Runner
        drive.setPoseEstimate(startPosition);
        telemetry.addData("Start Position", startPosition.getX());
        telemetry.update();

        // Do push movement
        updateStatus("Pushing pixel to: " + gameElementPosition + " position");
        setPrePushMovement(gameElementPosition);
        drive.followTrajectorySequence(pushMovementPreOuttake);

        // Place purple pixel
        WaitCommand finalCommand = new WaitCommand(1);
        new SequentialCommandGroup(
                new InstantCommand(intake::downPosition, intake),
                new WaitCommand(250),
                new InstantCommand(intake::intake, intake),
                new WaitCommand(250),
                new InstantCommand(intake::stop, intake),
                new InstantCommand(intake::mediumPosition, intake),
                finalCommand
        ).schedule();
        while (!finalCommand.isFinished()){
            CommandScheduler.getInstance().run();
        }

        setPostPushMovement(gameElementPosition);
        drive.followTrajectorySequence(pushMovementPostOuttake);

        if (goToBackDrop) {
            // Driving to backdrop
            updateStatus("Driving to backdrop");
            if (allianceColor == 1) { // Blue side movement
                driveToBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToSplineHeading(new Pose2d(48, 35), 0)
                        .strafeRight(extraMovement)
                        .build();
            } else { // Red side movement
                driveToBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToSplineHeading(new Pose2d(48, -35), 0)
                        .strafeRight(extraMovement)
                        .build();
            }
            drive.followTrajectorySequence(driveToBackdrop);

            // Scoring yellow pixel
            updateStatus("Moving slide");
            MoveSlideCommand slideCommand = new MoveSlideCommand(slide, Constants.LinearSlideConstants.LOW_SCORING_POSITION);
            new SequentialCommandGroup(
                    new InstantCommand(intake::downPosition),
                    slideCommand,
                    new InstantCommand(claw::openClaw, claw),
                    new WaitCommand(110)
            ).schedule();
            while (!slideCommand.isFinished() && canContinue()) {
                CommandScheduler.getInstance().run();
            }
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .back(5)
                    .turn(Math.toRadians(-85))
                    .build());

            updateStatus("Finished");
        }
    }

    public void redLeft(){
        startPosition = new Pose2d(-35.3, -62, Math.toRadians(90));
        allianceColor = 0;
        run();
    }

    public void blueRight(){
        startPosition = new Pose2d(-35.3, 62, Math.toRadians(270));
        allianceColor = 1;
        run();
    }

    public void redRight(){
        goToBackDrop = true;
        startPosition = new Pose2d(11.5, -62, Math.toRadians(90));
        allianceColor = 0;

//        // Building Movement to backdrop
//        driveToBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .splineToSplineHeading(new Pose2d(34, -60, 0), 0)
//                .splineToConstantHeading(new Vector2d(35, -35), 0)
//                .build();

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
        allianceColor = 1;

//        // Building Movement to backdrop
//        driveToBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .splineToSplineHeading(new Pose2d(35, 59, 0), 0)
//                .splineToConstantHeading(new Vector2d(25, 35), 0)
//                .build();

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

    private void setPrePushMovement(int gameElementPosition){
        if (gameElementPosition == -1){
            pushMovementPreOuttake = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(24)
                    .turn(Math.toRadians(90))
                    .strafeRight(5)
                    .build();
            extraMovement = -6;
        }
        if (gameElementPosition == 0){
            pushMovementPreOuttake = drive.trajectorySequenceBuilder(startPosition)
                    .forward(24)
                    .strafeLeft(8)
                    .back(2)
                    .build();
            extraMovement = 0.1;
        }
        if (gameElementPosition == 1){
            pushMovementPreOuttake = drive.trajectorySequenceBuilder(startPosition)
                    .forward(24)
                    .turn(Math.toRadians(-90))
                    .strafeLeft(5)
                    .back(2)
                    .build();
            extraMovement = 9;
        }
    }

    private void setPostPushMovement(int gameElementPosition){
        if(gameElementPosition == -1) {
            pushMovementPostOuttake = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeLeft(30)
                    .build();
        } else if (gameElementPosition == 0) {
            pushMovementPostOuttake = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .back(11)
                    .build();
        } else if (gameElementPosition == 1) {
            pushMovementPostOuttake = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeRight(30)
                    .build();
        }
    }

    private void lineUpWithAprilTag(){
        CameraSubsystem cameraSubsystem = new CameraSubsystem(hardwareMap);
        updateStatus("Lining up with april tag ID: " + aprilTagID);

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

        } while (Math.abs(aprilTagForwardDistance - aprilTagForwardTarget) >= 0.25 && canContinue()); // +-1/4" tolerance

//         Line up with april tag
        do {
            // Re-check lateral distance to april tag
            aprilTagLateralDistance = cameraSubsystem.getLateralDistance(aprilTagID);
//
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
        } while (Math.abs(aprilTagLateralTarget - aprilTagLateralDistance) >= 1 && canContinue()); // +-1" tolerance
    }

    private void startOpenCV(){
        pipeline = new OpenCVPipeline();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, Constants.CameraConstants.CAMERA_NAME), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }
            @Override
            public void onError(int errorCode)
            {
            }
        });
    }

    private void updateStatus(String text) {
        telemetry.addData("Status", text);
        telemetry.update();
    }

    private boolean canContinue() {
        // TODO: figure out a way to check if there is a stop requested from opmode, to stop the controller from crashing when the robot stops its auto routine earlier
        return true;
//        return opMode.canContinue();
    }

}
