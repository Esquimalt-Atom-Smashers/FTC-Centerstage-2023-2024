package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.MoveElbowCommand;
import org.firstinspires.ftc.teamcode.commands.MoveSlideCommand;
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
import org.openftc.easyopencv.OpenCvWebcam;

public class AutonomousController {

    enum State {
        PICKING_UP_PIXELS,
        MOVING_TO_SPIKE_MARKS,
        MOVING_FROM_SPIKE_MARKS,
        PLACING_PURPLE,
        MOVING_TO_BACKDROP,
        PLACING_YELLOW,
        HIDING,
        IDLE
    }

    enum SpikeMark {
        LEFT,
        RIGHT,
        MIDDLE
    }

    /*
    TODO after scrimmage: Implement a finite state machine like
        https://github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/AsyncFollowingFSM.java
        so we can move PID controlled subsystem while we are moving
    */
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    // Positional things and trajectories
    private Pose2d startPosition;
    private TrajectorySequence pushMovementPreOuttake;
    private TrajectorySequence pushMovementPostOuttake;
    private TrajectorySequence driveToBackdrop;

    // Trajectory variables
    private int aprilTagID;
    private int gameElementPosition;
    private double extraMovement;
    private boolean goToBackDrop;
    private double aprilTagLateralDistance;
    private double aprilTagLateralTarget = -4;
    private double aprilTagForwardDistance;
    private double aprilTagForwardTarget = 14;

    // Subsystems used by the autonomous
    private final SampleMecanumDrive drive;
    private final ClawSubsystem claw;
    private final IntakeSubsystem intake;
    private final ElbowSubsystem elbow;
    private final LinearSlideSubsystem slide;

    // Camera things
    private OpenCVPipeline pipeline;
    private OpenCvWebcam webcam;

    private boolean isBlueAlliance;
    private boolean isUpstage;
    private boolean isPlacingYellow;

    private float positionMultiplier;
    private float rotationalOffset;

    private State currentState = State.IDLE;
    private SpikeMark spikeMarkPosition;

    private SequentialCommandGroup setUpCommand;
    private SequentialCommandGroup placePurplePixel;
    private SequentialCommandGroup armToPlacePosition;
    private SequentialCommandGroup placeYellowPixel;

    private WaitCommand currentCommand;


    public AutonomousController(LinearOpMode autoOpMode) {
        this(autoOpMode.hardwareMap, autoOpMode.telemetry);
    }

    public AutonomousController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        drive = new SampleMecanumDrive(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        elbow = new ElbowSubsystem(hardwareMap);
        elbow.resetEncoder();
        slide = new LinearSlideSubsystem(hardwareMap);
        slide.resetEncoder();
        updateStatus("Not Ready (Starting OpenCVPipeline)");
        startOpenCV();
        while (!pipeline.cameraReady) updateStatus("Not Ready (Waiting for camera) Do not press stop");
        updateStatus("Ready");
    }

    public void initializeCommands() {
        setUpCommand = new SequentialCommandGroup(
                new MoveSlideCommand(slide, Constants.LinearSlideConstants.IN_POSITION),
                new MoveElbowCommand(elbow, Constants.ElbowConstants.INTAKE_POSITION),
                new InstantCommand(claw::closeClawSingle, claw),
                new WaitCommand(500),
                new InstantCommand(intake::mediumPosition, intake),
                new MoveElbowCommand(elbow, Constants.ElbowConstants.DRIVING_POSITION)
        );
        placePurplePixel = new SequentialCommandGroup(
                new InstantCommand(intake::downPosition, intake),
                new WaitCommand(250),
                new InstantCommand(intake::intake, intake),
                new WaitCommand(250),
                new InstantCommand(intake::stop, intake),
                new InstantCommand(intake::mediumPosition, intake)
        );
        armToPlacePosition = new SequentialCommandGroup(
                new InstantCommand(intake::downPosition, intake),
                new MoveElbowCommand(elbow, Constants.ElbowConstants.LOW_SCORING_POSITION),
                new MoveSlideCommand(slide, Constants.LinearSlideConstants.LOW_SCORING_POSITION)
        );
        placeYellowPixel = new SequentialCommandGroup(
                new InstantCommand(claw::openClaw, claw),
                new WaitCommand(750),
                new MoveSlideCommand(slide, Constants.LinearSlideConstants.IN_POSITION)
        );
    }

    public void startStateMachine() {
        currentState = State.PICKING_UP_PIXELS;
        currentCommand = scheduleCommand(setUpCommand);
        drive.setPoseEstimate(startPosition);
    }

    public void runStateMachine() {
        switch (currentState) {
            // Picking up the pixel from the ground as well as waiting for the camera to be ready
            case PICKING_UP_PIXELS:
                if (currentCommand.isFinished() && pipeline.cameraReady) {
                    gameElementPosition = pipeline.findGameElement(isBlueAlliance);
                    spikeMarkPosition = gameElementPosition == -1 ? SpikeMark.LEFT : gameElementPosition == 0 ? SpikeMark.MIDDLE : SpikeMark.RIGHT;
                    webcam.closeCameraDevice();
                    currentState = State.MOVING_TO_SPIKE_MARKS;
                    drive.followTrajectorySequenceAsync(driveToSpikeMarks());
                }
                break;
            // Carrying the pixels, moving to the center of the spike marks
            case MOVING_TO_SPIKE_MARKS:
                if (!drive.isBusy()) {
                    currentState = State.PLACING_PURPLE;
                    currentCommand = scheduleCommand(placePurplePixel);
                }
                break;
            // Placing the purple pixel on the spike mark using the intake
            case PLACING_PURPLE:
                if (currentCommand.isFinished()) {
                    // TODO: Make a way to always turn the correct direction at the end of auto
                    if (isUpstage && isPlacingYellow) {
                        currentState = State.MOVING_FROM_SPIKE_MARKS;
                        currentCommand = scheduleCommand(armToPlacePosition);
                        drive.followTrajectorySequenceAsync(driveFromSpikeMarks());
                    }
                    else currentState = State.IDLE;
                }
                break;
            // Moving out of the way of the pixel and game element
            case MOVING_FROM_SPIKE_MARKS:
                if (currentCommand.isFinished() && !drive.isBusy()) {
                    currentState = State.MOVING_TO_BACKDROP;
                    drive.followTrajectorySequenceAsync(driveToBackdrop());
                }
                break;
            // Moving to the correct spot to place the yellow pixel
            case MOVING_TO_BACKDROP:
                if (currentCommand.isFinished()) {
                    currentState = State.PLACING_YELLOW;
                    currentCommand = scheduleCommand(placeYellowPixel);
                }
                break;
            // Placing the yellow pixel on the backstage
            case PLACING_YELLOW:
                if (currentCommand.isFinished()) {
                    currentState = State.HIDING;
                    drive.followTrajectorySequenceAsync(driveToCorner());
                }
                break;
            // Moving to the corner to leave room
            case HIDING:
                if (!drive.isBusy()) {
                    currentState = State.IDLE;
                    // We are done, idle
                }
                break;
            case IDLE:
                break;
        }

        CommandScheduler.getInstance().run();
        drive.update();
        telemetry.addData("State", currentState);
        telemetry.update();

    }

    private WaitCommand scheduleCommand(SequentialCommandGroup command) {
        WaitCommand waitCommand = new WaitCommand(1);
        command.addCommands(waitCommand);
        command.schedule();
        return waitCommand;
    }

    public void setSettings(boolean isBlue, boolean isUpstage, boolean isPlacingYellow) {
        this.isBlueAlliance = isBlue;
        this.isUpstage = isUpstage;
        this.isPlacingYellow = isPlacingYellow;
        // TODO: Check this
        positionMultiplier = isBlueAlliance ? 1 : -1;
        rotationalOffset = isBlueAlliance ? 0 : 180;
        // TODO: Check
        startPosition = new Pose2d(isUpstage ? 11.5 : -35.5, 62 * positionMultiplier, Math.toRadians(isBlue ? 270 : 90));
//        public void redLeft(){
//            startPosition = new Pose2d(-35.3, -62, Math.toRadians(90));
//        public void blueRight(){
//            startPosition = new Pose2d(-35.3, 62, Math.toRadians(270));
//        public void redRight(){
//            startPosition = new Pose2d(11.5, -62, Math.toRadians(90));
//        public void blueLeft(){
//            startPosition = new Pose2d(11.5, 62, Math.toRadians(270));
    }

    // TODO: Test all trajectories
    private TrajectorySequence driveToSpikeMarks() {
        // I think this drives to the correct spike mark position
        if (spikeMarkPosition == SpikeMark.LEFT)
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(24)
                    .turn(Math.toRadians(90))
                    .strafeRight(5)
                    .back(2)
                    .build();
        else if (spikeMarkPosition == SpikeMark.MIDDLE)
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(25)
                    .strafeLeft(8)
                    .build();
        else if (spikeMarkPosition == SpikeMark.RIGHT)
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(24)
                    .turn(Math.toRadians(-90))
                    .strafeLeft(5)
                    .back(2)
                    .build();

        return drive.trajectorySequenceBuilder(drive.getPoseEstimate()).build();
    }

    private TrajectorySequence driveFromSpikeMarks() {
        if (spikeMarkPosition == SpikeMark.LEFT)
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeLeft(30)
                    .build();
        else if (spikeMarkPosition == SpikeMark.MIDDLE)
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeRight(6)
                    .back(11)
                    .build();
        else if (spikeMarkPosition == SpikeMark.RIGHT)
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeRight(30)
                    .build();
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate()).build();
    }

    private TrajectorySequence driveToBackdrop() {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(46, 35 * positionMultiplier, Math.toRadians(0)))
                .strafeRight(extraMovement)
                .build();
    }

    private TrajectorySequence driveToCorner() {
        // TODO: Check the math in this trajectory
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(3)
                .turn(Math.toRadians(isBlueAlliance ? -90 : 90))
                .lineTo(new Vector2d(49, 60 * positionMultiplier))
                .build();
    }

    private TrajectorySequence testing() {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .waitSeconds(1000)
                .build();
    }

    // -------------------------------------------------------------------------------------------

    public void run() {
        while (!pipeline.cameraReady) updateStatus("Waiting for camera");
        gameElementPosition = pipeline.findGameElement(isBlueAlliance);
        webcam.closeCameraDevice();

        // Initial subsystem movements
        MoveElbowCommand elbowCommand = new MoveElbowCommand(elbow, Constants.ElbowConstants.LOW_SCORING_POSITION);
        SequentialCommandGroup autoStartCommand = new SequentialCommandGroup(
//                new MoveElbowCommand(elbow, Constants.ElbowConstants.INTAKE_POSITION),
                new MoveSlideCommand(slide, Constants.LinearSlideConstants.IN_POSITION),
                new InstantCommand(claw::closeClawSingle),
                new WaitCommand(500),
                new InstantCommand(intake::mediumPosition),
                new WaitCommand(1000),
                elbowCommand
        );
        autoStartCommand.schedule();
        while (!elbowCommand.isFinished()) {
            CommandScheduler.getInstance().run();
            updateStatus("Running commands");
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
            if (isBlueAlliance) { // Blue side movement
                driveToBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(46, 35, Math.toRadians(0)))
                        .strafeRight(extraMovement)
                        .build();
            } else { // Red side movement
                driveToBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(46, -35, 0))
                        .strafeRight(extraMovement)
                        .build();
            }
            drive.followTrajectorySequence(driveToBackdrop);

            // Scoring yellow pixel
            updateStatus("Moving slide");
            finalCommand = new WaitCommand(1);
            SequentialCommandGroup placeYellowPixel = new SequentialCommandGroup(
                    new InstantCommand(intake::downPosition),
                    new MoveElbowCommand(elbow, Constants.ElbowConstants.LOW_SCORING_POSITION),
                    new MoveSlideCommand(slide, Constants.LinearSlideConstants.LOW_SCORING_POSITION),
                    new WaitCommand(750),
                    new InstantCommand(claw::openClaw, claw),
                    new WaitCommand(750),
                    new MoveSlideCommand(slide, Constants.LinearSlideConstants.IN_POSITION),
                    finalCommand
            );
            placeYellowPixel.schedule();
            while (!finalCommand.isFinished()) {
                CommandScheduler.getInstance().run();
            }
            if (isBlueAlliance) {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(3)
                        .turn(Math.toRadians(-90))
                        .lineTo(new Vector2d(49, 60))
                        .build());
            } else {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(3)
                        .turn(Math.toRadians(90))
                        .lineTo(new Vector2d(49, -60))
                        .build());
            }

            updateStatus("Finished");
        }
    }

    public void redLeft(){
        startPosition = new Pose2d(-35.3, -62, Math.toRadians(90));
        isBlueAlliance = false;
        run();
    }

    public void blueRight(){
        startPosition = new Pose2d(-35.3, 62, Math.toRadians(270));
        isBlueAlliance = true;
        run();
    }

    public void redRight(){
        goToBackDrop = true;
        startPosition = new Pose2d(11.5, -62, Math.toRadians(90));
        isBlueAlliance = false;

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
        isBlueAlliance = true;

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
                    .back(2)
                    .build();
            extraMovement = -6;
        }
        if (gameElementPosition == 0){
            pushMovementPreOuttake = drive.trajectorySequenceBuilder(startPosition)
                    .forward(25)
                    .strafeLeft(8)
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
                    .strafeRight(6)
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

        } while (Math.abs(aprilTagForwardDistance - aprilTagForwardTarget) >= 0.25); // +-1/4" tolerance

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
        } while (Math.abs(aprilTagLateralTarget - aprilTagLateralDistance) >= 1); // +-1" tolerance
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
}
