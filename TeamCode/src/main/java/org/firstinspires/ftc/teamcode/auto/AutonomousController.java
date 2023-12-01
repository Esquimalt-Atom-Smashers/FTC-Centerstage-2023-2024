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

/**
 * A controller that controls the drive base and arm during autonomous.
 */
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
//    private TrajectorySequence pushMovementPreOuttake;
//    private TrajectorySequence pushMovementPostOuttake;
//    private TrajectorySequence driveToBackdrop;

    // Trajectory variables
//    private int aprilTagID;
//    private int gameElementPosition;
//    private double extraMovement;
//    private boolean goToBackDrop;
//    private double aprilTagLateralDistance;
//    private double aprilTagLateralTarget = -4;
//    private double aprilTagForwardDistance;
//    private double aprilTagForwardTarget = 14;

    // Subsystems used by the autonomous
    private final SampleMecanumDrive drive;
    private final ClawSubsystem claw;
    private final IntakeSubsystem intake;
    private final ElbowSubsystem elbow;
    private final LinearSlideSubsystem slide;

    // Camera things
    private OpenCVPipeline pipeline;
    private OpenCvWebcam webcam;

    // Settings for controlling what we do
    private boolean isBlueAlliance;
    private boolean isUpstage;
    private boolean isPlacingYellow;
    private boolean isSetUp;

    // Operators used to easily convert positions and angles between red and blue
    private float positionMultiplier;
    private float rotationalOffset;

    // Our state for out state machine
    private State currentState = State.IDLE;

    // The position of the spike mark
    private SpikeMark spikeMarkPosition;

    // Commands run in autonomous
    private SequentialCommandGroup setUpCommand;
    private SequentialCommandGroup placePurplePixel;
    private SequentialCommandGroup armToPlacePosition;
    private SequentialCommandGroup placeYellowPixel;

    private WaitCommand currentCommand;

    /**
     * Creates a new AutonomousController, initializing the subsystems using the HardwareMap from the provided LinearOpMode
     *
     * @param opMode The LinearOpMode that created the controller.
     */
    public AutonomousController(LinearOpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        drive = new SampleMecanumDrive(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        elbow = new ElbowSubsystem(hardwareMap);
        elbow.resetEncoder();
        slide = new LinearSlideSubsystem(hardwareMap);
        slide.resetEncoder();
        updateStatus("Not Ready (Starting OpenCVPipeline)");
        startOpenCV();
        while (!pipeline.cameraReady) updateStatus("Not Ready (Waiting for camera)\nDo not press stop!");
        updateStatus("Ready");
    }

    /**
     * Creates a new AutonomousController, initializing the subsystems using the HardwareMap from the provided LinearOpMode.
     * Also sets settings and initializes the commands using the provided condition
     *
     * @param opMode The LinearOpMode that created the controller.
     * @param isBlueAlliance If we are on the blue alliance.
     * @param isUpstage If we are upstage (closer to the backdrop).
     * @param isPlacingYellow Whether we want to place the yellow pixel on the backdrop.
     */
    public AutonomousController(LinearOpMode opMode, boolean isBlueAlliance, boolean isUpstage, boolean isPlacingYellow) {
        this(opMode);
        setSettings(isBlueAlliance, isUpstage, isPlacingYellow);
    }

    /**
     * Creates a webcam and asynchronously opens an open cv pipeline.
     */
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

    /**
     * Sets the settings for the AutonomousController, controlling where we start and if we want to place a yellow pixel on the backdrop
     *
     * @param isBlue If we are on the blue alliance.
     * @param isUpstage If we are upstage (closer to the backdrop).
     * @param isPlacingYellow Whether we want to place the yellow pixel on the backdrop.
     */
    public void setSettings(boolean isBlue, boolean isUpstage, boolean isPlacingYellow) {
        this.isBlueAlliance = isBlue;
        this.isUpstage = isUpstage;
        this.isPlacingYellow = isPlacingYellow;
        // TODO: Check this
        positionMultiplier = isBlueAlliance ? 1 : -1;
        rotationalOffset = isBlueAlliance ? 0 : 180;
        // TODO: Check
        startPosition = new Pose2d(isUpstage ? 11.5 : -35.5, 62 * positionMultiplier, Math.toRadians(isBlue ? 270 : 90));
        isSetUp = true;
        initializeCommands();
    }

    /**
     * Initializes the commands that move the elbow, slide and claw during autonomous.
     */
    private void initializeCommands() {
        if (!isSetUp) return;
        setUpCommand = new SequentialCommandGroup(
                new MoveSlideCommand(slide, Constants.LinearSlideConstants.IN_POSITION),
                new MoveElbowCommand(elbow, Constants.ElbowConstants.INTAKE_POSITION),
                new InstantCommand(claw::closeClawSingle, claw),
                new InstantCommand(intake::mediumPosition, intake),
                new WaitCommand(500),
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
                new MoveElbowCommand(elbow, Constants.ElbowConstants.LOW_SCORING_POSITION),
                new MoveSlideCommand(slide, Constants.LinearSlideConstants.LOW_SCORING_POSITION)
        );
        placeYellowPixel = new SequentialCommandGroup(
                new InstantCommand(claw::openClaw, claw),
                new WaitCommand(750),
                new MoveSlideCommand(slide, Constants.LinearSlideConstants.IN_POSITION)
        );
    }

    /**
     * Starts the state machine by starting the first command and setting the state to picking up pixels.
     */
    public void startStateMachine() {
        if (!isSetUp) {
            currentState = State.IDLE;
            return;
        }
        currentState = State.PICKING_UP_PIXELS;
        drive.setPoseEstimate(startPosition);
        currentCommand = scheduleCommand(setUpCommand);
    }

    /**
     * Runs the state machine. Switches states if needed, runs commands, updates the drive, and updating the telemetry
     */
    public void runStateMachine() {
        switch (currentState) {
            // Picking up the pixel from the ground as well as waiting for the camera to be ready
            case PICKING_UP_PIXELS:
                if (currentCommand.isFinished() && pipeline.cameraReady) {
                    int gameElementPosition = pipeline.findGameElement(isBlueAlliance);
                    spikeMarkPosition = gameElementPosition == -1 ? SpikeMark.LEFT : gameElementPosition == 0 ? SpikeMark.MIDDLE : SpikeMark.RIGHT;
                    webcam.closeCameraDevice();
                    currentState = State.MOVING_TO_SPIKE_MARKS;
                    drive.followTrajectorySequenceAsync(driveToSpikeMarks());
//                    drive.followTrajectorySequenceAsync(doNothing());
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
//                        drive.followTrajectorySequenceAsync(doNothing());

                    }
                    else currentState = State.IDLE;
                }
                break;
            // Moving out of the way of the pixel and game element
            case MOVING_FROM_SPIKE_MARKS:
                if (currentCommand.isFinished() && !drive.isBusy()) {
                    currentState = State.MOVING_TO_BACKDROP;
                    drive.followTrajectorySequenceAsync(driveToBackdrop());
//                    drive.followTrajectorySequenceAsync(doNothing());

                }
                break;
            // Moving to the correct spot to place the yellow pixel
            case MOVING_TO_BACKDROP:
                if (!drive.isBusy()) {
                    currentState = State.PLACING_YELLOW;
                    currentCommand = scheduleCommand(placeYellowPixel);
                }
                break;
            // Placing the yellow pixel on the backstage
            case PLACING_YELLOW:
                if (currentCommand.isFinished()) {
                    currentState = State.HIDING;
                    drive.followTrajectorySequenceAsync(driveToCorner());
//                    drive.followTrajectorySequenceAsync(doNothing());

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
        telemetry.addData("Spike mark position", spikeMarkPosition);
        telemetry.addData("State", currentState);
        telemetry.update();

    }

    /**
     * Schedules a command to be run. Adds a 1 ms wait commands to the end of the provided command and returns it so you can see when it is done.
     * (Calling isFinished on a SequentialCommandGroup doesn't seem to work properly)
     *
     * @param command The command to be scheduled.
     * @return The wait command at the end of the sequential command.
     */
    private WaitCommand scheduleCommand(SequentialCommandGroup command) {
        WaitCommand waitCommand = new WaitCommand(1);
        command.addCommands(waitCommand);
        command.schedule();
        return waitCommand;
    }

    /**
     * Constructs and returns the trajectory sequence used to drive to the correct spike mark.
     *
     * @return The built trajectory sequence.
     */
    private TrajectorySequence driveToSpikeMarks() {
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
        else /*if (spikeMarkPosition == SpikeMark.RIGHT)*/
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(24)
                    .turn(Math.toRadians(-90))
                    .strafeLeft(5)
                    .back(2)
                    .build();
    }

    /**
     * Constructs and returns the trajectory sequence used to drive away from the spike marks after placing the purple pixel.
     *
     * @return The built trajectory sequence.
     */
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
        else /*if (spikeMarkPosition == SpikeMark.RIGHT)*/
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeRight(30)
                    .build();
    }

    /**
     * Constructs and returns the trajectory sequence used to drive to the backdrop.
     *
     * @return The built trajectory sequence.
     */
    private TrajectorySequence driveToBackdrop() {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(46, 35 * positionMultiplier, Math.toRadians(0)))
                .strafeRight(spikeMarkPosition == SpikeMark.LEFT ? -6 : spikeMarkPosition == SpikeMark.MIDDLE ? 0.1 : 9)
                .build();
    }

    /**
     * Constructs and returns the trajectory sequence used to drive to the corner.
     *
     * @return The built trajectory sequence.
     */
    private TrajectorySequence driveToCorner() {
        // TODO: Check the math in this trajectory
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(3)
                .turn(Math.toRadians(isBlueAlliance ? -90 : 90))
                .lineTo(new Vector2d(49, 60 * positionMultiplier))
                .build();
    }

    private void updateStatus(String text) {
        telemetry.addData("Status", text);
        telemetry.update();
    }

    // -------------------------------------------------------------------------------------------

//    public void run() {
//        while (!pipeline.cameraReady) updateStatus("Waiting for camera");
//        gameElementPosition = pipeline.findGameElement(isBlueAlliance);
//        webcam.closeCameraDevice();
//
//        // Initial subsystem movements
//        MoveElbowCommand elbowCommand = new MoveElbowCommand(elbow, Constants.ElbowConstants.LOW_SCORING_POSITION);
//        SequentialCommandGroup autoStartCommand = new SequentialCommandGroup(
////                new MoveElbowCommand(elbow, Constants.ElbowConstants.INTAKE_POSITION),
//                new MoveSlideCommand(slide, Constants.LinearSlideConstants.IN_POSITION),
//                new InstantCommand(claw::closeClawSingle),
//                new WaitCommand(500),
//                new InstantCommand(intake::mediumPosition),
//                new WaitCommand(1000),
//                elbowCommand
//        );
//        autoStartCommand.schedule();
//        while (!elbowCommand.isFinished()) {
//            CommandScheduler.getInstance().run();
//            updateStatus("Running commands");
//        }
//
//
//        // Sets robot position in Road Runner
//        drive.setPoseEstimate(startPosition);
//        telemetry.addData("Start Position", startPosition.getX());
//        telemetry.update();
//
//        // Do push movement
//        updateStatus("Pushing pixel to: " + gameElementPosition + " position");
//        setPrePushMovement(gameElementPosition);
//        drive.followTrajectorySequence(pushMovementPreOuttake);
//
//        // Place purple pixel
//        WaitCommand finalCommand = new WaitCommand(1);
//        new SequentialCommandGroup(
//                new InstantCommand(intake::downPosition, intake),
//                new WaitCommand(250),
//                new InstantCommand(intake::intake, intake),
//                new WaitCommand(250),
//                new InstantCommand(intake::stop, intake),
//                new InstantCommand(intake::mediumPosition, intake),
//                finalCommand
//        ).schedule();
//        while (!finalCommand.isFinished()){
//            CommandScheduler.getInstance().run();
//        }
//
//        setPostPushMovement(gameElementPosition);
//        drive.followTrajectorySequence(pushMovementPostOuttake);
//
//        if (goToBackDrop) {
//            // Driving to backdrop
//            updateStatus("Driving to backdrop");
//            if (isBlueAlliance) { // Blue side movement
//                driveToBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .lineToSplineHeading(new Pose2d(46, 35, Math.toRadians(0)))
//                        .strafeRight(extraMovement)
//                        .build();
//            } else { // Red side movement
//                driveToBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .lineToSplineHeading(new Pose2d(46, -35, 0))
//                        .strafeRight(extraMovement)
//                        .build();
//            }
//            drive.followTrajectorySequence(driveToBackdrop);
//
//            // Scoring yellow pixel
//            updateStatus("Moving slide");
//            finalCommand = new WaitCommand(1);
//            SequentialCommandGroup placeYellowPixel = new SequentialCommandGroup(
//                    new InstantCommand(intake::downPosition),
//                    new MoveElbowCommand(elbow, Constants.ElbowConstants.LOW_SCORING_POSITION),
//                    new MoveSlideCommand(slide, Constants.LinearSlideConstants.LOW_SCORING_POSITION),
//                    new WaitCommand(750),
//                    new InstantCommand(claw::openClaw, claw),
//                    new WaitCommand(750),
//                    new MoveSlideCommand(slide, Constants.LinearSlideConstants.IN_POSITION),
//                    finalCommand
//            );
//            placeYellowPixel.schedule();
//            while (!finalCommand.isFinished()) {
//                CommandScheduler.getInstance().run();
//            }
//            if (isBlueAlliance) {
//                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .back(3)
//                        .turn(Math.toRadians(-90))
//                        .lineTo(new Vector2d(49, 60))
//                        .build());
//            } else {
//                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .back(3)
//                        .turn(Math.toRadians(90))
//                        .lineTo(new Vector2d(49, -60))
//                        .build());
//            }
//
//            updateStatus("Finished");
//        }
//    }

//    public void redLeft(){
//        startPosition = new Pose2d(-35.3, -62, Math.toRadians(90));
//        isBlueAlliance = false;
//        run();
//    }
//
//    public void blueRight(){
//        startPosition = new Pose2d(-35.3, 62, Math.toRadians(270));
//        isBlueAlliance = true;
//        run();
//    }
//
//    public void redRight(){
//        goToBackDrop = true;
//        startPosition = new Pose2d(11.5, -62, Math.toRadians(90));
//        isBlueAlliance = false;
//
////        // Building Movement to backdrop
////        driveToBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
////                .splineToSplineHeading(new Pose2d(34, -60, 0), 0)
////                .splineToConstantHeading(new Vector2d(35, -35), 0)
////                .build();
//
//        // Determine April Tag to line up with
//        if (gameElementPosition == -1){
//            aprilTagID = 4;
//        }
//        if (gameElementPosition == 0){
//            aprilTagID = 5;
//        }
//        if (gameElementPosition == 1){
//            aprilTagID = 6;
//        }
//        run();
//    }
//
//    public void blueLeft(){
//        goToBackDrop = true;
//        startPosition = new Pose2d(11.5, 62, Math.toRadians(270));
//        isBlueAlliance = true;
//
////        // Building Movement to backdrop
////        driveToBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
////                .splineToSplineHeading(new Pose2d(35, 59, 0), 0)
////                .splineToConstantHeading(new Vector2d(25, 35), 0)
////                .build();
//
//        // Determine April Tag to line up with
//        if (gameElementPosition == -1){
//            aprilTagID = 1;
//        }
//        if (gameElementPosition == 0){
//            aprilTagID = 2;
//        }
//        if (gameElementPosition == 1){
//            aprilTagID = 3;
//        }
//        run();
//    }

//    private void setPrePushMovement(int gameElementPosition){
//        if (gameElementPosition == -1){
//            pushMovementPreOuttake = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .forward(24)
//                    .turn(Math.toRadians(90))
//                    .strafeRight(5)
//                    .back(2)
//                    .build();
//            extraMovement = -6;
//        }
//        if (gameElementPosition == 0){
//            pushMovementPreOuttake = drive.trajectorySequenceBuilder(startPosition)
//                    .forward(25)
//                    .strafeLeft(8)
//                    .build();
//            extraMovement = 0.1;
//        }
//        if (gameElementPosition == 1){
//            pushMovementPreOuttake = drive.trajectorySequenceBuilder(startPosition)
//                    .forward(24)
//                    .turn(Math.toRadians(-90))
//                    .strafeLeft(5)
//                    .back(2)
//                    .build();
//            extraMovement = 9;
//        }
//    }

//    private void setPostPushMovement(int gameElementPosition){
//        if(gameElementPosition == -1) {
//            pushMovementPostOuttake = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .strafeLeft(30)
//                    .build();
//        } else if (gameElementPosition == 0) {
//            pushMovementPostOuttake = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .strafeRight(6)
//                    .back(11)
//                    .build();
//        } else if (gameElementPosition == 1) {
//            pushMovementPostOuttake = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .strafeRight(30)
//                    .build();
//        }
//    }

//    private void lineUpWithAprilTag(){
//        CameraSubsystem cameraSubsystem = new CameraSubsystem(hardwareMap);
//        updateStatus("Lining up with april tag ID: " + aprilTagID);
//
//        // Get to the correct distance away from the backdrop
//        do {
//            // Re-check distance to april tag
//            cameraSubsystem.detect();
//            aprilTagForwardDistance = cameraSubsystem.getDistance(aprilTagID);
//
//            telemetry.addData("Distance", aprilTagForwardDistance);
//
//            if (aprilTagForwardDistance != -1) {
//                // Move robot closer to april tag
//                if (aprilTagForwardDistance < aprilTagForwardTarget) {
//                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(aprilTagForwardTarget - aprilTagForwardDistance).build());
//                }
//                if (aprilTagForwardDistance > aprilTagForwardTarget) {
//                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(aprilTagForwardDistance - aprilTagForwardTarget).build());
//                }
//            }
//            telemetry.update();
//
//        } while (Math.abs(aprilTagForwardDistance - aprilTagForwardTarget) >= 0.25); // +-1/4" tolerance
//
////         Line up with april tag
//        do {
//            // Re-check lateral distance to april tag
//            aprilTagLateralDistance = cameraSubsystem.getLateralDistance(aprilTagID);
////
//            // Move robot closer to april tag
//            if (aprilTagLateralDistance < 0) {
//                telemetry.addData("Should be going left", aprilTagLateralDistance);
//                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(-aprilTagLateralDistance).build());
//            }
//            if (aprilTagLateralDistance > 0) {
//                telemetry.addData("Should be going right", aprilTagLateralDistance);
//                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(aprilTagLateralDistance).build());
//            }
//            telemetry.update();
//        } while (Math.abs(aprilTagLateralTarget - aprilTagLateralDistance) >= 1); // +-1" tolerance
//    }
}
