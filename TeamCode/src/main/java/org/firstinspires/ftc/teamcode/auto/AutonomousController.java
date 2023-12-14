package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.CommandManager;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * A controller that controls the drive base and arm during autonomous.
 */
public class AutonomousController {
    enum AutonomousState {
        PICKING_UP_PIXELS,
        MOVING_TO_SPIKE_MARKS,
        MOVING_TO_CORRECT_SPIKE_MARK,
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

    /** The hardware map of the robot */
    private final HardwareMap hardwareMap;
    /** The telemetry on the driver station */
    private final Telemetry telemetry;

    /** The collection of subsystems that make up the robot */
    private final Robot robot;
    /** Used to get commands used in auto */
    private final CommandManager commandManager;
    /** Used to build the trajectories used in auto */
    private final TrajectoryManager trajectoryManager;

    /** Starting position */
    private Pose2d startPosition;

    /** Drive base of the robot */
    private final SampleMecanumDrive drive;

    // Camera things
//    private OpenCVPipeline pipeline;
//    private OpenCvWebcam webcam;

    /** Whether we are on the blue alliance */
    private boolean isBlueAlliance;
    /** Whether we are upstage (closer to the backdrop) or downstage */
    private boolean isUpstage;
    /** Whether we are going to place a yellow pixel on the backdrop */
    private boolean isPlacingYellow;

    // Operators used to easily convert positions and angles between red and blue
    private float positionMultiplier;
    private float rotationalOffset;

    /** The current state of the controller */
    private AutonomousState currentState = AutonomousState.IDLE;

    /** Where the game element is and where we need to place the pixels */
    public SpikeMark spikeMarkPosition;

    /** The current command we are running */
    private WaitCommand currentCommand;

    /**
     * Creates a new AutonomousController, initializing the subsystems using the HardwareMap from the provided LinearOpMode.
     *
     * @param opMode The LinearOpMode that created the controller.
     * @param isBlueAlliance If we are on the blue alliance.
     * @param isUpstage If we are upstage (closer to the backdrop).
     * @param isPlacingYellow Whether we want to place the yellow pixel on the backdrop.
     */
    public AutonomousController(LinearOpMode opMode, boolean isBlueAlliance, boolean isUpstage, boolean isPlacingYellow) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        setSettings(isBlueAlliance, isUpstage, isPlacingYellow);

        drive = new SampleMecanumDrive(hardwareMap);
        robot = new Robot(opMode, true, true);
        commandManager = new CommandManager(robot);
        trajectoryManager = new TrajectoryManager(this);

//        startOpenCV();
    }

    /*
     * Creates a webcam and asynchronously opens an open cv pipeline. Waits until the camera is ready
     */
//    private void startOpenCV(){
//        updateStatus("Not Ready (Starting OpenCVPipeline)...");
//        pipeline = new OpenCVPipeline();
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, Constants.CameraConstants.CAMERA_NAME), cameraMonitorViewId);
//        webcam.setPipeline(pipeline);
//        webcam.setMillisecondsPermissionTimeout(5000);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
//            }
//            @Override
//            public void onError(int errorCode)
//            {
//            }
//        });
//        while (!pipeline.cameraReady) updateStatus("Not Ready (Waiting for camera)...\nDo not press stop!");
//        updateStatus("Ready");
//    }

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

        positionMultiplier = isBlueAlliance ? 1 : -1;
        rotationalOffset = isBlueAlliance ? 0 : 180;
        startPosition = new Pose2d(isUpstage ? 11.5 : -35.5, 62 * positionMultiplier, Math.toRadians(isBlue ? 270 : 90));
    }

    /** Starts the state machine by starting the first command and setting the state to picking up pixels. */
    public void start() {
        currentState = AutonomousState.PICKING_UP_PIXELS;
        drive.setPoseEstimate(startPosition);
        scheduleCommand(commandManager.getAutoSetupCommand());
    }

    /** Runs the state machine. Switches states if needed, runs commands, and updates the drive and telemetry. */
    public void run() {
        switch (currentState) {
            // Picking up the pixel from the ground
            case PICKING_UP_PIXELS:
                if (canContinue()) {
//                    int gameElementPosition = pipeline.findGameElement(isBlueAlliance);
//                    spikeMarkPosition = gameElementPosition == -1 ? SpikeMark.LEFT : gameElementPosition == 0 ? SpikeMark.MIDDLE : SpikeMark.RIGHT;
//                    webcam.closeCameraDevice();
                    drive.followTrajectorySequenceAsync(trajectoryManager.driveToSpikeMarksTrajectory());
                    currentState = AutonomousState.MOVING_TO_SPIKE_MARKS;
                }
                break;
            // Carrying the pixels, moving to the center of the spike marks
            case MOVING_TO_SPIKE_MARKS:
                if (canContinue()) {
//                    currentState = AutonomousState.PLACING_PURPLE;
//                    currentCommand = scheduleCommand(commandManager.getAutoPlacePurpleCommand());
                    // Find where the correct spike mark is
                    if (robot.getDistanceSensorSubsystem().isRightBlocked()) spikeMarkPosition = SpikeMark.RIGHT;
                    else if (robot.getDistanceSensorSubsystem().isLeftBlocked()) spikeMarkPosition = SpikeMark.LEFT;
                    else spikeMarkPosition = SpikeMark.MIDDLE;
                    drive.followTrajectorySequenceAsync(trajectoryManager.driveToCorrectSpikeMarkTrajectory());
                    currentState = AutonomousState.MOVING_TO_CORRECT_SPIKE_MARK;
                }
                break;
            case MOVING_TO_CORRECT_SPIKE_MARK:
                if (canContinue()) {
//                    drive.followTrajectorySequenceAsync(trajectoryManager.driveToCorrectSpikeMarkTrajectory());
                    scheduleCommand(commandManager.getAutoPlacePurpleCommand());
                    currentState = AutonomousState.PLACING_PURPLE;
                }
                break;
            // Placing the purple pixel on the spike mark using the intake
            case PLACING_PURPLE:
                if (canContinue()) {
                    if (isUpstage && isPlacingYellow) {
                        currentState = AutonomousState.MOVING_FROM_SPIKE_MARKS;
                        scheduleCommand(commandManager.getAutoMoveArmCommand());
                        drive.followTrajectorySequenceAsync(trajectoryManager.driveFromSpikeMarksTrajectory());
                    }
                    else {
                        currentState = AutonomousState.HIDING;
                        drive.followTrajectorySequenceAsync(trajectoryManager.finalTrajectory());
                    }
                }
                break;
            // Moving out of the way of the pixel and game element
            case MOVING_FROM_SPIKE_MARKS:
                if (canContinue()) {
                    currentState = AutonomousState.MOVING_TO_BACKDROP;
                    drive.followTrajectorySequenceAsync(trajectoryManager.driveToBackdropTrajectory());
                }
                break;
            // Moving to the correct spot to place the yellow pixel
            case MOVING_TO_BACKDROP:
                if (canContinue()) {
                    currentState = AutonomousState.PLACING_YELLOW;
                    scheduleCommand(commandManager.getAutoPlaceYellowCommand());
                }
                break;
            // Placing the yellow pixel on the backstage
            case PLACING_YELLOW:
                if (canContinue()) {
                    currentState = AutonomousState.HIDING;
                    drive.followTrajectorySequenceAsync(trajectoryManager.finalTrajectory());
                }
                break;
            // Moving to the corner to take up the least amount of room
            case HIDING:
                if (canContinue()) {
                    currentState = AutonomousState.IDLE;
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
     * Schedules a command to be run. Adds a 1 ms wait commands to the end of the provided command and sets current command to it.
     * (Calling isFinished on a SequentialCommandGroup doesn't seem to work properly)
     *
     * @param command The command to be scheduled.
     */
    private void scheduleCommand(Command command) {
        WaitCommand waitCommand = new WaitCommand(1);
        new SequentialCommandGroup(command, waitCommand).schedule();
        currentCommand = waitCommand;
    }

    /**
     * Checks our current command and trajectory to see if we are done moving.
     *
     * @return true if the current command has finished and the drive base isn't busy, false otherwise
     */
    private boolean canContinue() {
        return currentCommand.isFinished() && !drive.isBusy();
    }

    @Deprecated
    private void updateStatus(String text) {
        telemetry.addData("Status", text);
        telemetry.update();
    }

    public SampleMecanumDrive getDriveBase() {
        return drive;
    }

    public SpikeMark getSpikeMarkPosition() {
        return spikeMarkPosition;
    }

    public float getPositionMultiplier() {
        return positionMultiplier;
    }

    public boolean isBlueAlliance() {
        return isBlueAlliance;
    }

    public boolean isPlacingYellow() {
        return isPlacingYellow;
    }

    public boolean isUpstage() {
        return isUpstage;
    }

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
