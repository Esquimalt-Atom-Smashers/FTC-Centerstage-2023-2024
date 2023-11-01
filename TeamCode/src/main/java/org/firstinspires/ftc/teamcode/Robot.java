package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.MoveElbowCommand;
import org.firstinspires.ftc.teamcode.commands.MoveSlideCommand;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WinchSubsystem;

public class Robot {
    // OpMode
    private final OpMode opMode;

    // Gamepads
    private final GamepadEx driverGamepad;
    private final GamepadEx operatorGamepad;

    //Define subsystems here.
    private final DriveSubsystem driveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElbowSubsystem elbowSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final LinearSlideSubsystem linearSlideSubsystem;
    private final CameraSubsystem cameraSubsystem;
    private final DroneSubsystem droneSubsystem;
    private final WinchSubsystem winchSubsystem;

    private boolean usingPIDControllers;

    enum DriveState {
        DRIVER_CONTROLLED,
        SNAPPING,
        DETECTING_TAG,
        CENTERING_TAG,
        STEPPING_LEFT,
        STEPPING_RIGHT
    }
    enum ScoringState {
        STARTING,
        INTAKE,
        LOADING_PIXELS,
        DRIVING,
        MANUAL
    }

    private DriveState driveState = DriveState.DRIVER_CONTROLLED;
    private ScoringState scoringState = ScoringState.STARTING;

    public Robot(OpMode opMode, boolean usingPID) {
        this.opMode = opMode;
        this.usingPIDControllers = usingPID;
        if (!usingPID) scoringState = ScoringState.MANUAL;

        // Initialize the gamepads
        driverGamepad = new GamepadEx(opMode.gamepad1);
        operatorGamepad = new GamepadEx(opMode.gamepad2);

        // Initialize the subsystems
        driveSubsystem = new DriveSubsystem(opMode.hardwareMap);
        intakeSubsystem = new IntakeSubsystem(opMode.hardwareMap);
        elbowSubsystem = new ElbowSubsystem(opMode.hardwareMap);
        clawSubsystem = new ClawSubsystem(opMode.hardwareMap);
        linearSlideSubsystem = new LinearSlideSubsystem(opMode.hardwareMap, this);
        cameraSubsystem = new CameraSubsystem(opMode.hardwareMap, opMode.telemetry);
        droneSubsystem = new DroneSubsystem(opMode.hardwareMap);
        winchSubsystem = new WinchSubsystem(opMode.hardwareMap);

        initCommands();
    }

    // Initialize the commands that control the robot
    private void initCommands() {
        // Default commands for individual subsystems:
        // CameraSubsystem

        // ClawSubsystem
        Trigger clawTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.Y) && scoringState == ScoringState.DRIVING);
        clawTrigger.whenActive(new InstantCommand(clawSubsystem::openClaw));
//        clawSubsystem.setDefaultCommand(new RunCommand(() -> {
//            if (operatorGamepad.getButton(GamepadKeys.Button.Y)) clawSubsystem.openClaw();
//        }, clawSubsystem));

        // DriveSubsystem
        driveSubsystem.setDefaultCommand(new RunCommand(() -> {
            driveSubsystem.drive(driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getRightX());
        }, driveSubsystem));

        Trigger autoDriveTrigger = new Trigger(() -> isPressed(driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        autoDriveTrigger.toggleWhenActive(new AutoDriveCommand(driveSubsystem, 1000), new AutoDriveCommand(driveSubsystem, -1000));

//        Trigger autoSnapTrigger = new Trigger(() -> isPressed(driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
//        autoSnapTrigger.whenActive(new FunctionalCommand(
//                // init actions
//                () -> {},
//                // execute actions
//                () -> driveSubsystem.autoSnap(opMode.telemetry),
//                // end actions
//                (b) -> {},
//                // is finished?
//                driveSubsystem::isFinishedSnapping,
//                driveSubsystem
//        ));



        // DroneSubsystem
        droneSubsystem.setDefaultCommand(new RunCommand(() -> {
            if (driverGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) droneSubsystem.release();
            if (driverGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) droneSubsystem.startPosition();
        }, droneSubsystem));

        // ElbowSubsystem
        elbowSubsystem.setDefaultCommand(new RunCommand(() -> {
            if (operatorGamepad.getLeftY() <= -0.1) elbowSubsystem.lowerManually();
            else if (operatorGamepad.getLeftY() >= 0.1) elbowSubsystem.raiseManually();
            else elbowSubsystem.stop();
        }, elbowSubsystem));

        // IntakeSubsystem

        // LinearSlideSubsystem
        linearSlideSubsystem.setDefaultCommand(new RunCommand(() -> {
            if (operatorGamepad.getRightY() >= 0.1) linearSlideSubsystem.retractManually();
            else if (operatorGamepad.getRightY() <= -0.1) linearSlideSubsystem.extendManually();
            else linearSlideSubsystem.stop();
        }, linearSlideSubsystem));

        // PixelSubsystem

        // WinchSubsystem
        winchSubsystem.setDefaultCommand(new RunCommand(() -> {
            if (driverGamepad.getButton(GamepadKeys.Button.A)) winchSubsystem.winch();
            else if (driverGamepad.getButton(GamepadKeys.Button.B)) winchSubsystem.unwinch();
            else winchSubsystem.stop();
        }, winchSubsystem));

        // Controls for command groups
        Trigger intakeTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.A));
        intakeTrigger.whenActive(new SequentialCommandGroup(
                new InstantCommand(() -> scoringState = ScoringState.INTAKE),
                new InstantCommand(intakeSubsystem::downPosition, intakeSubsystem),
                new MoveSlideCommand(linearSlideSubsystem, Constants.LinearSlideConstants.IN_POSITION),
                new MoveElbowCommand(elbowSubsystem, Constants.ElbowConstants.DRIVING_POSITION),
                new InstantCommand(intakeSubsystem::intake, intakeSubsystem),
                new InstantCommand(clawSubsystem::openClaw, clawSubsystem)
        ));

        Trigger cancelIntakeTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.B) && scoringState == ScoringState.INTAKE);
        cancelIntakeTrigger.whenActive(new SequentialCommandGroup(
           new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
           new InstantCommand(intakeSubsystem::mediumPosition, intakeSubsystem),
           new InstantCommand(() -> scoringState = ScoringState.DRIVING)
        ));

        // Press X when in intake mode to pick up pixels
        Trigger pickPixelsTrigger = new Trigger(() -> scoringState == ScoringState.INTAKE && operatorGamepad.getButton(GamepadKeys.Button.X));
        pickPixelsTrigger.whenActive(new SequentialCommandGroup(
                // Set the scoring state to loading pixels
                new InstantCommand(() -> scoringState = ScoringState.LOADING_PIXELS),
                // Move the arm down to pick up the pixels
                new MoveElbowCommand(elbowSubsystem, Constants.ElbowConstants.INTAKE_POSITION),
                // Close the claw and wait 500ms for the claw to finish gripping on the pixels
                new InstantCommand(clawSubsystem::closeClaw, clawSubsystem),
                new WaitCommand(500),
                // Stop the intake and move it up
                new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
                new InstantCommand(intakeSubsystem::mediumPosition, intakeSubsystem),
                // Move the elbow to level position
                new MoveElbowCommand(elbowSubsystem, Constants.ElbowConstants.LEVEL_POSITION),
                // Move the intake down and wait 250ms for it to get there
                new InstantCommand(intakeSubsystem::downPosition, intakeSubsystem),
                new WaitCommand(250),
                // Extend the arm a little bit
                new MoveSlideCommand(linearSlideSubsystem, Constants.LinearSlideConstants.TILT_POSITION),
                // Move the arm down and up to tilt the pixels in the grip
                new MoveElbowCommand(elbowSubsystem, Constants.ElbowConstants.TILT_POSITION),
                new MoveElbowCommand(elbowSubsystem, Constants.ElbowConstants.LEVEL_POSITION),
                // Retract the arm again
                new MoveSlideCommand(linearSlideSubsystem, Constants.ElbowConstants.INTAKE_POSITION),
                // Set the state to driving again
                new InstantCommand(() -> scoringState = ScoringState.DRIVING)
        ));

        // Press left dpad while in driving mode to move the arm to low preset scoring position
        Trigger lowScoringPositionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_LEFT) && scoringState == ScoringState.DRIVING);
        lowScoringPositionTrigger.whenActive(new SequentialCommandGroup(
                new MoveElbowCommand(elbowSubsystem, Constants.ElbowConstants.LOW_SCORING_POSITION),
                new MoveSlideCommand(linearSlideSubsystem, Constants.LinearSlideConstants.LOW_SCORING_POSITION)
        ));

        // Press up dpad while in driving mode to move the arm to medium scoring position
        Trigger mediumScoringPositionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_UP) && scoringState == ScoringState.DRIVING);
        mediumScoringPositionTrigger.whenActive(new SequentialCommandGroup(
                new MoveElbowCommand(elbowSubsystem, Constants.ElbowConstants.MEDIUM_SCORING_POSITION),
                new MoveSlideCommand(linearSlideSubsystem, Constants.LinearSlideConstants.MEDIUM_SCORING_POSITION)
        ));

        // Press right dpad while in driving mode to move the arm to high scoring position
        Trigger highScoringPositionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) && scoringState == ScoringState.DRIVING);
        highScoringPositionTrigger.whenActive(new SequentialCommandGroup(
                new MoveElbowCommand(elbowSubsystem, Constants.ElbowConstants.HIGH_SCORING_POSITION),
                new MoveSlideCommand(linearSlideSubsystem, Constants.LinearSlideConstants.HIGH_SCORING_POSITION)
        ));

        // Press down dpad while in driving mode to move the arm to home position (used while driving)
        Trigger homePositionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_DOWN) && scoringState == ScoringState.DRIVING);
        homePositionTrigger.whenActive(new SequentialCommandGroup(
                new MoveSlideCommand(linearSlideSubsystem, Constants.LinearSlideConstants.IN_POSITION),
                new MoveElbowCommand(elbowSubsystem, Constants.ElbowConstants.DRIVING_POSITION)
        ));

        Trigger leftInstructionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER) && !operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        Trigger centerInstructionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER) && operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        Trigger rightInstructionTrigger = new Trigger(() -> !operatorGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER) && operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER));
    }

    public Robot(OpMode opMode) {
        this(opMode, true);
    }

    // Perform actions that happen when the robot starts
    public void start() {
//        droneSubsystem.startPosition();
//        new SequentialCommandGroup(
//                new MoveSlideCommand(linearSlideSubsystem, Constants.LinearSlideConstants.IN_POSITION),
//                new MoveElbowCommand(elbowSubsystem, Constants.ElbowConstants.DRIVING_POSITION)
//        ).schedule();
    }

    // Main robot loop
    public void run() {
        // Run the command scheduler, which polls the gamepad inputs, and performs commands created in initCommands
        CommandScheduler.getInstance().run();


//        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
//            linearSlideSubsystem.lowScoringPosition();
//        }
//        else if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_UP)) {
//            linearSlideSubsystem.mediumScoringPosition();
//        }
//        else if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
//            linearSlideSubsystem.highScoringPosition();
//        }

//        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
//            linearSlideSubsystem.runPID();
//        }


//        driveSubsystem.printPosition(opMode.telemetry);
        elbowSubsystem.printPosition(opMode.telemetry);
        linearSlideSubsystem.printData(opMode.telemetry);
        opMode.telemetry.update();

        // Instruction controls (driver):
        // Left trigger adds a left align
        // Right trigger adds a right align
        // Both triggers add a center align
        // Left bumper adds a step left
        // Right bumper adds a step right
        // A enters the instructions
        // B cancels the instructions
//        if (isPressed(driverGamepad.left_trigger) && isPressed(driverGamepad.right_trigger))
//            instructionExecutor.addInstruction(this::alignCenter);
//        else if (isPressed(driverGamepad.left_trigger))
//            instructionExecutor.addInstruction(this::alignLeft);
//        else if (isPressed(driverGamepad.right_trigger))
//            instructionExecutor.addInstruction(this::alignRight);
//        if (driverGamepad.left_bumper) instructionExecutor.addInstruction(this::stepLeft);
//        if (driverGamepad.right_bumper) instructionExecutor.addInstruction(this::stepRight);
//        if (driverGamepad.a) instructionExecutor.executeInstructions();
//        if (driverGamepad.b) instructionExecutor.clearInstructions();
    }

    private void driveLoop() {
        switch (driveState) {
            case DRIVER_CONTROLLED:
                driveSubsystem.drive(driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getRightX());
                if (driverGamepad.getButton(GamepadKeys.Button.A)) {
//                    driveSubsystem.autoSnap();
//                    driveState = DriveState.SNAPPING;
                }
                break;
            case SNAPPING:
                if (driveSubsystem.isFinishedSnapping()) {
                    driveState = DriveState.DETECTING_TAG;
                }
                break;
            case DETECTING_TAG:
                break;
            case CENTERING_TAG:
                if (driveSubsystem.isCentered()) {
//                    instructionExecutor.getNextInstruction();
                }
                break;
            case STEPPING_LEFT:
                if (driveSubsystem.isFinishedSteppingLeft()) {
                    driveState = DriveState.DRIVER_CONTROLLED;
                }
                break;
            case STEPPING_RIGHT:
                if (driveSubsystem.isFinishedSteppingRight()) {
                    driveState = DriveState.DRIVER_CONTROLLED;
                }
                break;
            default:
                driveState = DriveState.DRIVER_CONTROLLED;
        }
    }

    public void scoringLoop() {
//        switch (scoringState) {
//            case STARTING:
//            case INTAKE:
//            case LOADED_DRIVING:
//                runPIDControllers();
//                break;
//            case MANUAL:
//                runManually();
//                break;
//
//        }
    }

    public void runManually() {

        driveLoop();

        // Elbow Subsystem
        if (operatorGamepad.getButton(GamepadKeys.Button.X)) elbowSubsystem.raiseManually();
        else if (operatorGamepad.getButton(GamepadKeys.Button.Y)) elbowSubsystem.lowerManually();
        else elbowSubsystem.stop();
        elbowSubsystem.printPosition(opMode.telemetry);
        opMode.telemetry.update();

        // Intake Subsystem
        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_UP)) intakeSubsystem.mediumPosition();
        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) intakeSubsystem.downPosition();
        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) intakeSubsystem.intake();
        else if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)) intakeSubsystem.outtake();
        else intakeSubsystem.stop();

        // Linear Slide Subsystem
        if (operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) linearSlideSubsystem.extendManually();
        else if (operatorGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) linearSlideSubsystem.retractManually();
        else linearSlideSubsystem.stop();

        // Claw Subsystem
        if (operatorGamepad.getButton(GamepadKeys.Button.A)) clawSubsystem.openClaw();
        if (operatorGamepad.getButton(GamepadKeys.Button.B)) clawSubsystem.closeClaw();

        // Drone subsystem
        if (driverGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) droneSubsystem.release();
        if (driverGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) droneSubsystem.startPosition();

        // Winch subsystem
        if (driverGamepad.getButton(GamepadKeys.Button.A)) winchSubsystem.winch();
        else if (driverGamepad.getButton(GamepadKeys.Button.B)) winchSubsystem.unwinch();
        else winchSubsystem.stop();
    }

    public void runPIDControllers() {
        if (!usingPIDControllers) return;
        elbowSubsystem.runPID();
        linearSlideSubsystem.runPID();
    }

    private void alignLeft() {

    }

    private void alignCenter() {

    }

    private void alignRight() {

    }

    private void stepLeft() {
        driveSubsystem.halfStepLeft();
        driveState = DriveState.STEPPING_LEFT;
    }

    private void stepRight() {
        driveSubsystem.halfStepRight();
        driveState = DriveState.STEPPING_RIGHT;
    }

    private boolean isPressed(double controllerInput) {
        return Math.abs(controllerInput) >= Constants.DriveConstants.DEADZONE;
    }

    private void wait(int ms) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() <= ms) {}
    }
}
