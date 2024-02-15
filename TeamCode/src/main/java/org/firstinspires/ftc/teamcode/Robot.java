package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.CommandManager;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WinchSubsystem;

/**
 * Represents all of the subsystems that make up the robot
 *
 * @author Esquimalt Atom Smashers
 */
public class Robot {
    /** The op mode that created the robot */
    private final OpMode opMode;

    /** The gamepad that controls driving */
    private final GamepadEx driverGamepad;
    /** The gamepad that controls the arm and intake */
    private final GamepadEx operatorGamepad;

    /** The box subsystem of the robot */
    private final BoxSubsystem boxSubsystem;
    /** The distance sensor subsystem of the robot */
    private final DistanceSensorSubsystem distanceSensorSubsystem;
    /** The drive base subsystem of the robot */
    private final DriveSubsystem driveSubsystem;
    /** The drone holder and launcher subsystem of the robot */
    private final DroneSubsystem droneSubsystem;
    /** The elbow subsystem of the robot */
    private final ElbowSubsystem elbowSubsystem;
    /** The intake subsystem of the robot */
    private final IntakeSubsystem intakeSubsystem;
    /** The linear slide subsystem of the robot */
    private final LinearSlideSubsystem linearSlideSubsystem;
    /** The winch subsystem of the robot */
    private final WinchSubsystem winchSubsystem;
    /** The LED subsystem of the robot */
    private final LEDSubsystem ledSubsystem;

    private final CommandManager commandManager;

    private final boolean manualMode;

    private RobotState state = RobotState.DRIVING;

    // This for debugging, will remove later
    public static GamepadEx gamepadEx;

    public enum RobotState {
        DRIVING,
        INTAKE,
        LOADING_PIXELS,
        SHOOTING_DRONE
    }

    /**
     * Initializes all gamepads, subsystems, and if necessary, initializes the commands through {@link #bindCommands()}.
     *
     * @param opMode The opMode that created the Robot
     * @param manualMode Whether we are using commands or not
     * @param resetEncoders Whether we reset the encoders on the slide, elbow and drive subsystems
     */
    public Robot(OpMode opMode, boolean manualMode, boolean resetEncoders) {
        this.opMode = opMode;
        this.manualMode = manualMode;

        // Initialize the gamepads
        driverGamepad = new GamepadEx(opMode.gamepad1);
        operatorGamepad = new GamepadEx(opMode.gamepad2);

        gamepadEx = driverGamepad;

        // Initialize the subsystems
        boxSubsystem = new BoxSubsystem(opMode.hardwareMap, opMode.telemetry);
        distanceSensorSubsystem = new DistanceSensorSubsystem(opMode.hardwareMap, opMode.telemetry);
        driveSubsystem = new DriveSubsystem(opMode.hardwareMap, opMode.telemetry);
        droneSubsystem = new DroneSubsystem(opMode.hardwareMap, opMode.telemetry);
        elbowSubsystem = new ElbowSubsystem(opMode.hardwareMap, opMode.telemetry);
        intakeSubsystem = new IntakeSubsystem(opMode.hardwareMap, opMode.telemetry);
        linearSlideSubsystem = new LinearSlideSubsystem(opMode.hardwareMap, opMode.telemetry);
        winchSubsystem = new WinchSubsystem(opMode.hardwareMap, opMode.telemetry);
        ledSubsystem = new LEDSubsystem(opMode.hardwareMap, opMode.telemetry);

        // Initialize the command manager
        commandManager = new CommandManager(this);

        // If we should, then bind the commands
        if (!manualMode) bindCommands();

        // If we should reset the encoders
        if (resetEncoders) resetEncoders();
    }

    /** Binds the ftclib commands that control the robot. */
    private void bindCommands() {
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

        // --- BoxReleaseSubsystem ---
        // Press operator X to open the box
        Trigger openBoxTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.X));
        openBoxTrigger.whenActive(commandManager.getOpenBoxCommand());

        // --- DriveSubsystem ---
        driveSubsystem.setDefaultCommand(commandManager.getDefaultDriveCommand());

        // Press driver BACK to reset gyro
        Trigger resetGyroTrigger = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.BACK));
        resetGyroTrigger.whenActive(commandManager.getResetGyroCommand());

        // Hold right bumper and press driver left dpad while in driving mode to snap to facing left side of field
        Trigger snapLeftTrigger = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) && isJoystickClose(-0.5, 0));
        snapLeftTrigger.whenActive(commandManager.getSnapLeftCommand());

        // Hold right bumper and press driver up dpad while in driving mode to snap to facing forward (far edge) side of field
        Trigger snapUpTrigger = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) && isJoystickClose(0, 0.5));
        snapUpTrigger.whenActive(commandManager.getSnapUpCommand());

        // Hold right bumper and press driver right dpad while in driving mode to snap to facing right side of field
        Trigger snapRightTrigger = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) && isJoystickClose(0.5, 0));
        snapRightTrigger.whenActive(commandManager.getSnapRightCommand());

        // Hold right bumper and press driver down dpad while in driving mode to snap to facing back (near edge) side of field
        Trigger snapDownTrigger = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) && isJoystickClose(0, -0.5));
        snapDownTrigger.whenActive(commandManager.getSnapDownCommand());

        // --- DroneSubsystem ---
        // Press operator right bumper to raise the arm and enter shooting drone mode
        Trigger droneLaunchModeTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        droneLaunchModeTrigger.whenActive(commandManager.getDroneModeCommand());

        // Press operator right trigger to launch the drone
        Trigger droneLaunchTrigger = new Trigger(() -> isPressed(operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) && state == RobotState.SHOOTING_DRONE);
        droneLaunchTrigger.whenActive(commandManager.getDroneLaunchCommand());

        // Press operator Y to exit shooting drone mode
        Trigger cancelDroneModeTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.Y) && state == RobotState.SHOOTING_DRONE);
        cancelDroneModeTrigger.whenActive(commandManager.getDroneCancelCommand());

        // --- ElbowSubsystem ---
        elbowSubsystem.setDefaultCommand(commandManager.getDefaultElbowCommand());

        // --- LinearSlideSubsystem ---
        linearSlideSubsystem.setDefaultCommand(commandManager.getDefaultSlideCommand());

        // --- WinchSubsystem ---
        winchSubsystem.setDefaultCommand(commandManager.getDefaultWinchCommand());

        // --- IntakeSubsystem ---
        // Press operator A to enter intake mode, when in intake mode, press operator A to pick up the pixels
        // We don't want to be in shooting drone mode, but anything else is fine
        Trigger intakeTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.A) && state != RobotState.SHOOTING_DRONE);
        intakeTrigger.toggleWhenActive(commandManager.getIntakeModeCommand(), commandManager.getPickupPixelsCommand());

        // Press operator B to toggle between outtaking to intaking
        Trigger outtakeTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.B) && state == RobotState.INTAKE);
        outtakeTrigger.toggleWhenActive(commandManager.getOuttakeCommand(), commandManager.getIntakeCommand());

        // --- Complex Commands (commands that use more than one subsystem) ---
        // Press operator left dpad while in driving mode to move the arm to low preset scoring position
        Trigger lowScoringPositionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_LEFT));
        lowScoringPositionTrigger.whenActive(commandManager.getLowScoringPositionCommand());

        // Press operator up dpad while in driving mode to move the arm to medium scoring position
        Trigger mediumScoringPositionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_UP));
        mediumScoringPositionTrigger.whenActive(commandManager.getMediumScoringPositionCommand());

        // Press operator right dpad while in driving mode to move the arm to high scoring position
        Trigger highScoringPositionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT));
        highScoringPositionTrigger.whenActive(commandManager.getHighScoringPositionCommand());

        // Press operator down dpad while in driving mode to move the arm to home position (used while driving)
        Trigger homePositionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_DOWN));
        homePositionTrigger.whenActive(commandManager.getHomePostionCommand());
    }

    /** Resets the encoders on the subsystems that use them. */
    public void resetEncoders() {
        linearSlideSubsystem.resetEncoder();
        elbowSubsystem.resetEncoder();
        driveSubsystem.resetEncoder();
    }

    /** Schedule any commands that run at the start of teleop mode. */
    public void start() {
//        commandManager.getSetupCommand().schedule();
    }

    /**
     * Run the robot, including calling CommandScheduler.getInstance().run(), which polls the gamepad inputs and performs scheduled commands.
     * Also prints data from the subsystems and updates the telemetry.
     */
    public void run() {
        // Run the command scheduler, which polls the gamepad inputs, and performs the commands created in bindCommands
        if (!manualMode)
            CommandScheduler.getInstance().run();

        printData();
        opMode.telemetry.update();
    }

    /** Perform things that happen at the start of manual. */
    public void startManual() {
        droneSubsystem.startPosition();
    }

    /** Controls the elbow, intake, slide, box, drone and drive subsystem manually, without any commands running or PID controllers. */
    public void runManually() {
        boolean usingFieldCentric = !isPressed(driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        double speedMultiplier = isPressed(driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) ? 0.3 : 1;
        driveSubsystem.drive(driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getRightX(), usingFieldCentric, speedMultiplier);

        if (driverGamepad.getButton(GamepadKeys.Button.BACK)) driveSubsystem.resetGyro();

        // Elbow Subsystem (operator)
        // Move left joystick up to move the arm up, down to move down
        if (isPressed(operatorGamepad.getLeftY())) elbowSubsystem.moveManually(operatorGamepad.getLeftY());
        else elbowSubsystem.stopMotor();

        // Linear Slide Subsystem (operator)
        // Move right joystick up to move slide out, down to move in
        if (isPressed(operatorGamepad.getRightY())) linearSlideSubsystem.moveManually(operatorGamepad.getRightY());
        else linearSlideSubsystem.stopMotor();

        // Intake Subsystem (operator)
        // Up -> Up, Right -> Medium, Down -> Down, left trigger -> intake, right trigger -> outtake
        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_UP)) intakeSubsystem.upPosition();
        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)) intakeSubsystem.mediumPosition();
        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) intakeSubsystem.downPosition();

        if (operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) intakeSubsystem.outtake();
        else if (isPressed(operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))) intakeSubsystem.intake();
        else intakeSubsystem.stopMotor();

        // Box release Subsystem (driver)
        // B -> Open/Red, A -> Close/Green
        if (operatorGamepad.getButton(GamepadKeys.Button.B) && !operatorGamepad.getButton(GamepadKeys.Button.START)) boxSubsystem.openBox();
        if (operatorGamepad.getButton(GamepadKeys.Button.A) && !operatorGamepad.getButton(GamepadKeys.Button.START)) boxSubsystem.closeBox();

        // Drone subsystem (operator)
        // X -> Release, Y -> Go to start position
        if (operatorGamepad.getButton(GamepadKeys.Button.X)) droneSubsystem.release();
        if (operatorGamepad.getButton(GamepadKeys.Button.Y)) droneSubsystem.startPosition();

        // Winch subsystem (operator)
        // left trigger -> Winch in, left bumper -> Winch out
        if (isPressed(operatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))) winchSubsystem.unwinch();
        else if (operatorGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) winchSubsystem.winch();
        else winchSubsystem.stopMotor();

        if (operatorGamepad.getButton(GamepadKeys.Button.BACK)) resetEncoders();

        if (operatorGamepad.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) boxSubsystem.disableLights();

        printData();
        opMode.telemetry.update();
    }

    public void printData() {
        opMode.telemetry.addData("Robot state", state);
        opMode.telemetry.addData("Gyro heading: ",  driveSubsystem.getHeading());

        elbowSubsystem.printData();
        linearSlideSubsystem.printData();
        distanceSensorSubsystem.printData();
        boxSubsystem.printData();
        intakeSubsystem.printData();
    }

    /**
     * Set the state of the robot.
     *
     * @param newState The new state of the robot
     */
    public void setState(RobotState newState) {
        state = newState;
    }

    /** @return The current state of the robot */
    public RobotState getState() {
        return state;
    }

    // TODO: Test this, need to normalize one of the angles
    public boolean isJoystickClose(double angle) {
        double currentAngle = Math.toDegrees(Math.atan2(driverGamepad.getRightY(), driverGamepad.getRightX()));
        return Math.abs(currentAngle - angle) <= 10;
    }

    /**
     * Checks if the right joystick on the driver gamepad is close enough to the specified position.
     *
     * @param x The target x-position of the joystick
     * @param y The target y-position of the joystick
     * @return If the joystick is close enough
     */
    public boolean isJoystickClose(double x, double y) {

        if (y == 0) {
            if (x < 0)
                return driverGamepad.getRightX() < x && Math.abs(driverGamepad.getRightY()) <= Constants.DriveConstants.DEADZONE;
            else
                return driverGamepad.getRightX() > x && Math.abs(driverGamepad.getRightY()) <= Constants.DriveConstants.DEADZONE;
        }
        else if (x == 0) {
            if (y < 0)
                return -driverGamepad.getRightY() < y && Math.abs(driverGamepad.getRightX()) <= Constants.DriveConstants.DEADZONE;
            else
                return -driverGamepad.getRightY() > y && Math.abs(driverGamepad.getRightX()) <= Constants.DriveConstants.DEADZONE;
        }
        return false;
    }

    public GamepadEx getOperatorGamepad() {
        return operatorGamepad;
    }

    public GamepadEx getDriverGamepad() {
        return driverGamepad;
    }

    public BoxSubsystem getBoxSubsystem() {
        return boxSubsystem;
    }

    public DistanceSensorSubsystem getDistanceSensorSubsystem() {
        return distanceSensorSubsystem;
    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    public DroneSubsystem getDroneSubsystem() {
        return droneSubsystem;
    }

    public ElbowSubsystem getElbowSubsystem() {
        return elbowSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    public LinearSlideSubsystem getLinearSlideSubsystem() {
        return linearSlideSubsystem;
    }

    public WinchSubsystem getWinchSubsystem() {
        return winchSubsystem;
    }

    public LEDSubsystem getLedSubsystem() {
        return ledSubsystem;
    }

    /**
     * Checks if a input from the controller is outside the dead zone
     *
     * @param controllerInput the input from the controller, for example gamepad.left_bumper or gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
     * @return whether the input from the controller has passed the dead zone
     */
    public boolean isPressed(double controllerInput) {
        return Math.abs(controllerInput) >= Constants.DriveConstants.DEADZONE;
    }
}