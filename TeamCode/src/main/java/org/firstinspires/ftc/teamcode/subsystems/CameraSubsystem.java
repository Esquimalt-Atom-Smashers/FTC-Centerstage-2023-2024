package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import static org.firstinspires.ftc.teamcode.Constants.CameraConstants.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * A subsystem that represents the camera of the robot. Uses an april tag processor to detect april tags.
 *
 * @author Esquimalt Atom Smashers
 */
public class CameraSubsystem extends CustomSubsystemBase {

    private final AprilTagProcessor.Builder aprilTagProcessorBuilder;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private List<AprilTagDetection> detections;
//    private int minExposure = 20;
//    private ExposureControl exposureControl;


    /**
     * Constructs a new CameraSubsystem.
     *
     * @param hardwareMap The hardware map of the robot
     * @param telemetry The telemetry of the robot
     */
    public CameraSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        // Create a new Builder
        aprilTagProcessorBuilder = new AprilTagProcessor.Builder();

        aprilTagProcessorBuilder.setDrawTagID(true);
        aprilTagProcessorBuilder.setDrawTagOutline(true);
        aprilTagProcessorBuilder.setDrawAxes(false);
        aprilTagProcessorBuilder.setDrawCubeProjection(false);

        aprilTagProcessor = aprilTagProcessorBuilder.build();
//        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        // Create a Vision Portal with the default settings
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, CAMERA_NAME), aprilTagProcessor);
    }

    /** Get the april tag detections from the camera. */
    public void detectAprilTags() {
        detections = aprilTagProcessor.getDetections();
    }

    /** First detect any april tags and save them, then iterate over them and print out information about each one. */
    public void detectAndPrintAprilTags() {
        detectAprilTags();
        for (AprilTagDetection aprilTagDetection : detections) {
            if (aprilTagDetection.metadata != null) {
                int aprilTagIdCode = aprilTagDetection.id;
                telemetry.addData("Name", aprilTagDetection.metadata.name);
                telemetry.addData("ID", aprilTagIdCode);
                double distance = getDistance(aprilTagDetection);
                telemetry.addData("Distance", distance);
                double lateralDistance = getLateralDistance(aprilTagDetection);
                telemetry.addData("Lateral Distance", lateralDistance);
            }
        }
    }

    /**
     * Gets the distance the robot is from the april tag with the specified ID.
     *
     * @param tagID The ID of the tag we are looking for
     * @return The distance in inches that the robot is from the april tag, or -1 if the camera can't find the april tag
     */
    public double getDistance(int tagID) {
        if (detections == null || detections.size() < 1) return -1;
        for (AprilTagDetection aprilTagDetection : detections) {
            if (aprilTagDetection.id == tagID) {
                return getDistance(aprilTagDetection);
            }
        }
        return -1;
    }

    /**
     * Gets the distance the robot is from the specified april tag.
     *
     * @param aprilTag The april tag we are looking at
     * @return The distance in inches that the robot is from the april tag
     */
    public double getDistance(AprilTagDetection aprilTag) {
        double width = aprilTag.corners[0].x - aprilTag.corners[1].x;
         /*
        I found this formula by moving the robot specific distances away from the board
        and recording what the width was:
        d   h   w
        9   154 160
        12  126 126
        15  94  100
        18  79  84
        21  69  73
        27  56  60
        34  44  47
        42  36  38
        57  26  28

        Graphed it on Desmos:
        https://www.desmos.com/calculator/ioaxa6b1so
         */
        return 1600 / width;
    }

    /**
     * Gets the number of inches the robot is laterally away from the april tag with the specified ID.
     *
     * @param tagID The ID of the tag we are looking for
     * @return The lateral distance from the robot to the tag, or -1 if the camera can't find the april tag
     */
    public double getLateralDistance(int tagID) {
        if (detections == null || detections.size() < 1) return -1;
        for (AprilTagDetection aprilTagDetection : detections) {
            if (aprilTagDetection.id == tagID) {
                return getLateralDistance(aprilTagDetection);
            }
        }
        return -1;
    }

    /**
     * Gets the number of inches the robot is laterally away from the specified april tag.
     *
     * @param aprilTag The april tag we are looking at
     * @return The lateral distance from the robot to the tag
     */
    public double getLateralDistance(AprilTagDetection aprilTag) {
        // TODO: Fix this to account for distance from the board
        /*
        This formula was found by putting the robot at specific lateral distances
        away from the tag it was sensing and reading the center point.
        These values were found quite close to the board
        d   c
        0   539
        -2  450
        -4  350
        -6  265
        -8  163
        -9  113

        Graph on Desmos:
        https://www.desmos.com/calculator/ysofiovqff
         */
        return -(aprilTag.center.x - 540) / 45;
    }
}
