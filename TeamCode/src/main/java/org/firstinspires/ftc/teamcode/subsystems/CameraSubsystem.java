package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import static org.firstinspires.ftc.teamcode.Constants.CameraConstants.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class CameraSubsystem extends SubsystemBase {

    private final AprilTagProcessor.Builder aprilTagProcessorBuilder;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private List<AprilTagDetection> detections;
    private int minExposure = 20;

    Telemetry telemetry;

    public CameraSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public CameraSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
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

//        visionPortal.resumeStreaming();
//        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//            exposureControl.setMode(ExposureControl.Mode.Manual);
//        }
//        exposureControl.setExposure((long) Math.min(5, minExposure), TimeUnit.MILLISECONDS);

        this.telemetry = telemetry;

    }

    public void detect() {
        detections = aprilTagProcessor.getDetections();
    }

    public void detectAndPrint() {
        detect();
        for (AprilTagDetection aprilTagDetection : detections) {
            if (aprilTagDetection.metadata != null) {
                int aprilTagIdCode = aprilTagDetection.id;
                telemetry.addData("Name", aprilTagDetection.metadata.name);
                telemetry.addData("ID", aprilTagIdCode);
                double width = aprilTagDetection.corners[0].x - aprilTagDetection.corners[1].x;
                double height = aprilTagDetection.corners[2].y - aprilTagDetection.corners[0].y;
                double lateralDistance = getLateralDistance(aprilTagDetection);
                telemetry.addData("Lateral Distance", lateralDistance);

            }
        }
    }

    // Return the number of inches the robot is laterally away from the specified april tag
    public double getLateralDistance(int tagID) {
        if (detections == null || detections.size() < 1) return -1;
        for (AprilTagDetection aprilTagDetection : detections) {
            if (aprilTagDetection.id == tagID) {
                return getLateralDistance(aprilTagDetection);
            }
        }
        return -1;
//        // If the tag we are looking for is the center one, we give up
//        if (tagID == 2 || tagID == 5) return -1;
//        double distanceBetweenTags = 6;
//        // If we can't find the tag we want to, we can look to the center tags to figure out which way we need to go
//        if (tagID >= 4) {
            // TODO: Figure out which 6 should be where
//            return getLateralDistance(5) + (tagID == 4 ? 6 : -6);
//        }
//        else {
//            return getLateralDistance(2) + (tagID == 1 ? 6 : -6);
//        }
    }

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


    public double getDistance(int tagID) {
        if (detections == null || detections.size() < 1) return -1;
        for (AprilTagDetection aprilTagDetection : detections) {
            if (aprilTagDetection.id == tagID) {
                return getDistance(aprilTagDetection);
            }
        }
        return -1;
    }

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
}
