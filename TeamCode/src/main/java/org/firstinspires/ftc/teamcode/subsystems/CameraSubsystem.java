package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import static org.firstinspires.ftc.teamcode.Constants.CameraConstants.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CameraSubsystem extends SubsystemBase {

//    private final AprilTagProcessor.Builder aprilTagProcessorBuilder;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;

    Telemetry telemetry;

    private AprilTagLibrary tagLibrary;

    public CameraSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Create a new Builder
//        aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
//
//
//        aprilTagProcessorBuilder.setTagLibrary(tagLibrary);
//
//        aprilTagProcessorBuilder.setDrawTagID(true);
//        aprilTagProcessorBuilder.setDrawTagOutline(true);
//        aprilTagProcessorBuilder.setDrawAxes(true);
//        aprilTagProcessorBuilder.setDrawCubeProjection(true);
//
//        aprilTagProcessor = aprilTagProcessorBuilder.build();
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // Create a Vision Portal with the default settings
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, CAMERA_NAME), aprilTagProcessor);

        this.telemetry = telemetry;

    }

    public void detect() {
        List<AprilTagDetection> aprilTagDetections;
        int aprilTagIdCode;

        aprilTagDetections = aprilTagProcessor.getDetections();
        telemetry.addData("Detections", aprilTagDetections.size());
        for (AprilTagDetection aprilTagDetection : aprilTagDetections) {

            if (aprilTagDetection.metadata != null) {
                aprilTagIdCode = aprilTagDetection.id;
                telemetry.addLine("April Tag Detected");
                telemetry.addData("ID", aprilTagIdCode);
                telemetry.addData("Name", aprilTagDetection.metadata.name);
                telemetry.addData("Position", aprilTagDetection.center);
                telemetry.addData("Corners", aprilTagDetection.corners);
            }
        }
    }

    
}
