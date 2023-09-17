package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class WebcamSubsystem {
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private final Telemetry telemetry;

    private AprilTagDetection aprilTag;

    public WebcamSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "WebcamNam 1"), aprilTagProcessor);
        visionPortal.stopStreaming();

        this.telemetry = telemetry;
    }

    public void startStreaming() {
        visionPortal.resumeStreaming();
    }

    public void stopStreaming() {
        visionPortal.stopStreaming();
    }

    public boolean aprilTagFound() {
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection.metadata != null) {
                aprilTag = detection;
                return true;
            }
        }
        aprilTag = null;
        return false;
    }

    public AprilTagDetection getAprilTag() {
        return aprilTag;
    }
}
