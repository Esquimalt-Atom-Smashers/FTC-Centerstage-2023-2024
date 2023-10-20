package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
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

@Config
public class CameraSubsystem extends SubsystemBase {

    private final AprilTagProcessor.Builder aprilTagProcessorBuilder;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;

    Telemetry telemetry;



    public CameraSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Create a new Builder
        aprilTagProcessorBuilder = new AprilTagProcessor.Builder();



        aprilTagProcessorBuilder.setDrawTagID(true);
        aprilTagProcessorBuilder.setDrawTagOutline(true);
        aprilTagProcessorBuilder.setDrawAxes(true);
        aprilTagProcessorBuilder.setDrawCubeProjection(true);

        aprilTagProcessor = aprilTagProcessorBuilder.build();
//        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

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
                telemetry.addData("Name", aprilTagDetection.metadata.name);
                telemetry.addData("ID", aprilTagIdCode);
                telemetry.addData("Position", aprilTagDetection.center);
                double width = aprilTagDetection.corners[0].x - aprilTagDetection.corners[1].x;
                double height = aprilTagDetection.corners[2].y - aprilTagDetection.corners[0].y;
                telemetry.addData("Width", width);
                telemetry.addData("Height", height);
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
                double distance = 1600 / width;
                telemetry.addData("Distance (in)", distance);
            }
        }
    }
}
