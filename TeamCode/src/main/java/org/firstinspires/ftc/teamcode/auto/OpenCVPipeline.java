package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import static org.firstinspires.ftc.teamcode.Constants.CameraConstants.*;

public class OpenCVPipeline extends OpenCvPipeline {
    int cameraWidth;
    int cameraHeight;
    Mat input;
    Telemetry telemetry;
    Mat ycrcbMat = new Mat();
    Mat binaryMat = new Mat();
    Mat maskedInputMat = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        this.input = input;
        return null;
    }

    public void passTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public int findGameElement(int color){ // 0: RED, 1: BLUE
        Scalar lower;
        Scalar upper = new Scalar(255, 255, 255);

        cameraWidth = input.width();
        cameraHeight = input.height();

        // Determine what color to look for
        lower = (color == 0) ? new Scalar(88, 190, 82) : new Scalar(3, 0, 166);

        // Add color mask
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(ycrcbMat, lower, upper, binaryMat);
        maskedInputMat.release();
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);

        // Find how much color is in each section
        Mat leftZone = maskedInputMat.submat(new Rect(0, 0 , cameraWidth / 2, cameraHeight));
        Scalar avgColor1 = Core.mean(leftZone);

        Mat rightZone = maskedInputMat.submat(new Rect(cameraWidth /2, 0 , cameraWidth / 2, cameraHeight));
        Scalar avgColor2 = Core.mean(rightZone);

        telemetry.addData("Middle value", avgColor1);
        telemetry.addData("Right value", avgColor2);
        telemetry.update();

        // Case: Middle Zone
        if (avgColor1.val[0] > DETECTION_THRESHOLD) return 0;

        // Case: Right Zone
        if (avgColor2.val[0] > DETECTION_THRESHOLD) return 1;

        // Case: Left Zone
        return -1;
    }
}
