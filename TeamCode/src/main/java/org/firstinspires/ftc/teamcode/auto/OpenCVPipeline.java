package org.firstinspires.ftc.teamcode.auto;

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
    int frameCount = 0;
    int currentPregameMask = -1;
    public boolean cameraReady = false;
    private Mat input = new Mat();
    private final Mat ycrcbMat = new Mat();
    private final Mat binaryMat = new Mat();
    private final Mat maskedInputMat = new Mat();
    private final Mat pregameMaskedInputMat = new Mat();
    Scalar lower;
    Scalar upper = UPPER;

    @Override
    public Mat processFrame(Mat input) {
        cameraReady = true;
        this.input = input;
        frameCount++;
        if (frameCount < 30){ addPreGameColorMask(currentPregameMask); }
        else if (currentPregameMask < 1){
            frameCount = 0;
            currentPregameMask++;
        } else {
            frameCount = 0;
            currentPregameMask = -1;
        }
        return pregameMaskedInputMat;
    }

    public int findGameElement(boolean isBlue){ // 0: RED, 1: BLUE
        cameraWidth = input.width();
        cameraHeight = input.height();

        // Add color mask
        addColorMask(isBlue ? 1 : 0);

        // Find how much color is in each half
        Mat leftZone = maskedInputMat.submat(new Rect(0, 0 , cameraWidth / 2, cameraHeight));
        Scalar avgColor1 = Core.mean(leftZone);

        Mat rightZone = maskedInputMat.submat(new Rect(cameraWidth / 2, 0 , cameraWidth / 2, cameraHeight));
        Scalar avgColor2 = Core.mean(rightZone);

        // Case: Middle Zone
        if (avgColor1.val[0] > DETECTION_THRESHOLD) return 0;

        // Case: Right Zone
        if (avgColor2.val[0] > DETECTION_THRESHOLD) return 1;

        // Case: Left Zone
        return -1;
    }

    private void addColorMask(int color){ // 0: RED, 1: BLUE
        lower = (color == 0) ? LOWER_RED : LOWER_BLUE;
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(ycrcbMat, lower, upper, binaryMat);
        maskedInputMat.release();
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);
    }

    private void addPreGameColorMask(int color){ // -1: No mask, 0: RED, 1: BLUE
        if (color == -1){
            lower = new Scalar(0, 0, 0);
        } else if (color == 0){
            lower = LOWER_RED;
        } else if (color == 1){
            lower = LOWER_BLUE;
        }
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(ycrcbMat, lower, upper, binaryMat);
        pregameMaskedInputMat.release();
        Core.bitwise_and(input, input, pregameMaskedInputMat, binaryMat);
    }
}
