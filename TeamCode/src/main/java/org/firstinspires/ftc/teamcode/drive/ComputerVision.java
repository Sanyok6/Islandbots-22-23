package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class ComputerVision {

    private final OpenCvWebcam webcam;
    public SignalSleevePipeline pipeline;

    private static final int STREAM_WIDTH = 960;
    private static final int STREAM_HEIGHT = 720;

    public ComputerVision(HardwareMap hwMap, Telemetry telemetry)
    {
        WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SignalSleevePipeline();
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() { webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT); }

            @Override
            public void onError(int errorCode) { }
        });

        telemetry.addLine("Webcam Init Successful");
        telemetry.update();
    }


    public static class SignalSleevePipeline extends OpenCvPipeline {
        public SignalSleevePipeline() { }

        public enum Colors {RED, BLUE, GREEN}

        Point regionPointA = new Point(430, 250);
        Point regionPointB = new Point(450, 210);

        public volatile Colors color = Colors.RED;
        public volatile int r, g, b = 0;

        @Override
        public Mat processFrame(Mat input) {

            r = (int) Core.mean(input.submat(new Rect(regionPointA, regionPointB))).val[0];
            g = (int) Core.mean(input.submat(new Rect(regionPointA, regionPointB))).val[1];
            b = (int) Core.mean(input.submat(new Rect(regionPointA, regionPointB))).val[2];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    regionPointA, // First point which defines the rectangle
                    regionPointB, // Second point which defines the rectangle
                    new Scalar(0, 255, 0), // The color the rectangle is drawn in
                    1); // Thickness of the rectangle lines

            if (r > g && r > b) {
                color = Colors.RED;
            } else if (g > b) {
                color = Colors.GREEN;
            } else {
                color = Colors.BLUE;
            }

            return input;
        }
    }

//    public static class getFrame extends OpenCvPipeline {
//        @Override
//        public Mat processFrame(Mat input) {
//
//        }
//    }
}
