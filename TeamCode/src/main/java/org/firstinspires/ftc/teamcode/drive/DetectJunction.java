package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;

public class DetectJunction {

    OpenCvCamera camera;
    public DetectJunctionPipeline pipeline;

    public DetectJunction(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new DetectJunctionPipeline();

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public static class DetectJunctionPipeline extends OpenCvPipeline {

        int location;

        public int getLocation() {
            return location;
        }

        @Override
        public Mat processFrame(Mat input) {

            Mat mat = new Mat();

            // convert to HSV
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            if (mat.empty()) {
                return input;
            }

            // resize
            Mat resized = new Mat();
            Imgproc.resize(mat, resized, new Size(200, 112));

            // blur
            Mat blurred = new Mat();
            Imgproc.blur(resized, blurred, new Size(9,9));

            // filter color
            Mat thresh = new Mat();
            Core.inRange(blurred, new Scalar(20, 70, 80), new Scalar(32, 255, 255), thresh);

            // find median x-coordinate of yellow pixels
            ArrayList<Integer> results = new ArrayList();

            for (int r=0; r<thresh.rows(); r+=3)
            {
                ArrayList<Integer> locations = new ArrayList();
                for (int c=0; c<thresh.cols(); c++)
                {
                    if (thresh.get(r, c)[0] == 255) {
                        locations.add(c);
                    }
                }

                if (locations.size() > 10) {
                    Collections.sort(locations);
                    int middle = locations.size() / 2;
                    middle = middle % 2 == 0 ? middle - 1 : middle;
                    results.add(locations.get(middle));
                }
            }


            // find average x-coordinate
            int sum = 0;

            for (int i = 0; i < results.size(); i++) {
                sum += (int) Math.floor(results.get(i));
            }
            if (results.size() > 0) {
                sum = (int) Math.floor(sum/results.size());
            }

            // draw line
            for (int i = 0; i < thresh.rows(); i++) {
                thresh.put(i, sum, 150);
            }

            location = sum;

            input.release();
            mat.release();
            resized.release();
            blurred.release();
            thresh.copyTo(input);
            thresh.release();

            return input;
        }


    }
}