package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ComputerVision;
import org.openftc.apriltag.AprilTagDetection;


import java.util.ArrayList;


@TeleOp
public class AprilTagAuto extends LinearOpMode {

    @Override
    public void runOpMode()
    {
        ComputerVision computerVision = new ComputerVision(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> detections = computerVision.pipeline.getAprilTagDetections();

            if (detections.size() != 0) {
                AprilTagDetection detection = detections.get(0);
                if (detection.id == 17) {
                    telemetry.addLine("Detected parking spot #1");
                } else if (detection.id == 19) {
                    telemetry.addLine("Detected parking spot #2");
                } else if (detection.id == 12) {
                    telemetry.addLine("Detected parking spot #3");
                }
            } else {
                telemetry.addLine("No tag visible!");
            }

            telemetry.update();
        }
    }
}