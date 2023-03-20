package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ComputerVision;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous()
public class AUTO_THREE_CONE_LEFT extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ComputerVision computerVision = new ComputerVision(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> detections = computerVision.pipeline.getAprilTagDetections();

            if (detections.size() != 0) {
                int detection = detections.get(0).id;
                if (detection == 17) {
                    telemetry.addLine("Detected parking spot #1");
                } else if (detection == 19) {
                    telemetry.addLine("Detected parking spot #2");
                } else if (detection == 12) {
                    telemetry.addLine("Detected parking spot #3");
                }
            } else {
                telemetry.addLine("No tag visible!");
            }

            int junctionPosition = computerVision.pipeline.getJunctionLocation();

            telemetry.addData("junction pos", junctionPosition);

            telemetry.update();
        }

    }
}
