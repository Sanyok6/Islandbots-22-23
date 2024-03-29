package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.ComputerVision;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Config
@Autonomous(group = "drive")
public class AUTO_MAIN_LEFT extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor lights = hardwareMap.dcMotor.get("Lights");
        lights.setPower(0.01);

        DistanceSensor YDist = hardwareMap.get(DistanceSensor.class, "YDist");

        ComputerVision computerVision = new ComputerVision(hardwareMap);
        int detection;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor LSmotor = hardwareMap.dcMotor.get("LSmotor");
        LSmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo clawServo = hardwareMap.servo.get("claw");

        boolean decreasing = false;
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> detections = computerVision.pipeline.getAprilTagDetections();

            if (detections.size() != 0) {
                detection = detections.get(0).id;
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

            telemetry.update();

            if (decreasing) {
                lights.setPower(lights.getPower()-0.001);
                if (lights.getPower() < 0.001) { decreasing = false; }
            } else {
                lights.setPower(lights.getPower()+0.001);
                if (lights.getPower() > 0.95) { decreasing = true; }
            }

            telemetry.addData("lights", lights.getPower());

        }

        lights.setPower(1);

        sleep(100);

        detection = computerVision.pipeline.getAprilTagDetections().size() == 0 ? 19 : computerVision.pipeline.getAprilTagDetections().get(0).id | 19;

        clawServo.setPosition(0.7);

        sleep(500);

        while (LSmotor.getCurrentPosition() > -50) {
            LSmotor.setPower(0.7);
        }
        LSmotor.setPower(0.1);

        Pose2d startingPose = new Pose2d(-35, -62, Math.toRadians(90));
        drive.setPoseEstimate(startingPose);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-34, -40), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-22, -37), Math.toRadians(90))
                .build();
        drive.followTrajectory(traj);

        while (LSmotor.getCurrentPosition() > -2000) {
            LSmotor.setPower(0.7);
        }
        LSmotor.setPower(0.1);

        lineUpToJunctionUsingCamera(computerVision);

        sleep(200);
        clawServo.setPosition(0.3);

        sleep(500);

        traj = drive.trajectoryBuilder(traj.end()).back(3).build();
        drive.followTrajectory(traj);

        clawServo.setPosition(0.4);

        while (LSmotor.getCurrentPosition() < -20) {
            LSmotor.setPower(-1);
        }
        LSmotor.setPower(0.1);

        if (detection == 17) {
            drive.followTrajectory(drive.trajectoryBuilder(traj.end()).strafeLeft(35).build());
        } else if (detection == 19) {
            drive.followTrajectory(drive.trajectoryBuilder(traj.end()).strafeLeft(10).build());
        } else if (detection == 12) {
            drive.followTrajectory(drive.trajectoryBuilder(traj.end()).strafeRight(10).build());
        }
    }

    public void lineUpToJunction() {
        DistanceSensor YDist = hardwareMap.get(DistanceSensor.class, "YDist");

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("LFmotor");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("LBmotor");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("RFmotor");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("RBmotor");

        double f = YDist.getDistance(DistanceUnit.CM);
        while (!(f > 16 && f < 17)) {

            f = YDist.getDistance(DistanceUnit.CM);

            double y=0;
            double x=0;
            if (f > 25) {
                y=0.07;
            } else {
                x = f > 16.5 ? 0.07 : -0.07;
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            double frontLeftPower = (y + x) / denominator;
            double backLeftPower = (y - x) / denominator;
            double frontRightPower = (y - x) / denominator;
            double backRightPower = (y + x) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            sleep(20);
        }
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        sleep(50);
    }

    void lineUpToJunctionUsingCamera(ComputerVision computerVision) {
        DistanceSensor YDist = hardwareMap.get(DistanceSensor.class, "YDist");

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("LFmotor");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("LBmotor");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("RFmotor");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("RBmotor");

        int location = computerVision.pipeline.getJunctionLocation();
        double distance = YDist.getDistance(DistanceUnit.CM);

        while (!(Math.abs(location-115) < 20 && distance < 40)) {
            telemetry.addData("x", location);
            telemetry.addData("y", distance);
            telemetry.update();

            double y=0;
            if (distance > 40) {
                y = 0.07;
            }

            double x = location > 115 ? 1 : -1;
            x *= Math.abs(location-115) > 30 ? 0.09 : 0.065;

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            double frontLeftPower = (y + x) / denominator;
            double backLeftPower = (y - x) / denominator;
            double frontRightPower = (y - x) / denominator;
            double backRightPower = (y + x) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            location = computerVision.pipeline.getJunctionLocation();
            distance = YDist.getDistance(DistanceUnit.CM);
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

}