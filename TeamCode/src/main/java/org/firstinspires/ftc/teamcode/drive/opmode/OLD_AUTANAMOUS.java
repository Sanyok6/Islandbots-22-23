package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.ComputerVision;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class OLD_AUTANAMOUS extends LinearOpMode {

        public static double FORWARD_DIST = 37;
        public static double STRAFE_DIST = 60;

        public volatile ComputerVision.SignalSleevePipeline.Colors color;

        @Override
        public void runOpMode() throws InterruptedException {
            ComputerVision vision = new ComputerVision(hardwareMap, telemetry);

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            DcMotor LSmotor = hardwareMap.dcMotor.get("LSmotor");
            Servo clawServo = hardwareMap.servo.get("claw");

            Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                    .forward(FORWARD_DIST)
                    .build();

            Trajectory trajectoryBackward = drive.trajectoryBuilder(new Pose2d())
                    .back(FORWARD_DIST)
                    .build();

            Trajectory strafeRight = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(STRAFE_DIST)
                    .build();

            Trajectory strafeLeft = drive.trajectoryBuilder(new Pose2d())
                    .strafeLeft(STRAFE_DIST)
                    .build();

            waitForStart();

            sleep(1600);

            color = vision.pipeline.color;

            clawServo.setPosition(0.7);

            drive.followTrajectory(trajectoryForward);

            if (color == ComputerVision.SignalSleevePipeline.Colors.BLUE) {
                drive.followTrajectory(strafeLeft);
            } else if (color == ComputerVision.SignalSleevePipeline.Colors.RED) {
            } else {
                drive.followTrajectory(strafeRight);
            }

            while (!isStopRequested() && opModeIsActive()) {
                telemetry.addData("color", vision.pipeline.color);
                telemetry.addData("r", vision.pipeline.r);
                telemetry.addData("g", vision.pipeline.g);
                telemetry.addData("b", vision.pipeline.b);
                telemetry.update();
            };

        }
    }

