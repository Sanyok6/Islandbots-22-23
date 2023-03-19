package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SignalSleeveColorDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class AUTO_PARK_RIGHT extends LinearOpMode {

    public static double FORWARD_DIST = 28;
    public static double STRAFE_DIST = 26;

    public volatile SignalSleeveColorDetection.SignalSleevePipeline.Colors color;

    @Override
    public void runOpMode() throws InterruptedException {
        SignalSleeveColorDetection vision = new SignalSleeveColorDetection(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor LSmotor = hardwareMap.dcMotor.get("LSmotor");
        Servo clawServo = hardwareMap.servo.get("claw");

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD_DIST)
                .build();

        Trajectory trajectoryBackward = drive.trajectoryBuilder(new Pose2d())
                .back(FORWARD_DIST)
                .build();

        Trajectory strafeRight = drive.trajectoryBuilder(trajectoryForward.end())
                .strafeRight(STRAFE_DIST)
                .build();

        Trajectory strafeLeft = drive.trajectoryBuilder(trajectoryForward.end())
                .strafeLeft(STRAFE_DIST)
                .build();

        waitForStart();

        sleep(1600);

        color = vision.pipeline.color;

        clawServo.setPosition(0.7);

        drive.followTrajectory(trajectoryForward);

        if (color == SignalSleeveColorDetection.SignalSleevePipeline.Colors.BLUE) {
            drive.followTrajectory(strafeRight);
        } else if (color == SignalSleeveColorDetection.SignalSleevePipeline.Colors.RED) {
        } else {
            drive.followTrajectory(strafeLeft);
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

