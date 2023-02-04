package org.firstinspires.ftc.teamcode.drive.opmode;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.ComputerVision;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.drive.ComputerVision.SignalSleevePipeline.Colors;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Config
@Autonomous(group = "drive")
public class RIGHTAUTANAMOUS extends LinearOpMode {

    public static double FORWARD_DIST = 44;
    public static double STRAFE_DIST = 60 *-1;  // CHANGE TO *-1 WHEN WE ARE ON THE RIGHT SIDE

    public volatile ComputerVision.SignalSleevePipeline.Colors color;


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("LFmotor");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("LBmotor");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("RFmotor");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("RBmotor");

        DcMotor lights = hardwareMap.dcMotor.get("Lights");
        lights.setPower(1);

        DistanceSensor RightDist = hardwareMap.get(DistanceSensor.class, "RightDist");
        DistanceSensor LeftDist = hardwareMap.get(DistanceSensor.class, "LeftDist");
        DistanceSensor YDist = hardwareMap.get(DistanceSensor.class, "YDist");

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

        sleep(500);

        sleep(2000);

        color = vision.pipeline.color;

        clawServo.setPosition(0.7);

        drive.followTrajectory(trajectoryForward);
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).strafeRight(STRAFE_DIST*1.5).build());

        sleep(1000);

        LSmotor.setTargetPosition(3100);
        LSmotor.setPower(0.9);

        sleep(1500);

        LSmotor.setPower(0.1);

        boolean d = false;

        double r = RightDist.getDistance(DistanceUnit.CM);
        double l = LeftDist.getDistance(DistanceUnit.CM);
        double f = YDist.getDistance(DistanceUnit.CM);


        while (f > 25) {

            r = RightDist.getDistance(DistanceUnit.CM);
            l = LeftDist.getDistance(DistanceUnit.CM);
            f = YDist.getDistance(DistanceUnit.CM);

            double y = 0.04;

            double x = r > l ? 0.05 : -0.05;
            x = d ? 0 : x;

            if (r < 32 || l < 32) {
                d = true;
                telemetry.addData("d", "d");
                telemetry.update();
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

            sleep(50);
        }

        sleep(100);

        while (!(f > 11.5 && f < 14.5)) {

            f = YDist.getDistance(DistanceUnit.CM);

            double x = f > 13 ? 0.05 : -0.05;
            double y = 0;

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            double frontLeftPower = (y + x) / denominator;
            double backLeftPower = (y - x) / denominator;
            double frontRightPower = (y - x) / denominator;
            double backRightPower = (y + x) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            sleep(50);

            telemetry.addData("d", f);
            telemetry.update();

        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);

        clawServo.setPosition(0.3);
        sleep(300);
        clawServo.setPosition(0.7);

        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(10).build());

        LSmotor.setPower(-0.4);

        sleep(1500);

        LSmotor.setPower(0);

        if (color == ComputerVision.SignalSleevePipeline.Colors.BLUE) {
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).strafeLeft(STRAFE_DIST*2.5).build());
        } else if (color == ComputerVision.SignalSleevePipeline.Colors.RED) {
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).strafeLeft(STRAFE_DIST*1.5).build());
        } else {
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).strafeLeft(STRAFE_DIST*0.5).build());
        }


        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("RightDist", RightDist.getDistance(DistanceUnit.CM));
            telemetry.addData("LeftDist", LeftDist.getDistance(DistanceUnit.CM));
            telemetry.update();
        };

    }


}
