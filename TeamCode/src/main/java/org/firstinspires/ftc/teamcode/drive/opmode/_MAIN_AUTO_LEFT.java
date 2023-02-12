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

import org.firstinspires.ftc.teamcode.drive.ComputerVision.SignalSleevePipeline.Colors;

@Config
@Autonomous(group = "drive")
public class _MAIN_AUTO_LEFT extends LinearOpMode {

    public volatile Colors color;


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

        waitForStart();

        sleep(100);
        color = vision.pipeline.color;

        clawServo.setPosition(0.7);

        sleep(500);

        while (LSmotor.getCurrentPosition() > -2000) {
            LSmotor.setPower(0.7);
        }
        LSmotor.setPower(0.1);

        Pose2d startingPose = new Pose2d(-35, -62, Math.toRadians(90));
        drive.setPoseEstimate(startingPose);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-34, -40), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-23.4, -34), Math.toRadians(90))
                .build();
        drive.followTrajectory(traj);

        traj = drive.trajectoryBuilder(traj.end()).forward(1).build();
        drive.followTrajectory(traj);

        lineUpToJunction();

        sleep(200);
        clawServo.setPosition(0.3);

        sleep(500);

        traj = drive.trajectoryBuilder(traj.end()).back(4).build();
        drive.followTrajectory(traj);

        while (LSmotor.getCurrentPosition() < -500) {
            LSmotor.setPower(-0.7);
        }
        LSmotor.setPower(0.1);

        traj = drive.trajectoryBuilder(traj.end()).lineToLinearHeading(new Pose2d(-30, -35, Math.toRadians(180))).build();
        drive.followTrajectory(traj);

        traj = drive.trajectoryBuilder(traj.end())
                .splineToConstantHeading(new Vector2d(-34, -11), Math.toRadians(90))
                .build();
        drive.followTrajectory(traj);

        traj = drive.trajectoryBuilder(traj.end())
                .lineTo(new Vector2d(-60, -11))
                .build();
        drive.followTrajectory(traj);

        sleep(500);

        clawServo.setPosition(0.7);
        sleep(500);

        while (LSmotor.getCurrentPosition() > -3000) {
            LSmotor.setPower(0.7);
        }
        LSmotor.setPower(0.1);

        traj = drive.trajectoryBuilder(traj.end())
                .lineTo(new Vector2d(-32, -12))
                .build();
        drive.followTrajectory(traj);

        traj = drive.trajectoryBuilder(traj.end())
                .lineToLinearHeading(new Pose2d(-24, -14, Math.toRadians(90)))
                .build();
        drive.followTrajectory(traj);

        lineUpToJunction();
        clawServo.setPosition(0.3);
        sleep(500);

        traj = drive.trajectoryBuilder(traj.end())
                .back(2)
                .build();
        drive.followTrajectory(traj);

        while (LSmotor.getCurrentPosition() > -500) {
            LSmotor.setPower(-0.7);
        }
        LSmotor.setPower(0.1);

        traj = drive.trajectoryBuilder(traj.end())
                .splineToConstantHeading(new Vector2d(-34, -11), Math.toRadians(90))
                .build();
        drive.followTrajectory(traj);

        traj = drive.trajectoryBuilder(traj.end())
                .lineToLinearHeading(new Pose2d(-35, -35, Math.toRadians(90)))
                .build();
        drive.followTrajectory(traj);


        if (color == Colors.BLUE) {
            drive.followTrajectory(drive.trajectoryBuilder(traj.end()).strafeLeft(20).build());
        } else if (color == Colors.RED) {
        } else {
            drive.followTrajectory(drive.trajectoryBuilder(traj.end()).strafeRight(20).build());
        }


//        Trajectory traj = drive.trajectoryBuilder(startingPose)
//                .splineToConstantHeading(new Vector2d(-0, -46), Math.toRadians(90))
//                .build();
//        drive.followTrajectory(traj);


//        sleep(500);
//
//        sleep(2000);
//
//        color = vision.pipeline.color;
//
//        clawServo.setPosition(0.7);
//
//        drive.followTrajectory(trajectoryForward);
//        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).strafeRight(STRAFE_DIST*1.5).build());
//
//        sleep(1000);
//
//        LSmotor.setTargetPosition(3100);
//        LSmotor.setPower(0.9);
//
//        sleep(1500);
//
//        LSmotor.setPower(0.1);
//
//        boolean d = false;
//
//        double r = RightDist.getDistance(DistanceUnit.CM);
//        double l = LeftDist.getDistance(DistanceUnit.CM);
//        double f = YDist.getDistance(DistanceUnit.CM);
//
//
//        while (f > 25) {
//
//            r = RightDist.getDistance(DistanceUnit.CM);
//            l = LeftDist.getDistance(DistanceUnit.CM);
//            f = YDist.getDistance(DistanceUnit.CM);
//
//            double y = 0.04;
//
//            double x = r > l ? 0.05 : -0.05;
//            x = d ? 0 : x;
//
//            if (r < 32 || l < 32) {
//                d = true;
//            }
//
//            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
//            double frontLeftPower = (y + x) / denominator;
//            double backLeftPower = (y - x) / denominator;
//            double frontRightPower = (y - x) / denominator;
//            double backRightPower = (y + x) / denominator;
//
//            motorFrontLeft.setPower(frontLeftPower);
//            motorBackLeft.setPower(backLeftPower);
//            motorFrontRight.setPower(frontRightPower);
//            motorBackRight.setPower(backRightPower);
//
//            sleep(50);
//        }
//
//        sleep(100);
//
//        while (!(f > 11.5 && f < 14.5)) {
//
//            f = YDist.getDistance(DistanceUnit.CM);
//
//            double x = f > 13 ? 0.05 : -0.05;
//            double y = 0;
//
//            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
//            double frontLeftPower = (y + x) / denominator;
//            double backLeftPower = (y - x) / denominator;
//            double frontRightPower = (y - x) / denominator;
//            double backRightPower = (y + x) / denominator;
//
//            motorFrontLeft.setPower(frontLeftPower);
//            motorBackLeft.setPower(backLeftPower);
//            motorFrontRight.setPower(frontRightPower);
//            motorBackRight.setPower(backRightPower);
//
//            sleep(50);
//
//            telemetry.addData("d", f);
//            telemetry.update();
//
//        }
//
//        motorFrontLeft.setPower(0);
//        motorBackLeft.setPower(0);
//        motorFrontRight.setPower(0);
//        motorBackRight.setPower(0);
//
//        clawServo.setPosition(0.3);
//        sleep(300);
//        clawServo.setPosition(0.7);
//
//        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(10).build());
//
//        LSmotor.setPower(-0.4);
//
//        sleep(1500);
//
//        LSmotor.setPower(0);
//
//        if (color == Colors.BLUE) {
//            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).strafeLeft(STRAFE_DIST*2.5).build());
//        } else if (color == Colors.RED) {
//            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).strafeLeft(STRAFE_DIST*1.5).build());
//        } else {
//            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).strafeLeft(STRAFE_DIST*0.5).build());
//        }
//
//

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("d", YDist.getDistance(DistanceUnit.CM));
            telemetry.update();
        };
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
    }
}
