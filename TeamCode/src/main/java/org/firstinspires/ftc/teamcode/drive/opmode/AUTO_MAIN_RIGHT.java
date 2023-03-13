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

@Config
@Autonomous(group = "drive")
public class AUTO_MAIN_RIGHT extends LinearOpMode {

    public volatile ComputerVision.SignalSleevePipeline.Colors color;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor lights = hardwareMap.dcMotor.get("Lights");
        lights.setPower(1);

        DistanceSensor YDist = hardwareMap.get(DistanceSensor.class, "YDist");

        ComputerVision vision = new ComputerVision(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor LSmotor = hardwareMap.dcMotor.get("LSmotor");
        LSmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo clawServo = hardwareMap.servo.get("claw");

        waitForStart();

        sleep(100);
        color = vision.pipeline.color;

        clawServo.setPosition(0.7);

        sleep(500);

        while (LSmotor.getCurrentPosition() > -50) {
            LSmotor.setPower(0.7);
        }
        LSmotor.setPower(0.1);

        Pose2d startingPose = new Pose2d(35, -62, Math.toRadians(90));
        drive.setPoseEstimate(startingPose);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(35, -62, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(34, -40), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(20.5, -38), Math.toRadians(90))
                .build();
        drive.followTrajectory(traj);


        while (LSmotor.getCurrentPosition() > -2000) {
            LSmotor.setPower(0.7);
        }
        LSmotor.setPower(0.1);
//
//        traj = drive.trajectoryBuilder(traj.end()).forward(4).build();
//        drive.followTrajectory(traj);

        lineUpToJunction();

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

        telemetry.addData("c", color);
        telemetry.update();

        if (color == ComputerVision.SignalSleevePipeline.Colors.BLUE) {
            drive.followTrajectory(drive.trajectoryBuilder(traj.end()).strafeLeft(10).build());
        } else if (color == ComputerVision.SignalSleevePipeline.Colors.RED) {
            drive.followTrajectory(drive.trajectoryBuilder(traj.end()).strafeRight(10).build());
        } else if (color == ComputerVision.SignalSleevePipeline.Colors.GREEN) {
            drive.followTrajectory(drive.trajectoryBuilder(traj.end()).strafeRight(35).build());
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

}
