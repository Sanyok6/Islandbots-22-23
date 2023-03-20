package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.ComputerVision;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class DetectJunctionAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        ComputerVision computerVision = new ComputerVision(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        lineUpToJunctionUsingCamera(computerVision);
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