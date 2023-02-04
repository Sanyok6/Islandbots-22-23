package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(group = "default")
public class TELEOP extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("LFmotor");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("LBmotor");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("RFmotor");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("RBmotor");

        DcMotor LSmotor = hardwareMap.dcMotor.get("LSmotor");

        Servo clawServo = hardwareMap.servo.get("claw");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor lights = hardwareMap.dcMotor.get("Lights");
        lights.setPower(1);

        boolean clawOpen = false;
        boolean loweringSlide = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_x * -0.5;
            double x = -gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x * 0.6;
            double ls = gamepad2.dpad_up ? 0.8 : (gamepad2.dpad_down ? -0.2 : 0.1);

            telemetry.addData("y", gamepad1.left_stick_y);
            telemetry.addData("x", gamepad1.left_stick_x);
            telemetry.addData("rx", -gamepad1.right_stick_x);
            telemetry.update();

            //sensitivity adjustments
            y /= 1.6;
            x /= 1.6;
            rx /= 2;

            if (gamepad2.a && !loweringSlide) {
                LSmotor.setTargetPosition(0);
                LSmotor.setPower(-0.4);
                loweringSlide = true;
            }

            if (LSmotor.getCurrentPosition() > -10 && loweringSlide) {
                LSmotor.setPower(0);
                loweringSlide = false;
            }


            clawOpen = !(gamepad2.left_trigger > 0);
            clawServo.setPosition(clawOpen ? 0.65 : 0.3);
            telemetry.addData("claw", clawOpen);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            if (!loweringSlide) {
                LSmotor.setPower(ls);
            }

            telemetry.addData("d", LSmotor.getCurrentPosition());
            telemetry.update();

        }
    }
}
