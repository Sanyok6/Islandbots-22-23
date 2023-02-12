package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "default")
public class TELEOP extends LinearOpMode {

    double target = 0;
    boolean reached = true;

    private class liftSlide extends Thread {
        DcMotor LSmotor = hardwareMap.dcMotor.get("LSmotor");

        public liftSlide() {}

        @Override
        public void run()
        {
            while (!isInterrupted() && opModeIsActive())
            {
                if (reached) {
                    double ls = gamepad2.dpad_up ? 0.9 : (gamepad2.dpad_down ? -0.5 : 0.1);
                    if (LSmotor.getCurrentPosition() < -3000 && ls > 0) {ls = 0.1;}
                    LSmotor.setPower(ls);
                } else {
                    LSmotor.setPower(target > LSmotor.getCurrentPosition() ? -0.6 : 0.6);
                    reached = LSmotor.getCurrentPosition() > target-50 && LSmotor.getCurrentPosition() < target+50;
                    telemetry.addData("r", reached);
                    telemetry.addData("l", LSmotor.getCurrentPosition());
                    telemetry.update();
                }

                if (gamepad2.y) {
                    LSmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    LSmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                idle();
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("LFmotor");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("LBmotor");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("RFmotor");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("RBmotor");

//        DcMotor LSmotor = hardwareMap.dcMotor.get("LSmotor");

        Servo clawServo = hardwareMap.servo.get("claw");
        Servo flipServo = hardwareMap.servo.get("flip");

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor lights = hardwareMap.dcMotor.get("Lights");
        lights.setPower(1);

        waitForStart();

        if (isStopRequested()) return;

        Thread ls = new liftSlide();
        ls.start();

        sleep(1000);

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_x * -0.5;
            double x = -gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x * 0.6;
//            double ls = gamepad2.dpad_up ? 0.9 : (gamepad2.dpad_down ? -0.5 : 0.1);

//            if (ls > 0.5 && LSmotor.getCurrentPosition() < -3000) { ls = 0.1; }
//            else if (ls < 0.2 && LSmotor.getCurrentPosition() >= -125) { ls = 0.1; }

            telemetry.addData("y", gamepad1.left_stick_y);
            telemetry.addData("x", gamepad1.left_stick_x);
            telemetry.addData("rx", -gamepad1.right_stick_x);
            telemetry.update();

            //sensitivity adjustments
            y /= 1.6;
            x /= 1.6;
            rx /= 2;

            if (gamepad2.a) {
                target = 0;
                reached = false;
            }

            boolean clawOpen = !(gamepad2.left_trigger > 0);
            clawServo.setPosition(clawOpen ? 0.65 : 0.3);
            telemetry.addData("claw", clawOpen);

            boolean flipping = !(gamepad2.right_trigger > 0);
            flipServo.setPosition(flipping ? 0.3 : 0.7);
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

        }
    }
}
