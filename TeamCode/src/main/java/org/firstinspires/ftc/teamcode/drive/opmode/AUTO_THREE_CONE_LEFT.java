package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SignalSleeveDetection;

@Autonomous(group = "drive")
public class AUTO_THREE_CONE_LEFT extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor lights = hardwareMap.dcMotor.get("Lights");
        lights.setPower(0.01);

        DistanceSensor YDist = hardwareMap.get(DistanceSensor.class, "YDist");

        SignalSleeveDetection signalSleeveDetection = new SignalSleeveDetection(hardwareMap);
        int detection;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor LSmotor = hardwareMap.dcMotor.get("LSmotor");
        LSmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo clawServo = hardwareMap.servo.get("claw");
    }
}
