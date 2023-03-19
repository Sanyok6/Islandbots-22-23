package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DetectJunction;

import java.util.ArrayList;

@TeleOp
public class DetectJunctionAuto extends LinearOpMode {

    @Override
    public void runOpMode()
    {
        DetectJunction junctionDetection = new DetectJunction(hardwareMap);

        while (!isStarted()) {
            telemetry.addData("l", junctionDetection.pipeline.getLocation());
            telemetry.update();
        }


        waitForStart();

    }
}