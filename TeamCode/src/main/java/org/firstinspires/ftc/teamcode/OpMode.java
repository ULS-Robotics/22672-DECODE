package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
/*
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
*/

@TeleOp(name="Teleop 2025", group="Linear OpMode")

public class OpMode extends LinearOpMode {
    /*
    private Gyroscope imu;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private Servo servoTest;
    */
    public DcMotor motorTest;
    public DcMotor motorFL, motorFR, motorBL, motorBR;


    @Override
    public void runOpMode() {
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        //motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        //servoTest = hardwareMap.get(Servo.class, "servoTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        while (opModeIsActive()) {
            //FL = y-x-T, FR = y+x-T, BL = y+x+T, BR = y-x-T

            double yInput = gamepad1.left_stick_y;
            double xInput = gamepad1.left_stick_x;
            double tInput = gamepad1.right_stick_x;

            double powerFL = yInput-xInput-tInput;
            double powerFR = yInput+xInput-tInput;
            double powerBL = yInput+xInput+tInput;
            double powerBR = yInput-xInput-tInput;

            motorFL.setPower(powerFL);
            motorFR.setPower(powerFR);
            motorBL.setPower(powerBL);
            motorBR.setPower(powerBR);

            telemetry.addData("FL Power", motorFL.getPower());
            telemetry.addData("FR Power", motorFR.getPower());
            telemetry.addData("BL Power", motorBL.getPower());
            telemetry.addData("BR Power", motorBR.getPower());
            telemetry.update();

            // Check if left bumper is currently held
            /*tgtPower = -this.gamepad1.left_trigger;

            motorTest.setPower(tgtPower);

            telemetry.addData("Motor Power", motorTest.getPower());
            telemetry.update();
             */
        }

    }
}