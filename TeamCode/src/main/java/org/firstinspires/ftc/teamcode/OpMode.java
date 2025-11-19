package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
@TeleOp(name="Teleop 2025", group="Linear OpMode")

public class OpMode extends LinearOpMode {
    public CRServoImpl servoTest;
    public DcMotor motorFL, motorFR, motorBL, motorBR;


    @Override
    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        servoTest = hardwareMap.get(CRServoImpl.class, "servoTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        while (opModeIsActive()) {
            //FL = y-x-T, FR = y+x-T, BL = y+x+T, BR = y-x-T

            double yInput = -gamepad1.left_stick_y;
            double xInput = gamepad1.left_stick_x;
            double tInput = gamepad1.right_stick_x;
            boolean xHeld = gamepad1.dpad_right;
            boolean bHeld = gamepad1.dpad_left;

            double powerFL, powerFR, powerBL, powerBR;

            if (xHeld) {
                // STRAFE LEFT
                powerFL = -1;
                powerFR = 1;
                powerBL = 1;
                powerBR = -1;

            } else if (bHeld) {
                // STRAFE RIGHT
                powerFL = 1;
                powerFR = -1;
                powerBL = -1;
                powerBR = 1;

            } else {
                // NORMAL DRIVE
                powerFL = yInput - xInput + tInput;
                powerFR = yInput + xInput - tInput;
                powerBL = yInput + xInput + tInput;
                powerBR = yInput - xInput - tInput;
            }

            if(gamepad1.y) {
                // move to 0 degrees.
                servoTest.setPosition(0);
            } else if (gamepad1.x || gamepad1.b) {
                // move to 90 degrees.
                servoTest.setPosition(0.5);
            } else if (gamepad1.a) {
                // move to 180 degrees.
                servoTest.setPosition(1);
            }

            motorFL.setPower(powerFL);
            motorFR.setPower(powerFR);
            motorBL.setPower(powerBL);
            motorBR.setPower(powerBR);

            telemetry.addData("FL Power", motorFL.getPower());
            telemetry.addData("FR Power", motorFR.getPower());
            telemetry.addData("BL Power", motorBL.getPower());
            telemetry.addData("BR Power", motorBR.getPower());
            telemetry.update();
        }

    }
}