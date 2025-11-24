package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Teleop 2025", group="Linear OpMode")

public class OpMode extends LinearOpMode {
    public BNO055IMU imu;
    //public Servo servoTest;
    public DcMotor motorFL, motorFR, motorBL, motorBR;


    @Override
    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
        imu.initialize(parameters);
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        //servoTest = hardwareMap.get(Servo.class, "servoTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //FL = y-x-T, FR = y+x-T, BL = y+x+T, BR = y-x-T

            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double strafe = gamepad1.right_stick_x * 0.5;
            boolean leftHeld = gamepad1.dpad_right;
            boolean rightHeld = gamepad1.dpad_left;
            boolean upHeld = gamepad1.dpad_up;
            boolean downHeld = gamepad1.dpad_down;

            double powerFL, powerFR, powerBL, powerBR;

            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double adjustedLx = -drive * Math.sin(heading) + turn * Math.cos(heading);
            double adjustedLy = drive * Math.cos(heading) + turn * Math.sin(heading);

            if (leftHeld) {
                // STRAFE LEFT
                powerFL = -1;
                powerFR = 1;
                powerBL = 1;
                powerBR = -1;

            } else if (rightHeld) {
                // STRAFE RIGHT
                powerFL = 1;
                powerFR = -1;
                powerBL = -1;
                powerBR = 1;

            } else if (upHeld) {
                // NORMAL DRIVE
                powerFL = adjustedLy - adjustedLx + strafe;
                powerFR = adjustedLy + adjustedLx - strafe;
                powerBL = adjustedLy + adjustedLx + strafe;
                powerBR = adjustedLy - adjustedLx - strafe;
            } else {
                // not NORMAL DRIVE
                powerFL = Range.clip(drive - turn - strafe, -1, 1);
                powerFR = Range.clip(drive - turn + strafe, -1, 1);
                powerBL = Range.clip(drive + turn + strafe, -1, 1);
                powerBR = Range.clip(drive + turn - strafe, -1, 1);
            }

            /*if(gamepad1.y) {
                // move to 0 degrees.
                servoTest.setPosition(0);
            } else if (gamepad1.b) {
                // move to 90 degrees.
                servoTest.setPosition(0.5);
            } else if (gamepad1.a) {
                // move to 180 degrees.
                servoTest.setPosition(1);
            }*/

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