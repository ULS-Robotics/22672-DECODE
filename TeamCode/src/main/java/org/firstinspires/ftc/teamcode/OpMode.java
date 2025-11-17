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

    @Override
    public void runOpMode() {
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
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

            // Check if left bumper is currently held
            boolean lBIsPressed = gamepad1.left_bumper;

            if (lBIsPressed) {
                // Run motor only while holding LB
                tgtPower = -1;
                motorTest.setPower(tgtPower);
            } else {
                // Stop motor when LB is released
                motorTest.setPower(0);
            }

            telemetry.addData("Left Bumper Held?", lBIsPressed);
            telemetry.addData("Motor Power", motorTest.getPower());
            telemetry.update();
        }

    }
}