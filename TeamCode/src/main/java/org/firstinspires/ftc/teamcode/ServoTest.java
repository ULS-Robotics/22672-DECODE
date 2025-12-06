package org.firstinspires.ftc.teamcode;

// ---------------------------
// Import Statements
// ---------------------------
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Servo Test", group="DECODE")
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo servoTest = hardwareMap.get(Servo.class, "servoTest");

        telemetry.addData("Status", "Servo Test: OFF");
        telemetry.update();

        waitForStart();

        double currentPos = 0.5;
        servoTest.setPosition(currentPos);

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                currentPos -= 0.05;
            }
            if (gamepad1.right_bumper) {
                currentPos += 0.05;
            }

            currentPos = Range.clip(currentPos, 0, 1);
            servoTest.setPosition(currentPos);

            telemetry.addData("Status", "Servo Test: ON");
            telemetry.addData("Servo Position", currentPos);
            telemetry.update();
        }
    }
}
