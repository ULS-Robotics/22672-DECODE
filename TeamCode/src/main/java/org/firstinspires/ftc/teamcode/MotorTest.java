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

@TeleOp(name="Motor Test", group="DECODE")
public class MotorTest extends LinearOpMode{

    @Override
    public void runOpMode() {
        DcMotor motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        Gamepad gmpd = gamepad1;

        telemetry.addData("Status", "Motor Test: OFF");
        telemetry.update();

        waitForStart();

        // ---------------------------
        // Main Loop
        // ---------------------------
        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            motorTest.setPower(y);

            telemetry.addData("Status", "Motor Test: ON");
            telemetry.addData("Motor Power", y);
            telemetry.update();

        }
    }
}
