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

// ---------------------------
// Teleop Code
// ---------------------------
@TeleOp(name="Teleop", group="DECODE")
public class OpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        // ---------------------------
        // Hardware Mapping
        // ---------------------------
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        DcMotor shooterL = hardwareMap.get(DcMotor.class, "shooterL");
        DcMotor shooterR = hardwareMap.get(DcMotor.class, "shooterR");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        //DcMotor motorTest = hardwareMap.get(DcMotor.class, "motorTest");

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        shooterL.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        // ---------------------------
        // Servo Mapping (Dual-Mode Torque Servo)
        // ---------------------------
        Servo servoTest = hardwareMap.get(Servo.class, "servoTest");
        servoTest.setPosition(0.0);  // start at 0 for testing
        servoTest.setDirection(Servo.Direction.FORWARD);

        // ---------------------------
        // IMU Initialization
        // ---------------------------
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        imu.resetYaw();
        // ---------------------------
        // Main Loop
        // ---------------------------
        while (opModeIsActive()) {
            // ---------------------------
            // Gamepad Input
            // ---------------------------
            Gamepad gmpdA = gamepad1;
            Gamepad gmpdB = gamepad2;
            double y = -gmpdA.left_stick_y;  // Forward/back
            double x = gmpdA.left_stick_x;   // Strafe
            double rotation = gmpdA.right_stick_x * 0.5;  // Turn

            boolean dpadLeft  = gmpdA.dpad_left;
            boolean dpadRight = gmpdA.dpad_right;
            boolean dpadUp    = gmpdA.dpad_up;
            boolean dpadDown  = gmpdA.dpad_down;

            // Reset IMU (A button)
            if (gmpdA.a)
                imu.resetYaw();

            // ---------------------------
            // Motor Testing! Comment out if not testing
            // ---------------------------
            //double testPower = gmpdB.right_stick_y;
            //motorTest.setPower(testPower);

            // ---------------------------
            // Servo TEST (note from Pan: ok so it doesn't error, but servo doesn't move? further testing will be done tomorrow)
            // ---------------------------
            double currentPos = servoTest.getPosition();

            if (gmpdA.left_bumper) {
                currentPos -= 0.05; // move toward 0
            }
            if (gmpdA.right_bumper) {
                currentPos += 0.05; // move toward 1
            }

            servoTest.setPosition(Range.clip(currentPos, 0, 1));

            // ---------------------------
            // Intake and Shooter
            // ---------------------------
            boolean moving = false;
            boolean gmpdBX = gmpdB.x;
            boolean gmpdBB = gmpdB.b;
            if (gmpdBB) {
                gmpdBX = false;
                moving = true;
                intake.setPower(-1);
            } else if (gmpdBX) {
                moving = true;
                intake.setPower(1);
            } else {
                moving = true;
                intake.setPower(0);
                moving = false;
            }

            double power = 0;
            float shoot = gmpdB.right_trigger;
            if (shoot > 0) {
                power = -1;
            }
            if (shoot == 0) {
                power = 0;
            }
            shooterR.setPower(power);
            shooterL.setPower(power);

            // ---------------------------
            // Field-Centric Transformation
            // ---------------------------
            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            //WATCH
            double rotX = x * Math.cos(heading) - y * Math.sin(heading);
            double rotY = x * Math.sin(heading) + y * Math.cos(heading);


            // ---------------------------
            // Mecanum Mixing
            // ---------------------------
            double powerFL = rotY + rotX + rotation;
            double powerFR = rotY - rotX - rotation;
            double powerBL = rotY - rotX + rotation;
            double powerBR = rotY + rotX - rotation;
//            powerFL = Range.clip(y - x - rotation, -1, 1);
//            powerFR = Range.clip(y - x + rotation, -1, 1);
//            powerBL = Range.clip(y + x + rotation, -1, 1);
//            powerBR = Range.clip(y + x - rotation, -1, 1);

            // ---------------------------
            // DPAD Strafing Overrides
            // ---------------------------
            if (dpadLeft) {
                powerFL = -1; powerFR = 1;
                powerBL =  1; powerBR = -1;
            } else if (dpadRight) {
                powerFL =  1; powerFR = -1;
                powerBL = -1; powerBR =  1;
            } else if (dpadUp) {
                powerFL = 1; powerFR = 1;
                powerBL = 1; powerBR = 1;
            } else if (dpadDown) {
                powerFL = -1; powerFR = -1;
                powerBL = -1; powerBR = -1;
            }

            // ---------------------------
            // Normalize Motor Power
            // ---------------------------
            double max = Math.max(1.0, Math.max(Math.abs(powerFL),
                    Math.max(Math.abs(powerFR),
                            Math.max(Math.abs(powerBL), Math.abs(powerBR)))));

            powerFL /= max;
            powerFR /= max;
            powerBL /= max;
            powerBR /= max;

            // ---------------------------
            // Apply Motor Power
            // ---------------------------
            motorFL.setPower(powerFL);
            motorFR.setPower(powerFR);
            motorBL.setPower(powerBL);
            motorBR.setPower(powerBR);

            // ---------------------------
            // Telemetry
            // ---------------------------
            telemetry.addData("IMU", heading);
            telemetry.addData("Servo Position", servoTest.getPosition());
            telemetry.addData("Intake Status", moving);
            telemetry.addData("FL", powerFL);
            telemetry.addData("FR", powerFR);
            telemetry.addData("BL", powerBL);
            telemetry.addData("BR", powerBR);
            telemetry.update();
        }
    }
}
