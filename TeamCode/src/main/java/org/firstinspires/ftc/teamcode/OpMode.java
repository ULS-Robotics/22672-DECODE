package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Teleop", group = "DECODE")
public class OpMode extends LinearOpMode {

    // Hardware
    DcMotor motorFL, motorFR, motorBL, motorBR;
    DcMotor shooterL, shooterR;
    IMU imu;

    //AprilTagProcessor aprilTag;
    //VisionPortal visionPortal;

    // Subsystems
    DriveSubsystem drive;
    IntakeSubsystem intakeSystem;
    ShooterSubsystem shooterSystem;

    @Override
    public void runOpMode() {

        // ---------------------------
        // Hardware Mapping
        // ---------------------------
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        shooterL = hardwareMap.get(DcMotor.class, "shooterL");
        shooterR = hardwareMap.get(DcMotor.class, "shooterR");

        Servo intake = hardwareMap.get(Servo.class, "Intake");


        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);

        shooterL.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setDirection(DcMotor.Direction.FORWARD);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(parameters);

        // AprilTag
        /*aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();*/

        // ---------------------------
        // Initialize Subsystems
        // ---------------------------
        drive = new DriveSubsystem(motorFL, motorFR, motorBL, motorBR, imu);
        intakeSystem = new IntakeSubsystem(intake);
        shooterSystem = new ShooterSubsystem(shooterL, shooterR);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        imu.resetYaw();

        while (opModeIsActive()) {

            // Reset IMU if button A
            if (gamepad1.a) imu.resetYaw();

            // Apply deadzones to joystick values
            double y = -applyDeadzone(gamepad1.left_stick_y);
            double x = applyDeadzone(gamepad1.left_stick_x);
            double rotation = applyDeadzone(gamepad1.right_stick_x) * 0.5;

            // Drive
            drive.driveFieldCentric(y, x, rotation, gamepad1);

            // Intake & Shooter
            intakeSystem.handleIntake(gamepad2);
            shooterSystem.handleShooter(gamepad2);

            // Telemetry
            telemetry.update();
        }

        //if (visionPortal != null) visionPortal.close();
    }

    // ---------------------------
    // Deadzone Helper
    // ---------------------------
    private double applyDeadzone(double value) {
        return Math.abs(value) > 0.1 ? value : 0;
    }

    // ======================================================
    //  =================== SUBSYSTEMS =====================
    // ======================================================

    private static class DriveSubsystem {
        DcMotor motorFL, motorFR, motorBL, motorBR;
        IMU imu;

        public DriveSubsystem(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, IMU imu) {
            this.motorFL = fl;
            this.motorFR = fr;
            this.motorBL = bl;
            this.motorBR = br;
            this.imu = imu;
        }

        public void driveFieldCentric(double y, double x, double rotation, Gamepad gamepad) {
            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(heading) - y * Math.sin(heading);
            double rotY = x * Math.sin(heading) + y * Math.cos(heading);

            double fl = rotY + rotX + rotation;
            double fr = rotY - rotX - rotation;
            double bl = rotY - rotX + rotation;
            double br = rotY + rotX - rotation;

            // D-pad overrides
            if (gamepad.dpad_left) { fl=-1; fr=1; bl=1; br=-1; }
            else if (gamepad.dpad_right) { fl=1; fr=-1; bl=-1; br=1; }
            else if (gamepad.dpad_up) { fl=fr=bl=br=1; }
            else if (gamepad.dpad_down) { fl=fr=bl=br=-1; }

            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

            motorFL.setPower(fl / max);
            motorFR.setPower(fr / max);
            motorBL.setPower(bl / max);
            motorBR.setPower(br / max);
        }
    }

    private static class IntakeSubsystem {
        private final Servo intake;

        private static final double INTAKE_IN = 0.75;
        private static final double INTAKE_OUT = 0.0;
        private static final double INTAKE_STOP = 0.5;

        public IntakeSubsystem(Servo intake) {
            this.intake = intake;
            intake.setPosition(INTAKE_STOP);
        }

        public void handleIntake(Gamepad gamepad) {

            boolean up = gamepad.dpad_up;
            boolean down = gamepad.dpad_down;
            boolean left = gamepad.dpad_left;
            boolean right = gamepad.dpad_right;

            int pressedCount = 0;
            if (up) pressedCount++;
            if (down) pressedCount++;
            if (left) pressedCount++;
            if (right) pressedCount++;

            if (pressedCount == 1) {
                if (up) {
                    intake.setPosition(INTAKE_IN);
                } else if (down) {
                    intake.setPosition(INTAKE_OUT);
                } else {
                    intake.setPosition(INTAKE_STOP); // left or right alone
                }
            } else {
                intake.setPosition(INTAKE_STOP); // multiple or none
            }
        }
    }

    private static class ShooterSubsystem {
        DcMotor shooterL, shooterR;

        public ShooterSubsystem(DcMotor l, DcMotor r) {
            this.shooterL = l;
            this.shooterR = r;
        }

        public void handleShooter(Gamepad gamepad) {
            double power;
            power = gamepad.right_trigger > 0 ? -0.55 : 0;
            shooterL.setPower(power);
            shooterR.setPower(power);
        }
    }
}
