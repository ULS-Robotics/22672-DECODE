package org.firstinspires.ftc.teamcode;

// ---------------------------
// Import Statements
// ---------------------------
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="DECODE Auto Down", group="DECODE")
public class AutoRight extends LinearOpMode{
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            // ---------------------------
            // Hardware Mapping
            // ---------------------------
            DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFL");
            DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFR");
            DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBL");
            DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBR");
            motorFL.setDirection(DcMotor.Direction.REVERSE);
            motorFR.setDirection(DcMotor.Direction.FORWARD);
            motorBL.setDirection(DcMotor.Direction.REVERSE);
            motorBR.setDirection(DcMotor.Direction.FORWARD);
            while (getRuntime() < 6) {
                if (getRuntime() > 5) {
                    motorBR.setPower(-1);
                    motorBL.setPower(-1);
                    motorFR.setPower(-1);
                    motorFL.setPower(-1);
                }
            }
            telemetry.addData("Runtime", getRuntime());
            telemetry.addData("FL", motorFL.getPower());
            telemetry.addData("FR", motorFR.getPower());
            telemetry.addData("BL", motorBL.getPower());
            telemetry.addData("BR", motorBR.getPower());
            telemetry.update();
        }
    }
}
