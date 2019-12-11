package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MEMMES", group="REV")

public class PracticeBotCode1920 extends LinearOpMode {
    private DcMotor frontRight, frontLeft, backRight, backLeft;

    @Override
    public void runOpMode() {
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        backLeft = hardwareMap.get(DcMotor.class, "bl");



        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        double frpwr, flpwr, brpwr, blpwr, turn;
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            frpwr = -gamepad1.left_stick_y;
            flpwr = gamepad1.left_stick_y;
            brpwr = -gamepad1.left_stick_y;
            blpwr = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            frpwr = Range.clip(frpwr, -1.0, 1.0);
            brpwr = Range.clip(brpwr, -1.0, 1.0);
            flpwr = Range.clip(flpwr, -1.0, 1.0);
            blpwr = Range.clip(blpwr, -1.0,1.0);
            frontRight.setPower(frpwr);
            frontLeft.setPower(flpwr);
            backRight.setPower(brpwr);
            backLeft.setPower(blpwr);
        }
    }
}
/*public class GetJoyValues() {

}*/