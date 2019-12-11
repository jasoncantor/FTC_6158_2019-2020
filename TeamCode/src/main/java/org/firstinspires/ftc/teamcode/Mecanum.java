package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Memecanum", group="REV")
public class Mecanum extends OpMode {
	DcMotor frontleft, frontright, backleft, backright;
	public float x, y, z, w, pwr;
	public static double deadzone = 0.2;


	@Override
	public void init() {
		frontleft = hardwareMap.dcMotor.get("frontleft");
		frontright = hardwareMap.dcMotor.get("frontright");
		backleft = hardwareMap.dcMotor.get("backleft");
		backright = hardwareMap.dcMotor.get("backright");

		frontright.setDirection(DcMotor.Direction.REVERSE);
		backright.setDirection(DcMotor.Direction.REVERSE);

	}

	@Override
	public void loop() {
		getJoyVals();
		//updates joyvalues with deadzones, xyzw

		pwr = y; //this can be tweaked for exponential power increase

		frontright.setPower(Range.clip(pwr - x+z, -1, 1));
		backleft.setPower(Range.clip(pwr - x-z, -1, 1));
		frontleft.setPower(Range.clip(pwr + x-z, -1, 1));
		backright.setPower(Range.clip(pwr + x+z, -1, 1));
	}

	public void getJoyVals()
	{
		y = gamepad1.left_stick_y;
		x = gamepad1.left_stick_x;
		z = gamepad1.right_stick_x;
		w = gamepad1.right_stick_y;
		//updates joystick values

		if(Math.abs(x)<deadzone) x = 0;
		if(Math.abs(y)<deadzone) y = 0;
		if(Math.abs(z)<deadzone) z = 0;
		if(Math.abs(w)<0.9) w = 0;
		//checks deadzones
	}
}