package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp 2019-2020", group="REV")
public class Teleop_This_Year extends OpMode {
	DcMotor frontleft, frontright, backleft, backright, armhoz, armro;
	Servo arm, arm2;
	public float x, y, z, w, pwr, leftstick, rightstick;
	public static double deadzone = 0.2;
	public  static double power = 0.40;

	@Override
	public void init() {
		frontleft = hardwareMap.dcMotor.get("frontleft");
		frontright = hardwareMap.dcMotor.get("frontright");
		backleft = hardwareMap.dcMotor.get("backleft");
		backright = hardwareMap.dcMotor.get("backright");
		armhoz = hardwareMap.dcMotor.get("armhoz"); // this is the arm that move on a horizontal axis
		armro = hardwareMap.dcMotor.get("armro"); //this is the arm that rotates
		arm = hardwareMap.servo.get("arm"); //this is the servo that grabs
		arm2 = hardwareMap.servo.get("arm2"); // this is the one that grabs too if there is two servos then uncomment this but the first comment slash

		frontright.setDirection(DcMotor.Direction.REVERSE);
		backright.setDirection(DcMotor.Direction.REVERSE);
		armhoz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		armro.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		arm.setPosition(0);
		arm2.setPosition(1); // Uncomment this is there is two servo for grab

	}

	@Override
	public void loop() {
		double amountofpowerforarmro, amountofpowerforarmhoz;
		getJoyVals();
		//updates joyvalues with deadzones, xyzw

		pwr = y; //this can be tweaked for exponential power increase
		amountofpowerforarmro = 1; // Change this if you want to increase or decrese the power for the amount of power for armro
		amountofpowerforarmhoz = 0.25; // Change this if you want to increase or decrese the power for the amount of power for armhoz

		frontright.setPower(Range.clip(pwr - x+z, -power, power));
		backleft.setPower(Range.clip(pwr - x-z, -power, power));
		frontleft.setPower(Range.clip(pwr + x-z, -power, power));
		backright.setPower(Range.clip(pwr + x+z, -power, power));


        /*if (math.abs(gamepad2.left_stick_y) < .2) {
            armhoz.setMode(DcMotor.RunMode.RESET_ENCODERS);
            armhoz.setTargetPosition(1);
            armhoz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armhoz.setPower(.01);
        }*/
		armro.setPower(Range.clip(/*amountofpowerforarmro + */gamepad2.right_stick_y, -amountofpowerforarmro, amountofpowerforarmro));
		armhoz.setPower(Range.clip(/*amountofpowerforarmhoz + */gamepad2.left_stick_y, -amountofpowerforarmhoz, amountofpowerforarmhoz));

		if(gamepad2.y){
			arm.setPosition(1);
			arm2.setPosition(0); // Uncomment this is there is two servo for grab
		} else if (gamepad2.x || gamepad2.b){
			arm.setPosition(0.5);
			arm2.setPosition(-0.5); // Uncomment this is there is two servo for grab
		} else if (gamepad2.a){
			arm.setPosition(0);
			arm2.setPosition(1); // Uncomment this is there is two servo for grab
		}
	}

	public void getJoyVals()
	{
		y = gamepad1.left_stick_y;
		x = gamepad1.left_stick_x;
		z = gamepad1.right_stick_x;
		w = gamepad1.right_stick_y;
		leftstick = gamepad2.left_stick_y;
		rightstick = gamepad2.right_stick_y;
		//updates joystick values

		if(Math.abs(x)<deadzone) x = 0;
		if(Math.abs(y)<deadzone) y = 0;
		if(Math.abs(z)<deadzone) z = 0;
		if(Math.abs(w)<0.9) w = 0;
		if(Math.abs(rightstick)<deadzone) rightstick = 0;
		if(Math.abs(leftstick)<deadzone) leftstick = 0;
		//checks deadzones
	}
}