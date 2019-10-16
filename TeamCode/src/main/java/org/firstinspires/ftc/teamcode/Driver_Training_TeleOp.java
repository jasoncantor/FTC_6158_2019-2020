import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import android.os.Handler;
import android.os.SystemClock;
import android.util.Log;
import android.view.ViewParent;
 
public class Driver_Training_TeleOp extends OpMode {
    DcMotor frontleft, frontright, backleft, backright;
    Servo   arm;
	public float x, y, z, w, pwr;
	public static double deadzone = 0.2;
	y = .5
 
 
	@Override
	public void init() {
		frontleft = hardwareMap.dcMotor.get("front_left");
		frontright = hardwareMap.dcMotor.get("front_right");
		backleft = hardwareMap.dcMotor.get("back_left");
        backright = hardwareMap.dcMotor.get("back_right");
        arm = hardwareMap.Servo.get("arm");
		
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
        
        if(gamepad1.y) {
            // move to 0 degrees.
            arm.setPosition(0);
        } else if (gamepad1.a) {
            // move to 180 degrees.
            arm.setPosition(1);
        }
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