package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.DriveClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.Location;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.Toggle;

@TeleOp(group = "Jacouj")
//@Disabled
public class GoToTest extends LinearOpMode {
	final double tile = 0.6;

	// Declare OpMode members.
	private ElapsedTime runtime = new ElapsedTime();
	Location startingPosition = new Location(0 * tile, 0 * tile); //last x = -1.75*tile, y = 0*tile
	private DriveClass drive = new DriveClass(this, DriveClass.ROBOT.COBALT, startingPosition, DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE, DriveClass.DriveMode.BLUE);
	private Toggle turningToggle = new Toggle();
	private int direction = 1;
	private double targetHeading = 0;


	private Toggle x_toggle = new Toggle();
	private Toggle y_toggle = new Toggle();
	private Toggle a_toggle = new Toggle();
	private Toggle b_toggle = new Toggle();

	@Override
	public void runOpMode() {
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		drive.init(hardwareMap);


		// Wait for the game to start (driver presses PLAY)
		waitForStart();

		drive.resetOrientation(0); //default blue

		runtime.reset();

		int turningCount = 0;

		// run until the end of the match (driver presses STOP)
		while (opModeIsActive()) {

			boolean resetOrientation = gamepad1.start;

			if (resetOrientation) {
				if (gamepad1.x) {
					drive.resetOrientation(90);
				}
				if (gamepad1.y) {
					drive.resetOrientation(-90);
				}
				drive.resetPosition();
				targetHeading = drive.getHeading();
				continue;
			}

			boolean stopAll = gamepad1.a || gamepad2.a;
			boolean intake = gamepad1.dpad_right || gamepad2.dpad_right; // ?

			boolean fieldOriented = !gamepad1.y;
			double boost = gamepad1.right_trigger * 0.6 + 0.4;

			double y = -gamepad1.left_stick_y * boost;
			double x = gamepad1.left_stick_x * boost;
			double turn = gamepad1.right_stick_x * boost;

			turningToggle.update(Math.abs(turn) > 0.02);

			if (turningToggle.isReleased()) {
				turningCount = 8;
			}
			if (!turningToggle.isPressed()) {
				turningCount--;
			}

			if (turningCount == 0) {
				targetHeading = drive.getHeading();
			}

			if (!turningToggle.isPressed() && turningCount < 0) {
				double delta = drive.getDeltaHeading(targetHeading);
				double gain = 0.05;
				turn = delta * gain;
			}

			a_toggle.update(gamepad1.a);
			b_toggle.update(gamepad1.b);
			y_toggle.update(gamepad1.y);
			x_toggle.update(gamepad1.x);

			if (a_toggle.isClicked())
				drive.goTo(0.0,0.0,0.7,0.0,0.05, 0);
			if (b_toggle.isClicked())
				drive.goTo(0.0,1.2,0.7,0.0,0.05, 0);
			if (y_toggle.isClicked())
				drive.goTo(1.2,30.0,0.7,0.0,0.05, 0);
			if (x_toggle.isClicked())
				drive.goTo(-1.2,30,0.7,0.0,0.05, 0);
			drive.setPowerOriented(y, x, turn, fieldOriented);

			telemetry.addData("X Pos", drive.getPosX());
			telemetry.addData("Y Pos", drive.getPosY());
			telemetry.addData("Heading", drive.getHeading());
			telemetry.addData("Target", targetHeading);
			telemetry.addData("Delta", drive.getDeltaHeading(targetHeading));
			telemetry.update();
		}
	}
}
