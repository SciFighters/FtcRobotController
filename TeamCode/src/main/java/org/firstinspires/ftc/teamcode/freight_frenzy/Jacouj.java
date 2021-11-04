package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ultimate_goal.util.DriveClass;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.GameClass;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.Location;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.Toggle;

@TeleOp(group = "Jacouj")
public class Jacouj extends LinearOpMode {
	final double tile = 0.6;

	// Declare OpMode members.
	private ElapsedTime runtime = new ElapsedTime();
	Location startingPosition = new Location(0 * tile, 0 * tile); //last x = -1.75*tile, y = 0*tile
	private DriveClass drive = new DriveClass(this, DriveClass.ROBOT.COBALT, startingPosition).useEncoders().useBrake();
	private GameClass game = new GameClass(this);
	private Toggle turningToggle = new Toggle();
	private int direction = 1;
	private double targetHeading = 0;

	@Override
	public void runOpMode() {
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		drive.init(hardwareMap);
		game.init(hardwareMap);

		game.initLifterPosition();

		// Wait for the game to start (driver presses PLAY)
		waitForStart();

		drive.resetOrientation(90); //default blue

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
			boolean intake = gamepad1.dpad_right || gamepad2.dpad_right; // down armShooter

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

			drive.setPowerOriented(y, x, turn, fieldOriented);

			if (stopAll) {
				game.stopAll();
			}

			game.lifterMoveManually(-gamepad1.right_stick_y-gamepad2.right_stick_y);
			telemetry.addData("X Pos", drive.getPosX());
			telemetry.addData("Y Pos", drive.getPosY());
			telemetry.addData("Heading", drive.getHeading());
			telemetry.addData("Target", targetHeading);
			telemetry.addData("Delta", drive.getDeltaHeading(targetHeading));

			game.update();
			telemetry.update();
		}
	}
}
