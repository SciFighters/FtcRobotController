package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.HandRailClass;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.DriveClass;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.GameClass;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.Location;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.Toggle;

@TeleOp(group = "Jacouj")
public class Jaccouse extends LinearOpMode {
	final double tile = 0.6;


	// Declare OpMode members.
	private ElapsedTime runtime = new ElapsedTime();
	Location startingPosition = new Location(0 * tile, 0 * tile); //last x = -1.75*tile, y = 0*tile
	private DriveClass drive = new DriveClass(this, DriveClass.ROBOT.COBALT, startingPosition).useEncoders().useBrake();
	private Toggle turningToggle = new Toggle();
	org.firstinspires.ftc.teamcode.freight_frenzy.util.Location startingPoisition = new org.firstinspires.ftc.teamcode.freight_frenzy.util.Location(0 * tile, 0 * tile);
	private HandRailClass handRail = new HandRailClass(this);

	private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
	private int direction = 1;
	private double targetHeading = 0;

	private Toggle collector = new Toggle(); //  Collection toggle (A button)
	boolean release; // releasing object (B button)
	private Toggle homing = new Toggle();
	private Toggle spincarousel = new Toggle();

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
				if (gamepad2.x) {
					drive.resetOrientation(90);
				}
				if (gamepad2.y) {
					drive.resetOrientation(-90);
				}
				drive.resetPosition();
				targetHeading = drive.getHeading();
				continue;
			}

			boolean stopAll = gamepad1.y;
			//boolean intake = gamepad1.dpad_right || gamepad2.dpad_right; // ?

			boolean fieldOriented = !gamepad2.y;
			double boost = gamepad2.right_trigger * 0.6 + 0.4;

			double railPower = -gamepad2.right_trigger;
			double armPower = -gamepad2.right_stick_y;

			double y = -gamepad2.left_stick_y * boost;
			double x = gamepad2.left_stick_x * boost;
			double turn = gamepad2.right_stick_x * boost;

			handRail.rail_drive(railPower);
			handRail.hand_drive(armPower);

			turningToggle.update(Math.abs(turn) > 0.02);
			spincarousel.update(gamepad1.left_bumper );

			collector.update(gamepad1.a ); // update toggle (A button)
			//homing.update(gamepad1.x);
			release = gamepad1.b;

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

			if (!release) {
				if (collector.getState()) {
					handRail.grabberGrab();
				} else {
					handRail.grabberStop();
				}
			}
			else {
				handRail.grabberRelease();
				collector.set(false);
			}

			if(homing.isClicked()) {
				handRail.searchHome();
			}

			if(spincarousel.isChanged())
				handRail.carouselRun(0.9);
			else {
				handRail.carouselStop();
			}

			this.handRail.update_handRail();

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
