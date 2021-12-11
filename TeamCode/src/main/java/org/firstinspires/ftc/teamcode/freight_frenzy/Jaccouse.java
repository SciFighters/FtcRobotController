package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.HandRailClass;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.DriveClass;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.Location;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.Toggle;

// TODO: clean code
// TODO: hand rail boost
// TODO: hand rail coupling
// TODO: hand rail preset positions
// TODO: hand rail RED BLUE Flipping.
// TODO: drive heading correction - reduce game

@TeleOp(group = "Jaccouse")
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

	// HandRail variables
	private Toggle collector = new Toggle(); //  Collection toggle (A button)
	boolean release; // releasing object (B button)
	private Toggle homing = new Toggle();
	private Toggle spincarousel = new Toggle();
	private Toggle homingHand = new Toggle();
	private Toggle A = new Toggle();
	private Toggle B = new Toggle();
	private Toggle C = new Toggle();
	private Toggle X = new Toggle();

	@Override
	public void runOpMode() {
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		drive.init(hardwareMap);
		handRail.init(hardwareMap);


		// Wait for the game to start (driver presses PLAY)
		waitForStart();

		drive.resetOrientation(0); //default blue

		runtime.reset();

		int turningCount = 0;

		// run until the end of the match (driver presses STOP)
		while (opModeIsActive()) {

			if (gamepad1.start) {
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


			if (gamepad2.start) {
				homing.update(gamepad2.x);
				homingHand.update(gamepad2.y);

				if (homing.isClicked()) {
					telemetry.addData("[searchHome]", "starting");
					handRail.searchHomeRail();
				}

				if (homingHand.isClicked())
					handRail.searchHomeHand();
			}


			boolean stopAll = gamepad1.y;
			//boolean intake = gamepad1.dpad_right || gamepad2.dpad_right; //
			boolean fieldOriented = !gamepad1.y;
			double boost = gamepad1.right_trigger * 0.6 + 0.4;

			double y = -gamepad1.left_stick_y * boost;
			double x = gamepad1.left_stick_x * boost;
			double turn = gamepad1.right_stick_x * boost;

			// Hand rail
			double railPower = -gamepad2.left_stick_x;
			double armPower =   gamepad2.right_stick_x;

			handRail.rail_drive(Math.pow(railPower,2) * Math.signum(railPower));
			handRail.hand_drive(Math.pow(armPower,2) * Math.signum(armPower));

			turningToggle.update(Math.abs(turn) > 0.02);
			spincarousel.update(gamepad1.left_bumper);

			A.update(gamepad2.a);
			B.update(gamepad2.b);
			C.update(gamepad2.y);
			X.update(gamepad2.x);
			collector.update(gamepad2.dpad_down); // update toggle (A button)
			release = gamepad2.dpad_up;

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

//			if(A.isClicked()) {
//				handRail.gotoA();
//			} else if (B.isClicked()){
//				handRail.gotoB();
//			} else if (C.isClicked()) {
//				handRail.gotoC();
//			} else if (X.isClicked()){
//				handRail.gotoX();
//			}

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

			if(spincarousel.getState())
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
