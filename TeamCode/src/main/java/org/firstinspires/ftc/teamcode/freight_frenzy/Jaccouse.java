package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.HandRailClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.DriveClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.Location;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.Toggle;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.DuckLine;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous.AutoFlow;

import java.util.ArrayList;

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
	private AutoFlow.ALLIANCE alliance = AutoFlow.ALLIANCE.BLUE;

	Location startingPosition = new Location(-1.5 * tile, 2.75 * tile); //last x = -1.75*tile, y = 0*tile
	private DriveClass drive = new DriveClass(this, DriveClass.ROBOT.JACCOUSE, startingPosition, DriveClass.USE_BRAKE | DriveClass.USE_DASHBOARD_FIELD, alliance == AutoFlow.ALLIANCE.BLUE ? DriveClass.DriveMode.BLUE : DriveClass.DriveMode.RED); // TODO: useEncoders().
	private HandRailClass handRail = new HandRailClass(this);

	private Toggle turningToggle = new Toggle();

	private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
	private int direction = 1;
	private double targetHeading = 0;

	// HandRail variables
	private Toggle collector = new Toggle(); //  Collection toggle (A button)
//	boolean release; // releasing object (B button)
	private Toggle release = new Toggle();
	private Toggle homing = new Toggle();
	private Toggle spincarousel = new Toggle();
	private Toggle homingHand = new Toggle();
	private Toggle A = new Toggle();
	private Toggle B = new Toggle();
	private Toggle C = new Toggle();
	private Toggle X = new Toggle();
	private Toggle overrideLimits = new Toggle(); //key: , description: overrides handRail movement limitations
	private Toggle capping_button = new Toggle();
	private Toggle freightIn = new Toggle();
	private boolean capping_state = false;
	private ElapsedTime carouselAccelTime = new ElapsedTime();
	private ElapsedTime releaseCounter = new ElapsedTime();

	public double pow(double x){
		return Math.pow(x, 2) * Math.signum(x);
	}

	private final double defaultOrientationAngle = 90;

	@Override
	public void runOpMode() {
		telemetry.addData("Status", "Initialized");

		drive.init(hardwareMap);
		handRail.init(hardwareMap);


		telemetry.update();

		// Wait for the game to start (driver presses PLAY)
		waitForStart();

		handRail.searchHome();

		drive.resetOrientation(defaultOrientationAngle); //default blue

		runtime.reset();

		int turningCount = 0;

		// run until the end of the match (driver presses STOP)

		while (opModeIsActive()) {

			if (gamepad1.start) {
				if (gamepad1.x) {
					drive.resetOrientation(defaultOrientationAngle);
					this.alliance = AutoFlow.ALLIANCE.BLUE;
				}
				if (gamepad1.y) {
					drive.resetOrientation(-90);
					this.alliance = AutoFlow.ALLIANCE.RED;
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
					handRail.searchHome();
				}

				if (homingHand.isClicked())
					this.handRail.resetPotAndHand();
				continue;
			}


			boolean stopAll = gamepad1.y;
			//boolean intake = gamepad1.dpad_right || gamepad2.dpad_right; //
			boolean fieldOriented = !gamepad1.y;
			final double boostK = 0.5;
			double boost = gamepad1.right_trigger * boostK + (1 - boostK);


			double y = pow(-gamepad1.left_stick_y) * boost;
			double x = pow(gamepad1.left_stick_x) * boost;
			double turn = pow(gamepad1.right_stick_x * boost);

			// Hand rail
			final double handBoostK = 0.3;
			double boostHand = gamepad2.right_trigger * handBoostK + (1 - handBoostK);
//			double railPower = pow(gamepad2.left_stick_x * boostHand);// possibly changeable to gamepad2.left_stick_x
//			double armPower  = pow(gamepad2.right_stick_x * boostHand);
			double railPower = pow(gamepad2.left_stick_y * boostHand);// possibly changeable to gamepad2.left_stick_x
			double armPower  = pow(gamepad2.right_stick_y * boostHand);
			overrideLimits.update(gamepad2.right_bumper);

			// Hand limits, TODO: fix (adjust)

//			int limitThreshold = 10;

//			if (handRail.getHandPercent() > (100 - limitThreshold) && armPower > 0.4){
//				armPower = 0.2;
//			}
//
//			if (handRail.getHandPercent() < limitThreshold && armPower < -0.4){
//				armPower = -0.2;
//			}
			handRail.rail_drive(railPower, overrideLimits.getState());
			handRail.hand_drive(armPower, overrideLimits.getState());

			turningToggle.update(Math.abs(turn) > 0.02);
			spincarousel.update(gamepad1.left_bumper);


			A.update(gamepad2.a);
//			B.update(gamepad2.b);
			C.update(gamepad2.y);
			X.update(gamepad2.x);

			collector.update(gamepad2.dpad_down); // update toggle (A button)
//			release = gamepad2.dpad_up;
			release.update(gamepad2.dpad_up);

			if (turningToggle.isReleased()) {
				turningCount = 8;
			}
			if (!turningToggle.isPressed()) {
				turningCount--;
			}

			if (turningCount == 0) {
				targetHeading = drive.getHeading();
			}

//			freightIn.update(handRail.freightIn());
//			if (freightIn.isPressed()) {
//				if (freightIn.isPressed()) {
//					spincarousel.update(handRail.freightIn());
//				} else {
//					spincarousel.update(handRail.freightIn());
//				}
//			}



//			boolean blue = gamepad1.x;
//			boolean red = gamepad1.b;
//			if (blue) {
//				alliance = AutoFlow.ALLIANCE.BLUE;
//				telemetry.addData("Alliance is: ","Blue");
//			} else if(red) {
//				alliance = AutoFlow.ALLIANCE.RED;
//				telemetry.addData("Alliance is: ", "Red");
//			}

			if (!turningToggle.isPressed() && turningCount < 0) {
				double delta = drive.getDeltaHeading(targetHeading);
				double gain = 0.02;
				turn = delta * gain;
			}

			if (A.isClicked()) {
				handRail.gotoLevel(DuckLine.SH_Levels.Collect);
			}
//			else if (B.isClicked()) {
//				handRail.gotoLevel(DuckLine.SH_Levels.Middle);
//			}
			else if (C.isClicked()) {
				handRail.gotoLevel(DuckLine.SH_Levels.TopTeleop);
			} else if (X.isClicked()) {
				handRail.gotoLevel(DuckLine.SH_Levels.ReleaseShared);
			} else if (gamepad2.left_trigger > 0.5) {
				handRail.gotoLevel(DuckLine.SH_Levels.EndGamePark);
			}

			if (!release.isPressed()) {
				if (collector.getState()) {
					handRail.grabberGrab();
				}
			}
			else if(release.isClicked()) {
				handRail.grabberRelease();
				collector.set(false);
				releaseCounter.reset();
			}
			if(!collector.getState() && releaseCounter.milliseconds() > 200)
				handRail.grabberStop();





			// Carousel control
			double carouselBoost = gamepad1.left_trigger == 0 ?
					handRail.freightIn() ? 1 : 0
					: gamepad1.left_trigger;
			if(spincarousel.getState()) {
				if(spincarousel.isChanged()){
					carouselAccelTime.reset();
				}
				// y = mx + b, acc = speed * time * (1 - startOperator)
				double startOperator = 0.4;
				double speed = 1.15;
				double acc =  (1 - startOperator) * carouselAccelTime.seconds() * speed; // accelration limit equation
				handRail.carouselRun((startOperator + carouselBoost + acc) * alliance.mul);
			} else {
				handRail.carouselRun(carouselBoost * alliance.mul);
				//handRail.carouselStop();
			}

			// capping servo
//			double cappingPower = this.gamepad2.right_stick_y * -0.25;
//			double cappingPos = cappingPower + handRail.getCappingPos();
//			if(cappingPos > 1) cappingPos = 1;
//			if(cappingPos < -1) cappingPos = -1;
//			this.handRail.setCappingPos(cappingPos);
			capping_button.update(gamepad2.left_bumper);
			if (capping_button.isClicked()) {
				this.handRail.setCappingPos(capping_state ? 1 : 0);
				capping_state = !capping_state;
			}


			drive.setPowerOriented(y, x, turn, fieldOriented);

            this.handRail.telemetry_handRail();
            telemetry.addData("Pos", "X: %2.2f, \t Y: %2.2f", drive.getPosX(), drive.getPosY());
			telemetry.addData("Heading", drive.getHeading());
			telemetry.addData("Target", targetHeading);
			telemetry.addData("pressed", freightIn.isPressed());
			telemetry.addData("Delta", drive.getDeltaHeading(targetHeading));
			telemetry.update();

//			drive.update_dashboard_field();
		}
	}
}
