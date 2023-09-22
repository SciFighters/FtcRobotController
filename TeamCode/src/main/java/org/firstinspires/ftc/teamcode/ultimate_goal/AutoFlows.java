package org.firstinspires.ftc.teamcode.ultimate_goal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ultimate_goal.util.BananaPipeline;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.CvCam;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.DriveClass;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.GameClass;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.Location;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class AutoFlows {
	private LinearOpMode opMode; // First I declared it as OpMode now its LinearOpMode

	final double tile = 0.6;
	int mul;
	Location startingPosition;
	Location firstPos; // -0.25,0.73
	Location shootPos;
	Location a_pos;
	Location b_pos;
	Location c_pos;
	Location a_back_Pos;
	Location secondWobble_pos1;
	Location parkPos;
	Alliance alliance;

	double shootingAngle = 0; //default
	boolean shortened = false;
	enum ABC { A, B, C }

	public AutoFlows(LinearOpMode opMode, Alliance alliance, StartLine startline, boolean shortened) {
		this.opMode = opMode;
		this.shortened = shortened;
		this.alliance = alliance;
		if (alliance == Alliance.BLUE) {
			mul = blue;
		} else {
			mul = red;
		}
		// -1.4, 1.2
		startingPosition = new Location(0.6 * mul, 0);
		firstPos = new Location(0.27 * mul, 0.73); // -0.25,0.73
		shootPos = new Location(0.25 * mul, 1.13);
		a_pos = new Location(1.4 * mul, 1.45);
		b_pos = new Location(0.75 * mul, 2.15);
		c_pos = new Location(1.4 * mul, 2.65);
		a_back_Pos = new Location(1.10 * mul, 1.30);
		secondWobble_pos1 = new Location(2.5 * tile * mul, 2 * tile);
		parkPos = new Location(0.8 * mul, 2);

		if(startline == StartLine.OUTTER) {
			startingPosition = new Location(1.2 * mul, 0);
			firstPos = new Location(1.5 * mul,0.73);
			shootPos = new Location(1.5 * mul, 1.13);
			shootingAngle = 25.35;
		}
		if (alliance == Alliance.RED) {
			a_pos.x += 0.1;
			b_pos.x += 0.2;
			c_pos.x += 0.1;
			secondWobble_pos1.x -= 0.2;
		}
		if(alliance == Alliance.RED) {
			if (startline == StartLine.INNER) {
				shootingAngle = 25;
			} else {
				shootingAngle = 1;
			}
		} else {
			if(startline == StartLine.INNER) {
				shootingAngle = 0;
			} else {
				shootingAngle = -1;
			}
		}
		robot = new DriveClass(this.opMode, DriveClass.ROBOT.COBALT, startingPosition).useEncoders();
		game = new GameClass(this.opMode);    // Declare OpMode members.
	}

	BananaPipeline pipeline;
	OpenCvCamera cam;

	public enum Alliance {BLUE, RED}
	public enum StartLine {INNER,OUTTER}

	public enum FlowType {WALL, BRIDGE, SHORT, PARK_ONLY}


	private DriveClass robot;
	private GameClass game;

	public HardwareMap hardwareMap = null;


	private ElapsedTime runtime = new ElapsedTime();

	final int blue = -1;
	final int red = 1;

	public ABC getRingNum(BananaPipeline pipeline) {
		if (pipeline.getTargetRect() == null) {
			return (ABC.A);
		} else {
			Rect rect = pipeline.getTargetRect();
			if (rect.height < rect.width / 2) {
				return (ABC.B);
			} else {
				return (ABC.C);
			}
		}
	}


	private void initCamera() {
		cam = CvCam.getCam(this.opMode.hardwareMap, true);
		pipeline = new BananaPipeline();
		cam.setPipeline(pipeline);
//        this.opMode.telemetry.addData("pipeline initialized", pipeline);
//        this.opMode.telemetry.update();
		cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				cam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(int errorCode) {
				opMode.requestOpModeStop();
			}
		});
	}

	public void fullFlow() {
//        this.opMode.telemetry.addData("Status", "Initialized");
//        this.opMode.telemetry.update();

		initCamera();
		robot.init(this.opMode.hardwareMap);
		game.init(this.opMode.hardwareMap);


		game.initLifterPosition();
		game.setWobbleGrabber(false);
		game.initWobbleArmPosition();

		ABC abc = getRingNum(pipeline);
		this.opMode.telemetry.addData("Rings: ", abc);
		this.opMode.telemetry.update();

		// ================================================
		// ================================================
		// Wait for the game to start (driver presses PLAY)
		// ================================================
		// ================================================
		this.opMode.waitForStart();
		runtime.reset();

		abc = getRingNum(pipeline);// a b c?
        this.opMode.telemetry.addData("Rings: ", abc); //finding if it's a , b or c
        this.opMode.telemetry.update();

		game.wobbleArmGoTo(1500); //wobble up
		game.setSuperPosition(true);// fire position
		game.lifterUpDownSecondStage(true);

		double heading = robot.getHeading();

		//go shoot
		robot.goToLocation(firstPos, 1, heading, 0.15);
		robot.goToLocation(shootPos, 1, heading, 0.01);
		robot.turnTo(shootingAngle, 1);
		game.update();
		// robot.turnTo(20, 0.6);

		while (opMode.opModeIsActive() && !game.getSuperState()) ;

		for (int x = 0; x < 3; x++) { // fire ring
			game.update();
			this.opMode.sleep(1500);
			game.update();
			game.shoot();
			game.update();
		}

		this.opMode.sleep(1000);
		game.setSuperPosition(false); //intake
		game.lifterUpDownSecondStage(false);
		// robot.turnTo(0, 0.6);
        this.opMode.telemetry.addData("going to", abc);
        this.opMode.telemetry.update();

		// go to first wobble position
		switch (abc) {
			case A:
				robot.goToLocation(a_pos.offset(0.15), 1, heading, 0.05);
				break;
			case B:
				robot.goToLocation(b_pos.offset(0.15), 1, heading, 0.05);
				break;
			case C:
				robot.goToLocation(c_pos.offset(0.15), 1, heading, 0.05);
				break;
		}

		//drop #1wobble
		//Last current position - tiles: (x: -0.5, y: 4.5)
		game.wobbleArmGoTo(5778);
		this.opMode.sleep(1000);
		game.setWobbleGrabber(true);
		this.opMode.sleep(350);

		if (abc == ABC.A) {
			robot.goToLocation(a_back_Pos, 1, heading, 0.05);
		}

		if(shortened == false) {
			// going to pick up the second wobble rod
			robot.goToLocation(secondWobble_pos1, 1, heading, 0.1);
			if (alliance == Alliance.RED){
				game.wobbleArmGoTo(4000);
			}

			int targetAngle = (alliance == Alliance.BLUE) ? 175 : -175;

			robot.turnTo(targetAngle, 1);

			robot.drive(0.40, 0, 1, targetAngle, false);

			// picking up the second wobble
			game.wobbleArmGoTo(6500);
			opMode.sleep(300);
			game.setWobbleGrabber(false);
			opMode.sleep(700);
			game.wobbleArmGoTo(4000);

			// going back
			robot.drive(-0.40, 0, 1, targetAngle, false);
			opMode.sleep(150);
			robot.turnTo(heading, 1);

			switch (abc) {
				case A:
					robot.goToLocation(a_pos, 1, heading, 0.05);
					break;
				case B:
					robot.goToLocation(b_pos, 1, heading, 0.05);
					break;
				case C:
					robot.goToLocation(c_pos, 1, heading, 0.05);
					break;
			}
			// Putting down the second wobble
			game.wobbleArmGoTo(5778);
			opMode.sleep(400);
			game.setWobbleGrabber(true);
			opMode.sleep(350);

			if (abc == ABC.A) {
				robot.goToLocation(a_back_Pos, 1, heading, 0.05);
			}

			game.wobbleArmGoTo(100);
		}
		robot.goToLocation(parkPos, 1, heading, 0.05);
		game.setWobbleGrabber(false);
	}

}
