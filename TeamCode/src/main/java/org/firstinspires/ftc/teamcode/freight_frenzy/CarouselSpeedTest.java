package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.DriveClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.HandRailClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.Location;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.Toggle;

@TeleOp
@Disabled
public class CarouselSpeedTest extends LinearOpMode {
	DriveClass drive = new DriveClass(this, DriveClass.ROBOT.COBALT, new Location(0, 0), DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE, DriveClass.DriveMode.BLUE);
	HandRailClass handRail = new HandRailClass(this);

	private double tmp_power = 0.2;
	private Toggle tmp_dec = new Toggle();
	private Toggle tmp_inc = new Toggle();

	@Override
	public void runOpMode() {
		drive.init(hardwareMap);
		handRail.init(hardwareMap);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		drive.resetOrientation(0); //default blue


		waitForStart();

		while (opModeIsActive()) {
			if (gamepad1.start) {
				if (gamepad1.x) {
					drive.resetOrientation(0);
				}

				continue;
			}


			tmp_dec.update(gamepad1.left_bumper);
			tmp_inc.update(gamepad1.right_bumper);

			if (tmp_dec.isClicked()) {
				tmp_power -= 0.05;
			} else if (tmp_inc.isClicked()) {
				tmp_power += 0.05;
			}

			if (gamepad1.a) {
				handRail.carouselRun(tmp_power);
			} else {
				handRail.carouselRun(0);
			}


			double boost = gamepad1.right_trigger * 0.6 + 0.4;

			double y = -gamepad1.left_stick_y * boost;
			double x = gamepad1.left_stick_x * boost;
			double turn = gamepad1.right_stick_x * boost;

			drive.setPowerOriented(y, x, turn, true);

			telemetry.addData("carousel power", tmp_power);
			telemetry.update();
		}
	}
}
