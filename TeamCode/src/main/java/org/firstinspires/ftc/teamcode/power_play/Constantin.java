package org.firstinspires.ftc.teamcode.power_play;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.power_play.util.DriveClass;
import org.firstinspires.ftc.teamcode.power_play.util.Lift;
import org.firstinspires.ftc.teamcode.power_play.util.Location;

@TeleOp
public class Constantin extends LinearOpMode {
    private DcMotorEx fl = null, fr = null, bl = null, br = null;

    private Lift lift = new Lift();

    String state = "WITHOUT ENCODER";
    DriveClass drive = new DriveClass(this, DriveClass.ROBOT.JACCOUSE, new Location(0, 0), DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE, DriveClass.DriveMode.BLUE);

    @Override
    public void runOpMode() {
        telemetry.addLine("Starting Initializing");
        telemetry.update();

        drive.init(hardwareMap);
        lift.init(hardwareMap);
        telemetry.addLine("Finished Initializing");
        telemetry.update();

        waitForStart();

        drive.resetOrientation(0);
        while (opModeIsActive()) {
            final double boostK = 0.5;
            double boost = gamepad1.right_trigger * boostK + (1 - boostK);

            double y = pow(-gamepad1.left_stick_y) * boost;
            double x = pow(gamepad1.left_stick_x) * boost;
            double turn = pow(gamepad1.right_stick_x * boost);
            double power = 0;
            drive.setPowerOriented(y, x, turn, true);
            if (gamepad2.right_stick_y * 100 >= 10 || gamepad2.right_stick_y * 100 <= -10) { // Checks if the gamepad2 (right stick y axis)
                power = gamepad2.right_stick_y;
                if (lift.getPos() < 3000) telemetry.addData("Lift Power", power);
                else power = 0;
                this.lift.setPower(power);
                this.lift.fixPos(lift.getPos());
                this.lift.currentTarget = this.lift.getRelativePos();
            } else {
//                lift.fixPos(lift.getPos());
                lift.goTo(false);
                telemetry.addData("Lift Power", 0);
            }
            if (gamepad1.x && gamepad1.start) {
                drive.resetOrientation(0);
            }


            if (gamepad2.dpad_up) lift.setGrabbers(0.8);
            else if (gamepad2.dpad_down) lift.setGrabbers(-0.8);
            else lift.setGrabbers(0);


            telemetry.addData("lift pos : ", lift.getPos());
            telemetry.update();
        }
    }

    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x);
    }
}
