package org.firstinspires.ftc.teamcode.power_play;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.power_play.util.DriveClass;
import org.firstinspires.ftc.teamcode.power_play.util.Lift;
import org.firstinspires.ftc.teamcode.power_play.util.Location;

@TeleOp
public class Constantin extends LinearOpMode {
    DriveClass drive = new DriveClass(this, DriveClass.ROBOT.JACCOUSE, new Location(0, 0), DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE, DriveClass.DriveMode.BLUE);
    private Lift lift = new Lift();

    @Override
    public void runOpMode() {
        telemetry.addLine("Starting Initializing");
        telemetry.update();

        drive.init(hardwareMap);
        lift.init(hardwareMap);
        telemetry.addLine("Finished Initializing");
        telemetry.update();

        waitForStart();

        drive.resetOrientation(90);
        while (opModeIsActive()) {
            final double boostK = 0.5;
            double boost = gamepad1.right_trigger * boostK + (1 - boostK);

            double y = pow(-gamepad1.left_stick_y) * boost;
            double x = pow(gamepad1.left_stick_x) * boost;
            double turn = pow(gamepad1.right_stick_x * boost);
            double power = 0;
            drive.setPowerOriented(y, x, turn, true);
            if (gamepad1.right_stick_y * 100 >= 10 || gamepad1.right_stick_y * 100 <= -10) {
                if (lift.getPos() <= -2000) {
                    lift.breakMotor();
                } else {
                    power = gamepad1.right_stick_y;
                }
                this.lift.setPower(power);
                telemetry.addData("Lift Power", power);

            } else {
                this.lift.setPower(0);
                telemetry.addData("Lift Power", 0);
            }
            telemetry.addData("lift pos : ", lift.getPos());
            telemetry.update();
        }
    }

    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x);
    }
}
