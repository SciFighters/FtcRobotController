package org.firstinspires.ftc.teamcode.power_play;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.MathUtil;
import org.firstinspires.ftc.teamcode.power_play.util.DriveClass;
import org.firstinspires.ftc.teamcode.power_play.util.Lift;
import org.firstinspires.ftc.teamcode.power_play.util.Location;
import org.firstinspires.ftc.teamcode.power_play.util.Toggle;

@TeleOp
public class Constantin extends LinearOpMode {
    Lift lift = new Lift();
    DriveClass drive = new DriveClass(this, DriveClass.ROBOT.JACCOUSE, new Location(0, 0), DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE, DriveClass.DriveMode.BLUE);

    Toggle A = new Toggle();
    Toggle B = new Toggle();
    Toggle X = new Toggle();
    Toggle Y = new Toggle();

    @Override
    public void runOpMode() {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Starting Initializing");
        telemetry.update();

        drive.init(hardwareMap);
        lift.init(hardwareMap);
        telemetry.addLine("Finished Initializing");
        telemetry.update();

        waitForStart();

        drive.resetOrientation(0);
        while (opModeIsActive()) {
            if (gamepad1.start && gamepad1.x) {
                drive.resetOrientation(0);
            }

            final double boostK = 0.5;
            double boost = gamepad1.right_trigger * boostK + (1 - boostK);

            double y = pow(-gamepad1.left_stick_y) * boost;
            double x = pow(gamepad1.left_stick_x) * boost;
            double turn = pow(gamepad1.right_stick_x * boost);
            drive.setPowerOriented(y, x, turn, true);

            if (gamepad2.dpad_up) lift.setGrabbersPower(0.8);
            else if (gamepad2.dpad_down) lift.setGrabbersPower(-0.8);
            else lift.setGrabbersPower(0);

            lift.setLiftPower(-gamepad2.right_stick_y);

            A.update(gamepad2.a);
            B.update(gamepad2.b);
            X.update(gamepad2.x);
            Y.update(gamepad2.y);

            if (A.isClicked()) lift.gotoLevel(Lift.LiftLevel.Floor);
            if (B.isClicked()) lift.gotoLevel(Lift.LiftLevel.First);
            if (X.isClicked()) lift.gotoLevel(Lift.LiftLevel.Second);
            if (Y.isClicked()) lift.gotoLevel(Lift.LiftLevel.Third);

//            for (Levels level : levels) {
//                level.update(this);
//
//                if (level.isPressed())
//                    this.lift.setTargetPos(level.ticks);
//            }

//            if (MathUtil.outOfRange(gamepad2.right_stick_y * 100, -10, 10)) { // Checks if the gamepad2 (right stick y axis)
//                power = gamepad2.right_stick_y;
////                if (lift.getPos() < (this.lift.LIFT_RANGE - 120) && gamepad2.left_stick_y != 0)
////                    telemetry.addData("Lift Power", power);
////                else power = -0.1;
//                this.lift.setPower(power);
//                this.lift.setTargetPos(lift.getPos());
//            } else {
//                this.lift.setPower(0);
//                lift.goTo();
//            }

            telemetry.addData("y axis of right stick is activated", MathUtil.outOfRange(gamepad2.right_stick_y * 100, -10, 10));
            telemetry.addData("Lift Power", 0);
            telemetry.addData("current power (taken)", gamepad2.right_stick_y);

            telemetry.addData("lift pos : ", lift.right_elevator.getCurrentPosition());
            telemetry.addData("elevator busy", lift.right_elevator.isBusy());
            telemetry.update();
        }
    }

    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x);
    }
}
