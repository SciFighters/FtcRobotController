package org.firstinspires.ftc.teamcode.power_play;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.power_play.util.DriveClass;
import org.firstinspires.ftc.teamcode.power_play.util.Lift;
import org.firstinspires.ftc.teamcode.power_play.util.Location;
import org.firstinspires.ftc.teamcode.power_play.util.Toggle;

@TeleOp
public class Constantin extends LinearOpMode {
    Lift lift = new Lift();
    DriveClass drive = new DriveClass(this, DriveClass.ROBOT.JACCOUSE, new Location(0, 0), DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE, DriveClass.DriveMode.BLUE);

    Toggle Level0 = new Toggle();
    Toggle Level1 = new Toggle();
    Toggle Level2 = new Toggle();
    Toggle Level3 = new Toggle();
    Toggle grabber = new Toggle();
    Toggle flipGrabber = new Toggle();
    Toggle rotateGrabber = new Toggle();


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
                continue;
            }
            if (gamepad2.start && gamepad2.x) {
                this.lift.setLiftState(Lift.LiftState.Idle);
                continue;
            }

            final double boostK = 0.5;
            double boost = gamepad1.right_trigger * boostK + (1 - boostK);
            double y = pow(-gamepad1.left_stick_y) * boost;
            double x = pow(gamepad1.left_stick_x) * boost;
            double turn = pow(gamepad1.right_stick_x * boost);
            drive.setPowerOriented(y, x, turn, true);

            lift.setLiftPower(-gamepad2.right_stick_y);

            Level0.update(gamepad2.a);
            Level1.update(gamepad2.b);
            Level2.update(gamepad2.x);
            Level3.update(gamepad2.y);
            grabber.update(gamepad2.right_bumper);
            flipGrabber.update(gamepad2.dpad_up || gamepad2.dpad_down);
            rotateGrabber.update(gamepad2.dpad_right || gamepad2.dpad_left);

            if (Level0.isClicked()) lift.gotoLevel(Lift.LiftLevel.Floor);
            if (Level1.isClicked()) lift.gotoLevel(Lift.LiftLevel.First);
            if (Level2.isClicked()) lift.gotoLevel(Lift.LiftLevel.Second);
            if (Level3.isClicked()) lift.gotoLevel(Lift.LiftLevel.Third);
            if (grabber.isClicked()) lift.grabber(grabber.getState());
            if (rotateGrabber.isClicked()) lift.rotate(rotateGrabber.getState());
            if (flipGrabber.isClicked()) lift.toggleFlip();
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

//            telemetry.addData("y axis of right stick is activated", MathUtil.outOfRange(gamepad2.right_stick_y * 100, -10, 10));
//            telemetry.addData("current power (taken)", gamepad2.right_stick_y);

            telemetry.addData("Flip Grabber", lift.flipMotor.getCurrentPosition());
            telemetry.addData("Left lift pos", lift.leftElevator.getCurrentPosition());
            telemetry.addData("Right lift pos", lift.rightElevator.getCurrentPosition());
            telemetry.addData("Left lift pow", lift.leftElevator.getPower());
            telemetry.addData("Right lift pow", lift.rightElevator.getPower());
            telemetry.addData("left elevator busy", lift.leftElevator.isBusy());
            telemetry.addData("right elevator busy", lift.rightElevator.isBusy());
            telemetry.addData("Lift State", lift.getState());
            telemetry.addData("Arm State", lift.getArmState());
            telemetry.update();
        }
    }

    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x);
    }
}
