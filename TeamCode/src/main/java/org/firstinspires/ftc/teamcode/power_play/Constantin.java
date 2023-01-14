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
    DriveClass drive = new DriveClass(this, DriveClass.ROBOT.JACCOUSE, new Location(0, 0), DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE, DriveClass.DriveMode.LEFT);
    double batteryLevel;
    double batteryPercentage = (batteryLevel - 11.7) / (12.6 - 11.7);
    //region button toggles
    Toggle Level0 = new Toggle();
    Toggle Level1 = new Toggle();
    Toggle Level2 = new Toggle();
    Toggle Level3 = new Toggle();
    Toggle grabber = new Toggle();
    Toggle flipGrabber = new Toggle();
    Toggle rotateGrabber = new Toggle();
    Toggle Level3_1 = new Toggle();
    private Toggle turningToggle = new Toggle();
    //endregion

    @Override
    public void runOpMode() {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Starting Initializing");
        telemetry.update();
        drive.init(hardwareMap);
        lift.init(hardwareMap);

        lift.grabber(true); // Closes grabber
        telemetry.addLine("Finished Initializing");
        telemetry.update();

        waitForStart();

        int turningCount = 0;
        double targetHeading = 0;

        drive.resetOrientation(0);
        while (opModeIsActive()) {
            batteryLevel = hardwareMap.voltageSensor.get("Control Hub").getVoltage();
            batteryPercentage = (batteryLevel - 11.7) / (12.6 - 11.7);
            if (gamepad1.start) {
                if (gamepad1.x) {
                    drive.resetOrientation(0);
                    targetHeading = drive.getHeading();
                }
                continue;
            }
            if (gamepad2.start) {
                if (gamepad2.x) {
                    this.lift.setLiftState(Lift.LiftState.Idle);
                }
                continue;
            }

            final double boostK = 0.5;
            double boost = gamepad1.right_trigger * boostK + (1 - boostK);
            double y = pow(-gamepad1.left_stick_y) * boost;
            double x = pow(gamepad1.left_stick_x) * boost;
            double turn = pow(gamepad1.right_stick_x * boost);

            //region angle correction

            turningToggle.update(Math.abs(turn) > 0.02);

            if (turningToggle.isReleased()) turningCount = 8;
            if (!turningToggle.isPressed()) turningCount--;
            if (turningCount == 0) targetHeading = drive.getHeading();

            if (!turningToggle.isPressed() && turningCount < 0) {
                double delta = drive.getDeltaHeading(targetHeading);
                double gain = 0.02;
                turn = delta * gain;
            }

            //endregion

            if (gamepad1.dpad_left) x = -0.3;
            if (gamepad1.dpad_right) x = 0.3;
            drive.setPowerOriented(y, x, turn, true);

            lift.setLiftPower(-gamepad2.right_stick_y);

            if (gamepad1.back) {
                drive.hoverBoardMode();
            }
            Level0.update(gamepad2.a);
            Level1.update(gamepad2.x);
            Level2.update(gamepad2.b);
            Level3.update(gamepad2.y);
            Level3_1.update(gamepad2.back);
            grabber.update(gamepad2.right_bumper);
            flipGrabber.update(gamepad2.dpad_up || gamepad2.dpad_down);
            rotateGrabber.update(gamepad2.dpad_right || gamepad2.dpad_left);

            if (Level0.isClicked()) {
                lift.gotoLevel(Lift.LiftLevel.Floor, true, grabber);
            }
            if (Level1.isClicked()) {
                lift.gotoLevel(Lift.LiftLevel.First, true, grabber);
            }
            if (Level2.isClicked()) lift.gotoLevel(Lift.LiftLevel.Second, true, grabber);
            if (Level3.isClicked()) lift.gotoLevel(Lift.LiftLevel.Third, true, grabber);
            if (Level3_1.isClicked()) lift.gotoLevel(Lift.LiftLevel.ThirdFront, false, grabber);
            if (grabber.isClicked()) lift.grabber(grabber.getState());
            if (rotateGrabber.isClicked()) lift.rotate(rotateGrabber.getState());
            if (flipGrabber.isClicked()) lift.toggleFlip(grabber);
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

            drive.anglesTelemetry();

            telemetry.addData("Flip Grabber", lift.jointMotor.getCurrentPosition());
            telemetry.addData("Left lift pos", lift.leftElevator.getCurrentPosition());
            telemetry.addData("Right lift pos", lift.rightElevator.getCurrentPosition());
            telemetry.addData("Left lift pow", lift.leftElevator.getPower());
            telemetry.addData("Right lift pow", lift.rightElevator.getPower());
            telemetry.addData("left elevator busy", lift.leftElevator.isBusy());
            telemetry.addData("right elevator busy", lift.rightElevator.isBusy());
            telemetry.addData("Lift State", lift.getState());
            telemetry.addData("Arm State ", lift.getArmState());
            telemetry.addData("Battery level", batteryLevel);
            telemetry.addData("Battery percentage", batteryLevel);
            telemetry.addData("Position x", drive.getPosX());
            telemetry.addData("Position y", drive.getPosY());
            telemetry.addData("Heading", drive.getHeading());
            telemetry.update();
        }
    }

    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x);
    }
}
