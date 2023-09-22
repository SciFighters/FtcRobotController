package org.firstinspires.ftc.teamcode.power_play;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.power_play.util.DriveClass;
import org.firstinspires.ftc.teamcode.power_play.util.Lift;
import org.firstinspires.ftc.teamcode.power_play.util.Location;
import org.firstinspires.ftc.teamcode.power_play.util.Toggle;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class Constantin extends LinearOpMode {
    Lift lift = new Lift();
    DriveClass drive = new DriveClass(this, DriveClass.ROBOT.JACCOUSE, new Location(-0.9, 0.4404 / 2, 180), DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE, DriveClass.DriveMode.LEFT);
    double batteryLevel;
    double batteryPercentage = GetBatteryPercentage();

    //region Button Toggles
    Toggle Level0 = new Toggle();
    Toggle Level1 = new Toggle();
    Toggle Level2 = new Toggle();
    Toggle Level3 = new Toggle();
    Toggle grabber = new Toggle();
    Toggle flipGrabber = new Toggle();
    Toggle rotateGrabber = new Toggle();
    Toggle Level3_1 = new Toggle();
    Toggle cancelFlip = new Toggle();
    Toggle liftDescent = new Toggle();
    Toggle keepArmPos = new Toggle();
    private Toggle turningToggle = new Toggle();
    boolean keepingArmPos;
    //endregion
//    AprilTagInit aprilTagDetector = new AprilTagInit("webcam_rod");

    @Override
    public void runOpMode() {
//        OpenCvWebcam cam = CameraInitializer.initialize(this, "webcam_rod", 640, 360, null, false);
//        aprilTagDetector.run(this);
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Starting Initializing");
        telemetry.update();

        initHandling();

        lift.grabber(false); // Closes grabber
        telemetry.addLine("Finished Initializing");
        telemetry.update();

        waitForStart();
        int turningCount = 0;
        double targetHeading = drive.getHeading();

        drive.resetOrientation(0);

        while (opModeIsActive()) {
            //region Start Buttons
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
            //endregion

            //region Battery Variables
            batteryLevel = hardwareMap.voltageSensor.get("Control Hub").getVoltage();
            batteryPercentage = GetBatteryPercentage();
            //endregion

            //region Movement Variables
            final double boostK = 0.5;
            double boost = gamepad1.right_trigger * boostK + (1 - boostK);
            double y = pow(gamepad1.left_stick_y) * boost;
            double x = pow(-gamepad1.left_stick_x) * boost;
            double turn = pow(gamepad1.right_stick_x * boost);
            //endregion

            //region angle correction

            turningToggle.update(Math.abs(turn) > 0.02);

            if (turningToggle.isReleased()) turningCount = 8;
            if (!turningToggle.isPressed()) turningCount--;
            if (turningCount == 0) targetHeading = drive.getHeading();

            // angle correction close loop control
            if (!turningToggle.isPressed() && turningCount < 0) {
                double delta = drive.getDeltaHeading(targetHeading);
                double gain = 0.02;
                turn = delta * gain;
            }
            //endregion
            //region Driving Movement Controls
            if (gamepad1.dpad_left) drive.setPower(0, 0, 0.1);
            else if (gamepad1.dpad_right) drive.setPower(0, 0, -0.1);
            else if (gamepad1.dpad_up) drive.setPower(-0.1, 0, 0);
            else if (gamepad1.dpad_down) drive.setPower(0.1, 0, 0);
            else drive.setPowerOriented(y, x, turn, !gamepad1.start);
            if (gamepad1.y) {
                drive.zeroOnTargetOnes();
                targetHeading = drive.getHeading();
            }
            //endregion

            //region Toggle Updates
            keepArmPos.update(gamepad2.left_bumper);
            Level0.update(gamepad2.a);
            Level1.update(gamepad2.x);
            Level2.update(gamepad2.b);
            Level3.update(gamepad2.y);
            Level3_1.update(gamepad2.back);
            grabber.update(gamepad2.right_bumper);
            flipGrabber.update(gamepad2.dpad_up || gamepad2.dpad_down);
            rotateGrabber.update(gamepad2.dpad_right || gamepad2.dpad_left);
            cancelFlip.update(gamepad2.left_trigger < 0.4);
            //endregion

            //region Lift Controls
            Lift.LiftLevel setLiftLevel = null;

            if (Level0.isClicked()) {
                lift.autoLevelFunction(true, grabber, Lift.LiftLevel.Floor, cancelFlip.getState());
                setLiftLevel = Lift.LiftLevel.Floor;
                lift.grabber(false);
            }

            final int tickRaise = 275;

            if (Level1.isClicked()) {
                lift.autoLevelFunction(true, grabber, Lift.LiftLevel.First, cancelFlip.getState());
                setLiftLevel = Lift.LiftLevel.First;

            }
            if (Level2.isClicked()) {
                lift.autoLevelFunction(true, grabber, Lift.LiftLevel.Second, cancelFlip.getState());
                setLiftLevel = Lift.LiftLevel.Second;
            }
            if (Level3.isClicked()) {

                lift.autoLevelFunction(true, grabber, Lift.LiftLevel.Third, cancelFlip.getState());
                setLiftLevel = Lift.LiftLevel.Third;
            }
            if (Level3_1.isClicked())
                lift.gotoLevel(Lift.LiftLevel.ThirdFront, false, grabber, cancelFlip.getState());
            if (keepArmPos.isClicked()) {
                keepingArmPos = !keepingArmPos;
            }
            if (keepingArmPos) {
                lift.SaveArmPos();
            } else {
                lift.ResetArmPos();
            }

            if (grabber.isClicked()) lift.grabber(grabber.getState());
//            if (rotateGrabber.isClicked()) lift.rotate(rotateGrabber.getState());
//            if (flipGrabber.isClicked()) lift.toggleFlip(grabber);
            if (liftDescent.isClicked()) lift.gotoDescentLevel(grabber);

//            if (lift.leftElevator.getCurrentPosition() >= lift.MIN_HEIGHT_TO_ARM_MOVEMENT) {
//                if (lift.GetCameraServoPos() != 0) {
//                    lift.MoveCameraServo(true);
//                }
//            } else {
//                if (lift.GetCameraServoPos() != 1) {
//                    lift.MoveCameraServo(false);
//                }
//            }

            armControls();
            final float liftPower = -gamepad2.right_stick_y;
            lift.update();
            lift.runLift(liftPower, setLiftLevel);
            //endregion

            //region Telemetry
            drive.anglesTelemetry();
            telemetry.addData("Lift rot", lift.getArmPos());
            telemetry.addData("Left lift pos", lift.leftElevator.getCurrentPosition());
            telemetry.addData("Right lift pos", lift.rightElevator.getCurrentPosition());
            telemetry.addData("Left lift pow", lift.leftElevator.getPower());
            telemetry.addData("Right lift pow", lift.rightElevator.getPower());
            telemetry.addData("left elevator busy", lift.leftElevator.isBusy());
            telemetry.addData("right elevator busy", lift.rightElevator.isBusy());
            telemetry.addData("Lift State", lift.getState());
            telemetry.addData("Arm State ", lift.getArmState());
//            telemetry.addData("Flip Pos", lift.jointMotor.getCurrentPosition());
//            telemetry.addData("Flip Power", lift.jointMotor.getPower());
            telemetry.addData("Battery level", batteryLevel);
            telemetry.addData("Battery percentage", batteryLevel);
            telemetry.addData("Position x", drive.getPosX());
            telemetry.addData("Position y", drive.getPosY());
            telemetry.addData("Heading", drive.getHeading());
            telemetry.addData("Grabber position", lift.getGrabberPosition());
            telemetry.update();
            //endregion
        }
    }


    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x);
    }

    void armControls() {
        double armGotoPos = 0.5;
        if (!keepingArmPos) {
            armGotoPos = -gamepad2.left_stick_x / 2 + 0.5;
        } else {
            armGotoPos = lift.GetPrevArmPos();
        }
        lift.armGotoPosition(armGotoPos, true);
    }

    void initHandling() {
        lift.init(hardwareMap, this);
        drive.init(hardwareMap);
//        drive.initRodPipline(cam, null);
    }

    double GetBatteryPercentage() {
        return (batteryLevel - 11.7) / (12.6 - 11.7);
    }
}
