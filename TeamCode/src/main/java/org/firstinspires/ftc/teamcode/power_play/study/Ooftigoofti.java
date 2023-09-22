package org.firstinspires.ftc.teamcode.power_play.study;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.power_play.util.Toggle;

@TeleOp(name = "Oofti goofti", group = "Linear Opmode")
public class Ooftigoofti extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    double leftPower;
    double rightPower;
    double fixLeft, fixRight;
    Toggle fixLeftToggle, fixLeftReduceToggle, fixRightToggle, fixRightReduceToggle, revMode, joyStickModeToggle;
    double maxFix = 0.1;
    double turnFactor = 0.35;
    boolean revModeActive;
    boolean joystickMode;

    @Override
    public void runOpMode() {
        fixLeftToggle = new Toggle();
        fixRightToggle = new Toggle();
        fixLeftReduceToggle = new Toggle();
        fixRightReduceToggle = new Toggle();
        revMode = new Toggle();
        joyStickModeToggle = new Toggle();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive = hardwareMap.get(DcMotor.class, "fl");
        rightDrive = hardwareMap.get(DcMotor.class, "fr");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior to BRAKE for both motors
        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        double normalSpeed = 0.27;
        double boostK = 0.7;

        while (opModeIsActive()) {
            fixLeftToggle.update(gamepad1.dpad_left);
            fixRightReduceToggle.update(gamepad1.dpad_right);
            fixLeftReduceToggle.update(gamepad1.dpad_down);
            fixRightToggle.update(gamepad1.dpad_up);
            revMode.update(gamepad1.circle);
            joyStickModeToggle.update(gamepad1.triangle);

            if (fixRightReduceToggle.isClicked()) fixRight -= 0.1;
            if (fixLeftToggle.isClicked()) fixLeft += 0.1;
            if (fixLeftReduceToggle.isClicked()) fixLeft -= 0.1;
            if (fixRightToggle.isClicked()) fixRight += 0.1;
            if (revMode.isClicked()) revModeActive = !revModeActive;
            if (joyStickModeToggle.isClicked()) joystickMode = !joystickMode;
            // Apply reasonable limits to fixLeft and fixRight
            fixLeft = Range.clip(fixLeft, -maxFix, maxFix);
            fixRight = Range.clip(fixRight, -maxFix, maxFix);

            leftPower = 0;
            rightPower = 0;

            double activeBoostPower = normalSpeed;
            if (gamepad1.right_trigger > 0.1 ^ gamepad1.left_trigger > 0.1) {
                activeBoostPower = boostK;
            } else if (gamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1) {
                activeBoostPower = 1;
            }
            double drive, turn;
            drive = (revModeActive ? 1 : -1) * gamepad1.left_stick_y * activeBoostPower;
            if (!joystickMode) {
                // Normal controls
                turn = gamepad1.right_stick_x * turnFactor;
            } else {
                turn = gamepad1.left_stick_x * turnFactor;
            }
            setPower(drive, turn);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors\n-----------------------\n",
                    "left: (), right: ()\n turn: ()\n-----------------------",
                    leftPower, rightPower);
            telemetry.addData("Fixes\n-----------------------\n", "left : (), right : ()",
                    fixLeft, fixRight);
            telemetry.update();
        }
    }

    void setPower(double drive, double turn) {
        // Apply reasonable limits to fixLeft and fixRight
        fixLeft = Range.clip(fixLeft, -maxFix, maxFix);
        fixRight = Range.clip(fixRight, -maxFix, maxFix);

        // Adjust signs of fix values based on the direction of turn
        double leftFix = (drive > 0.01) ? fixLeft : -fixLeft;
        double rightFix = (drive > 0.01) ? fixRight : -fixRight;

        leftPower = Range.clip(drive + leftFix + turn, -1.0, 1.0);
        rightPower = Range.clip(drive + rightFix - turn, -1.0, 1.0);

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
}