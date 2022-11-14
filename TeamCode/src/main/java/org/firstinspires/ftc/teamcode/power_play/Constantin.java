package org.firstinspires.ftc.teamcode.power_play;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.power_play.util.DriveClass;
import org.firstinspires.ftc.teamcode.power_play.util.Lift;
import org.firstinspires.ftc.teamcode.power_play.util.Location;

@TeleOp
public class Constantin extends LinearOpMode {
    private DcMotorEx fl = null;
    private DcMotorEx fr = null;
    private DcMotorEx bl = null;
    private DcMotorEx br = null;

    private Lift lift = new Lift();

    String state = "WITHOUT ENCODER";
    DriveClass drive = new DriveClass(this, DriveClass.ROBOT.JACCOUSE, new Location(0, 0), DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE, DriveClass.DriveMode.BLUE);

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.FORWARD);
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.FORWARD);

        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Starting Initializing");
        telemetry.update();

        drive.init(hardwareMap);

        telemetry.addLine("Finished Initializing");
        telemetry.update();

        waitForStart();

        drive.resetOrientation(90);

        fl.setPower(gamepad1.left_stick_x);
        fr.setPower(gamepad1.right_stick_x);

        bl.setPower(-gamepad1.left_stick_y);
        br.setPower(-gamepad1.right_stick_y);
        if (gamepad1.a) {
            fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            state = "USING ENCODER";
        }
        if (gamepad1.b) {
            fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            state = "WITHOUT ENCODER";
        }
        if (gamepad1.right_stick_y * 100 >= 10 || gamepad1.right_stick_y * 100 <= -10) {
            double power = gamepad1.right_stick_y;
            this.lift.setPower(power);
            telemetry.addData("Lift Power", power);
        } else {
            this.lift.setPower(0);
            telemetry.addData("Lift Power", 0);
        }


        telemetry.addData("front left:", fl.getCurrentPosition());
        telemetry.addData("front right:", fr.getCurrentPosition());
        telemetry.addData("back left:", bl.getCurrentPosition());
        telemetry.addData("back right:", br.getCurrentPosition());
        telemetry.addData("state: ", state);
        telemetry.update();
    }





    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x); // x^2
    }
}
