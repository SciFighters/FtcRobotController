package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class GandalfsBeard extends LinearOpMode {
    private KiwiDrive kiwiDrive;
    private DcMotor wheel1;
    private DcMotor wheel2;
    private DcMotor wheel3;
    private DcMotor conveyorMotor;
    public IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        wheel1 = hardwareMap.get(DcMotor.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotor.class, "wheel2");
        wheel3 = hardwareMap.get(DcMotor.class, "wheel3");
        conveyorMotor = hardwareMap.get(DcMotor.class, "conveyorMotor");
        imu = hardwareMap.get(IMU.class, "imu");
        //alter when robot is built
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT
                        ,RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD))
        );
        kiwiDrive = new KiwiDrive(wheel1,wheel2,wheel3,imu);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            kiwiDrive.setBoost(gamepad1.left_bumper,gamepad1.right_bumper);
            //dpad driving
            if (gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up || gamepad1.dpad_down)
            {
                kiwiDrive.dpadMovement(gamepad1.dpad_down, gamepad1.dpad_up, gamepad1.dpad_left,
                        gamepad1.dpad_right, gamepad1.right_stick_x);
            }
            else {
                kiwiDrive.drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);

            }
            conveyorMotor.setPower(Range.clip((gamepad1.right_trigger - gamepad1.left_trigger) * 0.2,1,-1));

        }
    }

}