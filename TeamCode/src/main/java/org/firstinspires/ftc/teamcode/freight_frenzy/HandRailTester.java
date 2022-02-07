package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.HandRailClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.Location;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.Toggle;

@TeleOp
@Disabled
public class HandRailTester extends LinearOpMode {

    final double tile = 0.6;

    Location startingPoisition = new Location(0 * tile, 0 * tile);
    //    private DriveClass robot = new DriveClass(this, DriveClass.ROBOT.BIGFOOT, startingPoisition);
    private HandRailClass handRail = new HandRailClass(this);
    double targetHeading = 0;
    private HardwareMap HardwareMap;

    private Toggle collector = new Toggle(); //  Collection toggle (A button)
    boolean release; // releasing object (B button)
    private Toggle homing = new Toggle();

    @Override
    public void runOpMode() {

//        robot.init(hardwareMap);
        handRail.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            double railPower = -gamepad1.left_stick_y;
            double armPower = -gamepad1.right_stick_y;

            handRail.rail_drive(railPower, true);
            handRail.hand_drive(armPower, true);

            collector.update(gamepad1.a || gamepad2.a); // update toggle (A button)
            homing.update(gamepad1.x || gamepad2.x);
            release = gamepad1.b || gamepad2.b;

            if (!release) {
                if (collector.getState() && !handRail.grabber_ts()) { //collection toggle
                    handRail.grabberGrab();
                } else {
                    handRail.grabberStop();
                }
            }
            else {
                collector.set(false);
                handRail.grabberRelease();
            }
            if(homing.isClicked()) {
                handRail.searchHome();
            }

            this.handRail.telemetry_handRail(); //telemetry

        }
    }
}
