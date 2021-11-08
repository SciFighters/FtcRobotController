package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.DriveClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.HandRailClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.Location;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.Toggle;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    final double tile = 0.6;

    Location startingPoisition = new Location(0 * tile, 0 * tile);
    //    private DriveClass robot = new DriveClass(this, DriveClass.ROBOT.BIGFOOT, startingPoisition);
    private HandRailClass arm = new HandRailClass(this);
    double targetHeading = 0;
    private HardwareMap HardwareMap;

    private Toggle collector = new Toggle(); //  Collection toggle (A button)
    boolean release; // releasing object (B button)

    @Override
    public void runOpMode() {

//        robot.init(hardwareMap);
        arm.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

//                robot.resetPosition();
//                targetHeading = robot.getHeading();


            collector.update(gamepad1.a || gamepad2.a); // update toggle (A button)
            release = gamepad1.b || gamepad2.b;

            if (!collector.getState() && arm.grabber_ts()) { //collection toggle
                arm.grabberGrab();
            } else if (release) {
                arm.grabberRelease();
            } else {
                arm.grabberStop();
            }//אפשר הסבר למה איתי אומר ששינתם הרבה, כיוון שהדבר היחיד ששיניתם בקוד שלי זה ההוספה של RELEASE, והורדתם את ה! לפני arm.grabber_ts
        }

    }
}
