package org.firstinspires.ftc.teamcode.freight_frenzy.util;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.ultimate_goal.util.DriveClass;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.Location;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.Toggle;



@TeleOp(group = "JaccouseTest")
public class JaccouseTest extends LinearOpMode {
    final double tile = 0.6;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Location startingPosition = new Location(0,0.225); //last x = -1.75*tile, y = 0*tile
    private DriveClass drive = new DriveClass(this, DriveClass.ROBOT.COBALT, startingPosition).useEncoders().useBrake();
    org.firstinspires.ftc.teamcode.freight_frenzy.util.Location startingPoisition = new org.firstinspires.ftc.teamcode.freight_frenzy.util.Location(0 * tile, 0 * tile);


    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;

    private Toggle A = new Toggle();
    private Toggle B = new Toggle();
    private Toggle Y = new Toggle();
    private Toggle X = new Toggle();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive.init(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        drive.resetOrientation(0); //default blue

        runtime.reset();

        int turningCount = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            A.update(gamepad1.a);
            B.update(gamepad1.b);
            Y.update(gamepad1.x);
            X.update(gamepad1.y);

            if (A.isClicked()) {
                drive.goTo(0, 0.225, 0.9, 0, 0.05);

            } else if (B.isClicked()) {
                drive.goTo(1.2, 1.2, 0.9, 0, 0.05);

            } else if (Y.isClicked()) {
                drive.goTo(6, 2.4, 0.9, 0, 0.05);
                drive.turn(45,0.9);

            } else if (X.isClicked()){
                drive.goTo(-1.2,1.2,0.9,0,0.05);
                drive.turn(45,0.9);
            }

        }

    }
}
