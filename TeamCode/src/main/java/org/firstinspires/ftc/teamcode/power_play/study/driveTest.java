package org.firstinspires.ftc.teamcode.power_play.study;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.power_play.util.DriveClass;
import org.firstinspires.ftc.teamcode.power_play.util.Location;

public class driveTest extends LinearOpMode {
    private final double tile = 0.6;

    DriveClass drive;
    private ElapsedTime runtime;




    public void initiate(HardwareMap hw) {
        drive = new DriveClass(this, DriveClass.ROBOT.JACCOUSE, new Location(0,0), org.firstinspires.ftc.teamcode.freight_frenzy.util.DriveClass.USE_BRAKE | org.firstinspires.ftc.teamcode.freight_frenzy.util.DriveClass.USE_DASHBOARD_FIELD,org.firstinspires.ftc.teamcode.power_play.util.DriveClass.DriveMode.BLUE);
        runtime = new ElapsedTime();
        drive.init(hw);
    }

    @Override
    public void runOpMode() {
        initiate(hardwareMap);

        waitForStart();

        drive.goToLocation(new Location(2,2,30), 0.8, 0.1, 0.5);

        
    }
}
