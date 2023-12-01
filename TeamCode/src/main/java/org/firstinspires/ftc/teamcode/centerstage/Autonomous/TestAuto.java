package org.firstinspires.ftc.teamcode.centerstage.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.centerstage.Systems.DuckLine;

@Autonomous(group = "BLUE")
@Config
public class TestAuto extends LinearOpMode {
    public static DuckLine duckLine;
    @Override
    public void runOpMode() {
        AutoFlow auto = new AutoFlow(this, AutoFlow.Alliance.RED,
                AutoFlow.StartPos.BACKSTAGE, AutoFlow.Auto.FULL);

        auto.init();

        waitForStart();

        auto.test();
    }
}
