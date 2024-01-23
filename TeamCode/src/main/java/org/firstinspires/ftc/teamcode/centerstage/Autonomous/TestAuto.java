package org.firstinspires.ftc.teamcode.centerstage.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.DuckLine;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;

@Autonomous(group = "BLUE")
@Config
public class TestAuto extends Robot {
    public static DuckLine duckLine;
    AutoFlow auto = new AutoFlow(this, AutoFlow.Alliance.BLUE,
            AutoFlow.StartPos.PIXEL_STACK, AutoFlow.Auto.FULL);

    @Override
    public void updateLoop() {

    }

    @Override
    public void initRobot() {
        auto.init();
    }

    @Override
    public void startRobot() {
        auto.test();
    }
}
