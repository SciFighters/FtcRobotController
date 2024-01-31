package org.firstinspires.ftc.teamcode.centerstage.Autonomous.Paths.RedAlliance.Backdrop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.DuckLine;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;

@Autonomous(group = "RED", name = "Red: Backdrop - Full")
@Config
public class RedBackdropkFull extends Robot {
    public static DuckLine duckLine;
    AutoFlow auto = new AutoFlow(this, AutoFlow.Alliance.RED,
            AutoFlow.StartPos.BACKSTAGE, AutoFlow.Auto.FULL);

    @Override
    public void initRobot() {
        auto.init();
    }

    @Override
    public void startRobot() {
        auto.backdropSide();
    }
}
