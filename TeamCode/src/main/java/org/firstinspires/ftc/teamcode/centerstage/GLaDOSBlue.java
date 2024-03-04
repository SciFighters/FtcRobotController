package org.firstinspires.ftc.teamcode.centerstage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.Systems.RobotControl;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;


@TeleOp(group = "CENTERSTAGE BLUE", name = "GLaDOS : Blue Alliance")
@Config
public class GLaDOSBlue extends Robot {
    public static double kY = 0.03;
    public static double avgMaxVelocity = 2700;
    public static double minDist = 70;
    public static double avgMaxVelocityGain = 45;

    @Override
    public void initRobot() {
        addComponent(RobotControl.class);
        getComponent(RobotControl.class).setAlliance(AutoFlow.Alliance.BLUE);
    }

    @Override
    public void startRobot() {
    }
}