package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.Systems.RobotControl;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.RobotTelemetry;


@TeleOp(group = "CENTERSTAGE BLUE", name = "GLaDOS : Blue Alliance")
public class GLaDOSBlue extends Robot {
    @Override
    public void initRobot() {
        addComponent(RobotControl.class);
    }

    @Override
    public void startRobot() {
        getComponent(RobotControl.class).start();
        getComponent(RobotControl.class).setAlliance(AutoFlow.Alliance.BLUE);
    }

    @Override
    public void updateLoop() {
        return;
    }
}