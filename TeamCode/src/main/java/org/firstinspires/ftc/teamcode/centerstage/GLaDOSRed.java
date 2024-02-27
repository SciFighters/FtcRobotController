package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.Systems.RobotControl;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;


@TeleOp(group = "CENTERSTAGE RED", name = "GLaDOS : RED Alliance")
public class GLaDOSRed extends Robot {
    @Override
    public void initRobot() {
        addComponent(RobotControl.class);
    }

    @Override
    public void startRobot() {
        getComponent(RobotControl.class).start();
        getComponent(RobotControl.class).setAlliance(AutoFlow.Alliance.RED);
    }

    @Override
    public void updateLoop() {
        return;
    }
}