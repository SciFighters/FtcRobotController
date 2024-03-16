package org.firstinspires.ftc.teamcode.centerstage.OpModeTesters;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;


@TeleOp(group = "TESTER")
public class BoardAlignTester extends Robot {
    AutoFlow autoFlow = new AutoFlow(this, AutoFlow.Alliance.BLUE, AutoFlow.StartPos.BACKSTAGE, AutoFlow.Auto.FULL, AutoFlow.ParkLocation.FAR);

    @Override
    public void initRobot() {
        autoFlow.init();
        autoFlow.stopPropDetection();
        autoFlow.startAprilTagDetection();
    }

    @Override
    public void startRobot() {
        autoFlow.drive.resetOrientation(90);
    }

    @Override
    public void updateLoop() {
        if (gamepad1.a) {
            autoFlow.goToBoardByProp(AutoFlow.PropPos.LEFT, Arm.Position.One);
        }
    }
}