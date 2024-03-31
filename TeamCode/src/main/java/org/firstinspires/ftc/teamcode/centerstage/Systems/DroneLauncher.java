package org.firstinspires.ftc.teamcode.centerstage.Systems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Component;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.ThreadedComponent;

@ThreadedComponent
public class DroneLauncher extends Component {
    Servo droneLauncherServo;

    @Override
    public void init() {
        droneLauncherServo = hardwareMap.get(Servo.class, "droneLauncherServo");
        droneLauncherServo.setPosition(0.5);
    }

    public void launch() {
        droneLauncherServo.setPosition(1);
    }
}
