package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.LyonLib.utils.GamepadExEx;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;


@TeleOp(group = "MainOpMode")
public class MainTeleop extends OpMode{
    RobotContainer robotContainer;

    @Override
    public void init()
    {
        robotContainer = new RobotContainer(telemetry,
                new GamepadExEx(gamepad1), new GamepadExEx(gamepad2), RobotContainer.RobotMode.TELEOP_RED);

        robotContainer.initializeSubsystems(hardwareMap);
        robotContainer.init();
    }

    @Override
    public void start()
    {
        robotContainer.start();
    }

    @Override
    public void loop()
    {
        robotContainer.periodic();
    }

    @Override
    public void stop()
    {
        robotContainer.end();
    }
}
