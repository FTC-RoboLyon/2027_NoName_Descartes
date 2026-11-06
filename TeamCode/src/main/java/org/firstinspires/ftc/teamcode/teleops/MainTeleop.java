package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;


@TeleOp(group = "MainOpMode")
public class MainTeleop extends OpMode{
    RobotContainer robotContainer;

    @Override
    public void init()
    {
        robotContainer = new RobotContainer(telemetry,
                new GamepadEx(gamepad1), new GamepadEx(gamepad2), RobotContainer.RobotMode.TELEOP_RED);

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
