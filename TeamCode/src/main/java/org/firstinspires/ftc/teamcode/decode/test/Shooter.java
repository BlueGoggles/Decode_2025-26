package org.firstinspires.ftc.teamcode.decode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "The Shooter", group = "BackstageRedAuton")
public class Shooter extends LinearOpMode {

    protected RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        Utility.initializeRobot(robot);
        waitForStart();

        robot.getLeftShooter().setPower(1);
        robot.getRightShooter().setPower(1);

        sleep(20000);


    }


}