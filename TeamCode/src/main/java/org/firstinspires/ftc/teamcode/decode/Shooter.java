package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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