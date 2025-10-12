package org.firstinspires.ftc.teamcode.decode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Drive", group = "Autonomous")
public class Drive  extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive =new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                .lineToYSplineHeading(33, Math.toRadians(0))
//                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(10)
                .setTangent(Math.toRadians(0))
                .lineToX(10)
                .strafeTo(new Vector2d(10.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(10.5)
                .waitSeconds(3);

        telemetry.addData("Ready to Start", 123 );
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(tab1.build())

                );



    }

}
