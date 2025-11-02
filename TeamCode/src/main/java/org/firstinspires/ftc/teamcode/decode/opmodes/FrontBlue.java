package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeBeltServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.OutputAngleServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter2;
import org.firstinspires.ftc.teamcode.decode.mechanisms.TrajectoryActions;

@Config
@Autonomous(name = "FrontBlue", group = "Autonomous")
public class FrontBlue extends LinearOpMode {

    @Override
    public void runOpMode(){
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        IntakeBeltServo intakeBeltServo = new IntakeBeltServo(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Shooter2 shooter2 = new Shooter2(hardwareMap);
        KickerServo kickerServo = new KickerServo(hardwareMap);
        OutputAngleServo outputAngleServo = new OutputAngleServo(hardwareMap);
        TrajectoryActions trajectoryActions = new TrajectoryActions();

        waitForStart();

//        Actions.runBlocking(
//            new SequentialAction(
//                    trajectoryActions.getTrajectory_1_1(),
//
//                    shooter.startShooter(1.0),
//
////                    trajectoryActions.getTrajectory_1_2(),
//
//                    intakeMotor.startIntake(),
//                    intakeBeltServo.startIntakeBeltServo(),
//
////                    trajectoryActions.getTrajectory_1_3()
//            )
//        );



    }
}
