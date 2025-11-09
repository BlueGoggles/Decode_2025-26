package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decode.helper.Utility;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeBeltServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerStopperServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.OutputAngleServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter2;
import org.firstinspires.ftc.teamcode.decode.mechanisms.TrajectoryActions;

@Config
@Autonomous(name = "FrontRed", group = "DecodeAutonomous")
public class FrontRed extends LinearOpMode {

    @Override
    public synchronized void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        IntakeBeltServo intakeBeltServo = new IntakeBeltServo(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Shooter2 shooter2 = new Shooter2(hardwareMap);
        KickerServo kickerServo = new KickerServo(hardwareMap);
        OutputAngleServo outputAngleServo = new OutputAngleServo(hardwareMap);
        KickerStopperServo kickerStopperServo = new KickerStopperServo(hardwareMap);

        TrajectoryActions trajectoryActions = new TrajectoryActions();

        waitForStart();

        Actions.runBlocking(
            new SequentialAction(
                    trajectoryActions.getTrajectory_1_1(drive, true)
            )
        );

        Utility.shoot(this, outputAngleServo, shooter, intakeMotor, intakeBeltServo, kickerServo);

        telemetry.addData("Pose After 1_1: ", drive.localizer.getPose());
        telemetry.update();

        Actions.runBlocking(
                new SequentialAction(

                        trajectoryActions.getTrajectory_1_2(drive)
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        kickerServo.stopKickerServo(),
                        shooter.stopShooter(),
                        intakeMotor.startIntake(),
                        intakeBeltServo.startIntakeBeltServo()
                )
        );
        telemetry.addData("Pose After 1_2: ", drive.localizer.getPose());
        telemetry.update();


        drive.leftFront.setPower(0.15);
        drive.rightFront.setPower(0.15);
        drive.leftBack.setPower(0.15);
        drive.rightBack.setPower(0.15);

        wait(3500);
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActions.getTrajectory_1_1(drive, false)
                )
        );

        Utility.shoot(this, outputAngleServo, shooter, intakeMotor, intakeBeltServo, kickerServo);
    }
}
