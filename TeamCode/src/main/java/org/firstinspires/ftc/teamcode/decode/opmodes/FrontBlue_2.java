package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decode.helper.Constants;
import org.firstinspires.ftc.teamcode.decode.helper.Utility;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeBeltServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerStopperServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.OutputAngleServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.decode.mechanisms.TrajectoryActions;

@Config
@Autonomous(name = "Front Blue - Position 2", group = "DecodeAutonomous")
public class FrontBlue_2 extends LinearOpMode {

    @Override
    public synchronized void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        IntakeBeltServo intakeBeltServo = new IntakeBeltServo(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        KickerServo kickerServo = new KickerServo(hardwareMap);
        OutputAngleServo outputAngleServo = new OutputAngleServo(hardwareMap);
        KickerStopperServo kickerStopperServo = new KickerStopperServo(hardwareMap);

        TrajectoryActions trajectoryActions = new TrajectoryActions();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        outputAngleServo.setOutputAngle(Constants.BLUE_LAUNCH_LOCATION_2),
                        shooter.startShooter(Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_2)
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActions.getTrajectory_1_1(drive, 0, 0, 180, -7, 73, -52, true)
                )
        );

        Utility.shoot(this, outputAngleServo, shooter, intakeMotor, intakeBeltServo, kickerServo, Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_2, Constants.BLUE_LAUNCH_LOCATION_2);

        telemetry.addData("Pose After 1_1: ", drive.localizer.getPose());
        telemetry.update();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActions.getTrajectory_1_1(drive, -1, -1, -1, -8, 62,180, false)
                )
        );
        Utility.autonIntake(shooter, intakeMotor, intakeBeltServo, kickerServo);

        telemetry.addData("Pose After 1_2: ", drive.localizer.getPose());
        telemetry.update();

        Utility.drive(drive, 0.07);

        wait(4000);

        Actions.runBlocking(
                new SequentialAction(
                        intakeBeltServo.stopIntakeBeltServo()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActions.getTrajectory_1_1(drive, -1, -1, -1, -11, 73, -62, false)
                )
        );

        Utility.shoot(this, outputAngleServo, shooter, intakeMotor, intakeBeltServo, kickerServo, Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_2, Constants.BLUE_LAUNCH_LOCATION_2);

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActions.getTrajectory_1_1(drive, -1, -1, -1, -8, 25,180, false)
                )
        );
    }
}
