package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decode.helper.Constants;
import org.firstinspires.ftc.teamcode.decode.helper.Utility;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeBeltMotor;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.OutputAngleServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.decode.mechanisms.TrajectoryActions;

@Config
@Autonomous(name = "Front Red - Position 1", group = "DecodeAutonomous")
public class FrontRed_1 extends LinearOpMode {

    @Override
    public synchronized void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        IntakeBeltMotor intakeBeltMotor = new IntakeBeltMotor(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        KickerServo kickerServo = new KickerServo(hardwareMap);
        OutputAngleServo outputAngleServo = new OutputAngleServo(hardwareMap);

        TrajectoryActions trajectoryActions = new TrajectoryActions();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        outputAngleServo.setOutputAngle(Constants.RED_LAUNCH_LOCATION_1_AUTON),
                        shooter.startShooter(Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_1_AUTON)
                )
        );

        wait(1000);

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActions.getTrajectory_1_1(drive, 0, 0, -90, 0, 7, -68, true)
                )
        );

        Utility.shoot(this, outputAngleServo, shooter, intakeMotor, intakeBeltMotor, kickerServo, Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_1_AUTON, Constants.RED_LAUNCH_LOCATION_1_AUTON);

        if (Constants.PICK_FROM_HUMAN_AREA_FLAG_1) {

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActions.getTrajectory_1_1(drive, -1, -1, -1, -33, -4, 180, false)
                    )
            );

            double speed = 0.5;
            Utility.drive(drive, -speed, speed, speed, -speed); // Left strafe
            wait(400);

            Utility.autonIntakeHumanArea(this, shooter, intakeMotor, intakeBeltMotor, kickerServo, drive, Constants.HUMAN_AREA_INTAKE_WAIT_TIME);

            Actions.runBlocking(
                    new SequentialAction(
                            outputAngleServo.setOutputAngle(Constants.RED_LAUNCH_LOCATION_1_AUTON),
                            shooter.startShooter(Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_1_AUTON)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActions.getTrajectory_1_1(drive, -1, -1, -1, 0, 7, -70, false)
                    )
            );

            Utility.shoot(this, outputAngleServo, shooter, intakeMotor, intakeBeltMotor, kickerServo, Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_1_AUTON, Constants.RED_LAUNCH_LOCATION_1_AUTON);
        }

        if (Constants.PICK_BALLS_LINE_3_FLAG) {
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActions.getTrajectory_1_1(drive, -1, -1, -1, -8, 15, 186, false)
                    )
            );

            Utility.autonIntake(this, shooter, intakeMotor, intakeBeltMotor, kickerServo, drive, Constants.LINE_3_INTAKE_WAIT_TIME);

            Actions.runBlocking(
                    new SequentialAction(
                            outputAngleServo.setOutputAngle(Constants.RED_LAUNCH_LOCATION_1_AUTON),
                            shooter.startShooter(Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_1_AUTON)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActions.getTrajectory_1_1(drive, -1, -1, -1, 0, 7, -35, false)
                    )
            );
//            wait(2000);
//            Actions.runBlocking(
//                    new SequentialAction(
//                            trajectoryActions.turn(drive, -70)
//                    )
//            );

            Utility.shoot(this, outputAngleServo, shooter, intakeMotor, intakeBeltMotor, kickerServo, Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_1_AUTON, Constants.RED_LAUNCH_LOCATION_1_AUTON);
        }

        if (Constants.PICK_FROM_HUMAN_AREA_FLAG_2) {

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActions.getTrajectory_1_1(drive, -1, -1, -1, -28, -4, 180, false)
                    )
            );

            double speed = 0.5;
            Utility.drive(drive, -speed, speed, speed, -speed); // Left strafe
            wait(400);

            Utility.autonIntakeHumanArea(this, shooter, intakeMotor, intakeBeltMotor, kickerServo, drive, Constants.HUMAN_AREA_INTAKE_WAIT_TIME);

            Actions.runBlocking(
                    new ParallelAction(
                            shooter.reverseShooter(),
                            intakeBeltMotor.reverseIntakeBeltMotor()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActions.getTrajectory_1_1(drive, -1, -1, -1, 0, 9, -70, false)
                    )
            );
            Actions.runBlocking(
                    new SequentialAction(
                            outputAngleServo.setOutputAngle(Constants.RED_LAUNCH_LOCATION_1_AUTON),
                            shooter.startShooter(Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_1_AUTON)
                    )
            );
            wait(1000);

            Utility.shoot(this, outputAngleServo, shooter, intakeMotor, intakeBeltMotor, kickerServo, Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_1_AUTON, Constants.RED_LAUNCH_LOCATION_1_AUTON);
        }

        if (Constants.PICK_FROM_HUMAN_AREA_FLAG_3) {

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActions.getTrajectory_1_1(drive, -1, -1, -1, -33, -4, 180, false)
                    )
            );

            double speed = 0.5;
            Utility.drive(drive, -speed, speed, speed, -speed); // Left strafe
            wait(400);

            Utility.autonIntakeHumanArea(this, shooter, intakeMotor, intakeBeltMotor, kickerServo, drive, Constants.HUMAN_AREA_INTAKE_WAIT_TIME);

//            Actions.runBlocking(
//                    new SequentialAction(
//                            outputAngleServo.setOutputAngle(Constants.RED_LAUNCH_LOCATION_1_AUTON),
//                            shooter.startShooter(Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_1_AUTON)
//                    )
//            );

//            Actions.runBlocking(
//                    new SequentialAction(
//                            trajectoryActions.getTrajectory_1_1(drive, -1, -1, -1, 0, 7, -70, false)
//                    )
//            );

//            Actions.runBlocking(
//                    new SequentialAction(
//                            outputAngleServo.setOutputAngle(Constants.RED_LAUNCH_LOCATION_1_AUTON),
//                            shooter.startShooter(Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_1_AUTON)
//                    )
//            );

//            wait(1000);
//
//            Utility.shoot(this, outputAngleServo, shooter, intakeMotor, intakeBeltMotor, kickerServo, Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_1_AUTON, Constants.RED_LAUNCH_LOCATION_1_AUTON);
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActions.getTrajectory_1_1(drive, -1, -1, -1, -8, 4, 180, false)
                )
        );

    }

}
