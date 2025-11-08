package org.firstinspires.ftc.teamcode.decode.test;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
        import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter;

@Config
@Autonomous(name = "First Test Auton", group = "Autonomous")
@Disabled
public class FirstAuton  extends LinearOpMode {

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        // make a Shooter instance
        Shooter shooter = new Shooter(hardwareMap);

        // make a IntakeBeltServo instance
//        IntakeBeltServo servo = new IntakeBeltServo(hardwareMap);

        // actionBuilder builds from the drive steps passed to it
//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                .lineToYSplineHeading(33, Math.toRadians(0))
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .lineToY(10)
//                .setTangent(Math.toRadians(0))
//                .lineToX(10)
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3);

//        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
//                .lineToY(37)
//                .setTangent(Math.toRadians(0))
//                .lineToX(18)
//                .waitSeconds(3)
//                .setTangent(Math.toRadians(0))
//                .lineToXSplineHeading(46, Math.toRadians(180))
//                .waitSeconds(3);
//
//        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
//                .lineToYSplineHeading(33, Math.toRadians(180))
//                .waitSeconds(2)
//                .strafeTo(new Vector2d(46, 30))
//                .waitSeconds(3);

//        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
//                .strafeTo(new Vector2d(48, 12))
//                .build();

//        telemetry.addData("Initialization completed.\nServo Position", servo.getServo().getPosition());
        telemetry.addData("Bottom Shooter Position", shooter.getLeftWheel().getCurrentPosition());
        telemetry.update();

        waitForStart();

        if (isStopRequested())
            return;

//        Action trajectoryActionChosen;
//        if (true) {
//            trajectoryActionChosen = tab1.build();
//        } else if (false) {
//            trajectoryActionChosen = tab2.build();
//        } else {
//            trajectoryActionChosen = tab3.build();
//        }
        while(true)
         Actions.runBlocking(
                new SequentialAction(
//                        trajectoryActionChosen,
                        shooter.startShooter(1.0)
//                        servo.moveServo(),
//                        trajectoryActionCloseOut
                )
        );
//        sleep(30000);
//        Actions.runBlocking(
//                new SequentialAction(
//                        shooter.stopShooter()
//                )
//        );

//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(0, 0, 0))
//                        .lineToY(12)
////                        .lineToX(0)
//                        .build());
    }
}
