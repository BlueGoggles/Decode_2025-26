package org.firstinspires.ftc.teamcode.decode.mechanisms;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class TrajectoryActions {

    Pose2d pose;
    MecanumDrive drive;
    TrajectoryActionBuilder builder;

    public TrajectoryActions(MecanumDrive drive, Pose2d pose){
        this.drive = drive;
        this.pose = pose;
    }



     public Action getTrajectory_1_1(){

         TrajectoryActionBuilder builder = drive.actionBuilder(pose)
                 .strafeToSplineHeading(new Vector2d(5, 60), Math.toRadians(-45))
                 ;
//        this.builder = builder;
        return builder.build();
     }

    public Action getTrajectory_1_2(){

        this.builder = builder
//                .setTangent(Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-20, 60), 180)
                ;

        return builder.build();
    }

    public Action getTrajectory_1_3(){

        TrajectoryActionBuilder builder = drive.actionBuilder(pose)
                 .lineToYSplineHeading(33, Math.toRadians(135))
                ;

        return builder.build();
    }
}
