package org.firstinspires.ftc.teamcode.decode.mechanisms;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class TrajectoryActions {

//    TrajectoryActionBuilder builder;
//
//    public TrajectoryActions(TrajectoryActionBuilder builder) {
//        this.builder = builder;
//    }



     public Action getTrajectory_1_1(MecanumDrive drive, boolean initialPoseFlag){

         Pose2d pose;

         if (initialPoseFlag) {
             pose = new Pose2d(0, 0, Math.toRadians(-90));
         } else {
             pose = drive.localizer.getPose();
         }

         TrajectoryActionBuilder builder = drive.actionBuilder(pose)
                 .strafeToSplineHeading(new Vector2d(-5, 64), Math.toRadians(-45));

         return builder.build();
     }

//    public Action getTrajectory_1_2(){
//
//        this.builder = this.builder.endTrajectory().fresh()
//                .strafeToSplineHeading(new Vector2d(-30, 58), 135);
//
//        return builder.build();
//    }
//
//    public Action getTrajectory_1_3(){
//
//        this.builder = this.builder.endTrajectory().fresh()
//                .strafeToSplineHeading(new Vector2d(-5, 64), Math.toRadians(-135));
//
//        return this.builder.build();
//    }
}
