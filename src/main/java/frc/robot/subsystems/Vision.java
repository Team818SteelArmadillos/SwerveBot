// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.LimelightHelpers;
// import frc.robot.RobotContainer;

// /** Add your docs here. */
// public class Vision extends SubsystemBase{
//     SwerveDrivePoseEstimator m_SwerveDrivePoseEstimator;
//     String limelightName1 = "limelight1";
//     String limelightName2 = "limelight2";
//     String limelightName3 = "limelight3";

//     public Vision(){
//         m_SwerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
//             Constants.kinematics, 
//             RobotContainer.m_SwerveSubsystem.getPose().getRotation(), 
//             RobotContainer.m_SwerveSubsystem.getpo(), 
//             RobotContainer.m_SwerveSubsystem.getPose()
//             );

//         LimelightHelpers.setPipelineIndex(limelightName1, 0);
//         LimelightHelpers.setPipelineIndex(limelightName2, 0);
//         LimelightHelpers.setPipelineIndex(limelightName3, 0);
//     }
    
    
//     public void setObjectDetection(){
//         LimelightHelpers.setPipelineIndex(limelightName1, 1);
//     }

//     public void setApriTag(){
//         LimelightHelpers.setPipelineIndex(limelightName1, 0);
//     }

//     public double getPipeline(){
//         return LimelightHelpers.getCurrentPipelineIndex(limelightName1);
//     }

//     public void updatePoseLimelight(){
//         Pose2d limelightPoses[];
//         Double limelightLatencies[];
//         Double xDelta[], yDelta[], rDelta[];

//         Pose2d swerveOdometry = RobotContainer.m_SwerveSubsystem.getPose();

//         if(LimelightHelpers.getCurrentPipelineIndex(limelightName1) == 0){ //Checks if limelight 1 is looking at pieces
//             limelightPoses = new Pose2d[]{ //Creates array with 3 botposes
//                 LimelightHelpers.getBotPose2d(limelightName1),
//                 LimelightHelpers.getBotPose2d(limelightName2),
//                 LimelightHelpers.getBotPose2d(limelightName3)
//             };

//             limelightLatencies = new Double[]{
//                 LimelightHelpers.getLatency_Pipeline(limelightName1) + LimelightHelpers.getLatency_Capture(limelightName1),
//                 LimelightHelpers.getLatency_Pipeline(limelightName2) + LimelightHelpers.getLatency_Capture(limelightName2),
//                 LimelightHelpers.getLatency_Pipeline(limelightName3) + LimelightHelpers.getLatency_Capture(limelightName3)
//             };
//         }else{
//             limelightPoses = new Pose2d[]{ //Creates array with 2 botposes
//                 LimelightHelpers.getBotPose2d(limelightName2),
//                 LimelightHelpers.getBotPose2d(limelightName3)
//             };

//             limelightLatencies = new Double[]{
//                 LimelightHelpers.getLatency_Pipeline(limelightName2) + LimelightHelpers.getLatency_Capture(limelightName2),
//                 LimelightHelpers.getLatency_Pipeline(limelightName3) + LimelightHelpers.getLatency_Capture(limelightName3)
//             };
//         }

//         xDelta = new Double[limelightPoses.length];
//         yDelta = new Double[limelightPoses.length];
//         rDelta = new Double[limelightPoses.length];
//         //Creates delta values to validate vision results
//         for(var i = 0; i < limelightPoses.length; i++){
//             xDelta[i] = Math.abs(swerveOdometry.getX() - limelightPoses[i].getX());
//             yDelta[i] = Math.abs(swerveOdometry.getY() - limelightPoses[i].getY());
//             rDelta[i] = Math.abs(swerveOdometry.getRotation().getDegrees() - limelightPoses[i].getRotation().getDegrees());
//         }
        
//         //Adds valid values
//         for(var i = 0; i < limelightPoses.length; i++){
//             if(4.2 < limelightPoses[i].getX() && limelightPoses[i].getX() < -4.2 //Makes sure X value is in field range
//                 && 8.4 < limelightPoses[i].getY() && limelightPoses[i].getY() < -8.4 //Makes sure Y value is in field range
//                 && xDelta[i]  > 1 //Makes sure x, y and r delta's aren't ridiculous 
//                 && yDelta[i] > 1
//                 && rDelta[i] > 10
//             ){
//                 m_SwerveDrivePoseEstimator.addVisionMeasurement(limelightPoses[i], Timer.getFPGATimestamp() - limelightLatencies[i]); //Adds vision measure with latency adjusted timestamp
//             }
//         }
//     }

//     public boolean object(){ 
//         return LimelightHelpers.getTV(limelightName1); //Checks for valid targets within the target area
//     }

//     public double objectOffset(){
//         return LimelightHelpers.getTX(limelightName1); //Returns the horizontal offset of the detected object
//     }

//     public void updatePoseSwerve(){
//         m_SwerveDrivePoseEstimator.update(RobotContainer.m_SwerveSubsystem.getPose().getRotation(), 
//         RobotContainer.m_SwerveSubsystem.getPositions());
//     }

//     public Pose2d getFusedPose(){
//         return m_SwerveDrivePoseEstimator.getEstimatedPosition();
//     }
// }



    

