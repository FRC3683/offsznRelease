// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;

import frc.robot.utils.DavePath;

/** Add your docs here. */
public class TestAuto extends DavePath{

    public TestAuto(){
        super();
        try{
            addHolonomicSegment(1.0, 0.5, //max velocity, max acceleration
            Math.PI*2, Math.PI*4.0, //max angular velocity, max angular acceleration
            0.0, 0.7, //start velocity, end velocity
            0,      //start angle
            180,    //end angle
            // path weaver file name
            0.0, 0.0, 0.0, 1.0,
            2.0, 0.0, 0.0, 1.0);
            appendTargetTrackingSegment(1.0, 0.5, //max velocity, max acceleration
            Math.PI*2, Math.PI*4.0, //max angular velocity, max angular acceleration
            0,      //end velocity
            2, 2,   //targetx, targety
            // path weaver file name
            5.0, 0.0, 0.0, 1.0);
        } catch(Exception e){
            e.printStackTrace();
            loaded = false;
        }
        
    }

    @Override
    public void initialize(){
        super.initialize();
        swerve.recording = true;
    }

    @Override
    public void end(boolean interrupted) {
        try {
            swerve.recording = false;
            File pathFile = new File("recorded_path.txt");
            FileWriter fw = new FileWriter(pathFile);
            // fw.write(swerve.recordedPath.toString());
            fw.close();
        } catch (Exception e) {}
    }

    @Override
    public void execute(){
        super.execute();

    }
}
