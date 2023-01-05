// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import frc.robot.subsystems.DaveSubsystem.State;

/** Add your docs here. */
public class StateLogger {
    private class LoggingTuple{
        State state;
        double time;

        public LoggingTuple(State state, double time){
            this.state = state;
            this.time = time;
        }
    }
    private List<LoggingTuple> list;

    public StateLogger(){
        list = new ArrayList<>();
    }

    public void append(State state, double time){
        list.add(new LoggingTuple(state, time));
    }

    public void printStateLog(){
        for(LoggingTuple i: list){
            System.out.println(i.time + ": " +  i.state.toString());
        }
    }

    public void clearStateLog(){
        list = new ArrayList<>();
    }
}
