package org.chis.sim.userclasses;

import org.chis.sim.Constants;
import org.chis.sim.Util.Vector2D;
import org.chis.sim.userclasses.ModuleController.ModuleState;

public class RobotController {

    public RobotState robotState;

    public ModuleController leftController = new ModuleController(new ModuleState());
    public ModuleController rightController = new ModuleController(new ModuleState());

    public ModuleState targetLeftModuleState;
    public ModuleState targetRightModuleState;

    public Vector2D turnCenter = new Vector2D(); 

    public RobotController(RobotState robotState){
        this.robotState = robotState;
    }

    public void move(RobotState targetRobotState){

        double targetLeftModuleAngle;
        double targetRightModuleAngle;

        double targetTangentialSpeed = targetRobotState.linVelo.getMagnitude(); //of the center of the robot
        // double targetTangentialSpeed = targetRobotState.angVelo; //of the center of the robot

        if(Math.abs(targetRobotState.angVelo) < 0.1){  //going straight deadband
            targetLeftModuleAngle = leftController.state.moduleAngle;
            targetRightModuleAngle = rightController.state.moduleAngle;
        }else{
            //Coords of center of rotation
            turnCenter.x = targetRobotState.linVelo.y / targetRobotState.angVelo;
            turnCenter.y = -targetRobotState.linVelo.x / targetRobotState.angVelo;

            // turnCenter.x += targetRobotState.linVelo.x * 0.01;
            // turnCenter.y += targetRobotState.linVelo.y * 0.01;
    
            targetLeftModuleAngle = Math.atan2(turnCenter.y - Constants.HALF_DIST_BETWEEN_WHEELS, turnCenter.x) - Math.PI/2.0;
            targetRightModuleAngle = Math.atan2(turnCenter.y + Constants.HALF_DIST_BETWEEN_WHEELS, turnCenter.x) - Math.PI/2.0;
        }

        double targetLeftWheelSpeed = targetTangentialSpeed - targetRobotState.angVelo * Constants.HALF_DIST_BETWEEN_WHEELS;
        double targetRightWheelSpeed = targetTangentialSpeed + targetRobotState.angVelo * Constants.HALF_DIST_BETWEEN_WHEELS;

        System.out.println("left: " + targetLeftWheelSpeed);
        System.out.println("right: " + targetRightWheelSpeed);
        System.out.println("tan: " + targetTangentialSpeed);


        targetLeftModuleState = new ModuleState(targetLeftModuleAngle, 0, 0, 0 / Constants.WHEEL_RADIUS.getDouble());
        targetRightModuleState = new ModuleState(targetRightModuleAngle, 0, 0, 0 / Constants.WHEEL_RADIUS.getDouble());

        leftController.move(targetLeftModuleState);
        rightController.move(targetRightModuleState);
    }

    public double calcClosestHeading(double currentHeading, double targetHeading){
        double difference2Pi = (currentHeading - targetHeading) % (2 * Math.PI); //angle error from (-180, 180)
        double closestHeading;
        if(Math.abs(difference2Pi) < (Math.PI)){ //chooses closer of the two acceptable angles closest to currentAngle
            closestHeading = currentHeading - difference2Pi;
        }else{
            closestHeading = currentHeading - difference2Pi + Math.copySign(2 * Math.PI, difference2Pi);
        }
        return closestHeading;
    }

    public static void main(String[] args) {
        var cont = new RobotController(new RobotState());
        System.out.println(cont.calcClosestHeading(Math.PI+.1, 0));
    }

    public void updateState(
        double angVelo,     
        double leftTopEncoderPosition,
        double leftBottomEncoderPosition,
        double leftTopEncoderVelocity,
        double leftBottomEncoderVelocity,
        double rightTopEncoderPosition,
        double rightBottomEncoderPosition,
        double rightTopEncoderVelocity,
        double rightBottomEncoderVelocity
    ){
        robotState.angVelo = angVelo;
        leftController.updateState(
            leftTopEncoderPosition, 
            leftBottomEncoderPosition, 
            leftTopEncoderVelocity, 
            leftBottomEncoderVelocity);
        rightController.updateState(
            rightTopEncoderPosition, 
            rightBottomEncoderPosition, 
            rightTopEncoderVelocity, 
            rightBottomEncoderVelocity);
    }



    public static class RobotState{
        public Vector2D linVelo;
        public double angVelo;
        public RobotState(Vector2D linVelo, double angVelo){
            this.linVelo = linVelo;
            this.angVelo = angVelo;
        }
        public RobotState(){
            this.linVelo = new Vector2D();
            this.angVelo = 0;
        }
        public RobotState copy(){
            return new RobotState(linVelo, angVelo);
        }
    }

    









}