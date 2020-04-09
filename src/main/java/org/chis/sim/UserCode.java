package org.chis.sim;

import org.chis.sim.GraphicDebug.Serie;
import org.chis.sim.Util.Vector2D;
import org.chis.sim.Util.Vector2D.Type;
import org.chis.sim.userclasses.RobotController;
import org.chis.sim.userclasses.RobotController.RobotPowers;
import org.chis.sim.userclasses.RobotController.RobotState;

import java.awt.Color;

public class UserCode{

    static RobotController controller = new RobotController(new RobotState());
    static RobotState targetRobotState;

    static Motor leftTopMotor;
    static Motor leftBottomMotor;
    static Motor rightTopMotor;
    static Motor rightBottomMotor;

    static Vector2D joystick;

    static Vector2D targetLinVelo = new Vector2D();
    static double targetAngVelo = 0;

    public static void initialize(){ //this function is run once when the robot starts
        GraphicDebug.turnOnAll(); //displaying the graphs

        leftTopMotor = Main.robot.leftModule.topMotor;
        leftBottomMotor = Main.robot.leftModule.bottomMotor;
        rightTopMotor = Main.robot.rightModule.topMotor;
        rightBottomMotor = Main.robot.rightModule.bottomMotor;
        
    }

    public static void execute(){ //this function is run 50 times a second (every 0.02 second)

        joystick = new Vector2D(Controls.rawX, -Controls.rawY, Type.CARTESIAN);

        controller.updateState(
            Main.robot.angVelo,
            leftTopMotor.getEncoderPosition(), 
            leftBottomMotor.getEncoderPosition(), 
            leftTopMotor.getEncoderVelocity(), 
            leftBottomMotor.getEncoderVelocity(),
            rightTopMotor.getEncoderPosition(),
            rightBottomMotor.getEncoderPosition(), 
            rightTopMotor.getEncoderVelocity(),
            rightBottomMotor.getEncoderVelocity()
        );


        joystick = joystick.scalarMult(0.1);
        // joystick = joystick.scalarMult(0.1);

        if(Math.abs(joystick.getMagnitude()) > 0.01){
            targetLinVelo = targetLinVelo.add(joystick);
            if(targetLinVelo.getMagnitude() > 2){
                targetLinVelo = new Vector2D(2, targetLinVelo.getAngle(), Type.POLAR);
            }
        }else{
            targetLinVelo = targetLinVelo.scalarMult(0.9);
        }

        targetLinVelo.rotate(-Main.robot.heading);

        
        if(Math.abs(Controls.slider) > 0.1){
            if(Math.abs(targetAngVelo + Controls.slider * 0.5) < 4){
                targetAngVelo += Controls.slider * 0.5;
            }
        }else{
            targetAngVelo *= 0.9;
        }
        
        targetRobotState = new RobotState(targetLinVelo, targetAngVelo);
        // targetRobotState = new RobotState(new Vector2D(0., -0.0, Type.CARTESIAN), -1);

        RobotPowers robotPowers = controller.move(targetRobotState);

        setDrivePowersAndFeed(
            robotPowers.leftTopPower,
            robotPowers.leftBottomPower,
            robotPowers.rightTopPower,
            robotPowers.rightBottomPower,
            0.0
        );


        // setDrivePowersAndFeed(
        //     1,
        //     -1,
        //     1,
        //     -1,
        //     0.0
        // );


        // Main.robot.setDrivePowers(
        //     joystick.getMagnitude(),
        //     joystick.getMagnitude(),
        //     joystick.getMagnitude(),
        //     joystick.getMagnitude());

                        
        // double forward = -Controls.rawY;
        // double forward = 0;
        // double moduleRot = Controls.rawX;

        // SimpleMatrix wheelMatrix = new SimpleMatrix(new double[][] { { forward }, { moduleRot } });
        // SimpleMatrix diffMatrix = new SimpleMatrix(new double[][] { { 0.5 , -0.5 }, { 0.5, 0.5 } });

        // SimpleMatrix ringsMatrix = diffMatrix.solve(wheelMatrix);

        // double top = ringsMatrix.get(0, 0);
        // double bottom = ringsMatrix.get(1, 0);

        // power ranges from -1 to 1s
        // Main.robot.setDrivePowers(top, bottom, top, bottom); 
        // Main.robot.setDrivePowers(1, -1, 1, -1);
        // Main.robot.setDrivePowers(forward+moduleRot, -(forward+moduleRot), forward-moduleRot, -(forward-moduleRot)); //tank drive

        graph(); //updating the graphs
    }

    private static void setDrivePowersAndFeed(double LT, double LB, double RT, double RB, double feedforward){
        Main.robot.setDrivePowers(
            LT + Math.copySign(feedforward, LT),
            LB + Math.copySign(feedforward, LB), 
            RT + Math.copySign(feedforward, RT), 
            RB + Math.copySign(feedforward, RB)
        );
    }




    // Motion graphs
    static Serie w1s1 = new Serie(Color.BLUE, 3);
    static Serie w1s2 = new Serie(Color.RED, 3);
    static GraphicDebug w1 = new GraphicDebug("net forces", new Serie[]{w1s1, w1s2}, 100);

    static Serie w2s1 = new Serie(Color.BLUE, 3);
    static Serie w2s2 = new Serie(Color.RED, 3);
    static GraphicDebug w2 = new GraphicDebug("angVelo", new Serie[]{w2s1, w2s2}, 100);

    static Serie w3s1 = new Serie(Color.BLUE, 3);
    static Serie w3s2 = new Serie(Color.RED, 3);
    static GraphicDebug w3 = new GraphicDebug("left angle", new Serie[]{w3s1, w3s2}, 100);

    static Serie w4s1 = new Serie(Color.BLUE, 3);
    static Serie w4s2 = new Serie(Color.RED, 3);
    static GraphicDebug w4 = new GraphicDebug("left angVelo", new Serie[]{w4s1, w4s2}, 100);
    
    private static void graph(){
        w1s1.addPoint(Main.elaspedTime, Main.robot.leftModule.force.x);
        w1s2.addPoint(Main.elaspedTime, Main.robot.rightModule.force.x);

        w2s1.addPoint(Main.elaspedTime, Main.robot.angVelo);
        w2s2.addPoint(Main.elaspedTime, UserCode.targetRobotState.angVelo);

        w3s1.addPoint(Main.elaspedTime, controller.leftController.state.moduleAngle);
        w3s2.addPoint(Main.elaspedTime, controller.leftController.modifiedTargetState.moduleAngle);

        w4s1.addPoint(Main.elaspedTime, controller.leftController.state.wheelAngVelo);
        w4s2.addPoint(Main.elaspedTime, controller.leftController.modifiedTargetState.wheelAngVelo);
        // w1s1.addPoint(Main.robot.leftModule.topRingSpeed, Main.robot.leftModule.bottomRingSpeed);


        // w1s1.addPoint(Main.elaspedTime, Main.robot.leftModule.force.x);
        // w1s2.addPoint(Main.elaspedTime, Main.robot.rightModule.force.x);
        // w1s2.addPoint(Main.elaspedTime, Main.robot.leftModule.force.x);

        // w2s1.addPoint(Main.robot.leftModule.topRingTorque, Main.robot.leftModule.bottomRingTorque);
        // w2s1.addPoint(Main.robot.forceNet.x, Main.robot.forceNet.y);
        // w2s1.addPoint(Main.elaspedTime, Main.robot.leftModule.topMotor.position);
        // w2s2.addPoint(Main.elaspedTime, Main.robot.leftModule.bottomMotor.position);

        GraphicDebug.paintAll();
    }




}
