package org.chis.sim;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.ImageObserver;
import java.awt.Point;
import java.awt.Toolkit;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JPanel;

import org.chis.sim.Util.Vector2D;
import org.chis.sim.Util.Vector2D.Type;

public class GraphicSim extends JPanel implements MouseListener {
	private static final long serialVersionUID = -87884863222799400L;

	static JFrame frame;

	AffineTransform defaultTransform = new AffineTransform(); //to reset the g2d position and rotation

	static File robotFile;
	static File moduleFile;
    static File turnCenterFile;
    static File fieldFile;

	static BufferedImage robotImage;
    static BufferedImage moduleImage;
    static BufferedImage turnCenterImage;
    static BufferedImage fieldImage;


	static int screenHeight;
	static int screenWidth;
	static GraphicSim sim;

	static int robotImgHeight;
	static int robotDisplayWidth;
	static double robotScale;
	static int moduleDisplayWidth;

	static ArrayList<Point> points = new ArrayList<Point>();

  	GraphicSim() {
		addMouseListener(this);
	}

    @Override
	public void paint(Graphics g) { //gets called iteratively by JFrame
		double windowWidth = frame.getContentPane().getSize().getWidth();
		double windowHeight = frame.getContentPane().getSize().getHeight();
		super.paint(g);
		Graphics2D g2d = (Graphics2D) g;

		int x = (int) Util.posModulo(Main.robot.position.x * Constants.DISPLAY_SCALE.getDouble(), windowWidth); // robot position in pixels
		int y = (int) (windowHeight - Util.posModulo(Main.robot.position.y * Constants.DISPLAY_SCALE.getDouble(), windowHeight));

		// g.drawString("torque net " + Util.roundHundreths(Main.robot.torqueNet), 500, 575);
		g.drawString("heading " + Util.roundHundreths(Main.robot.heading), 500, 600);
		g.drawString("left angle " + UserCode.controller.leftController.state.moduleAngle, 500, 625);
		g.drawString("right angle " + UserCode.controller.rightController.state.moduleAngle, 500, 650);
		g.drawString("turn x "+ Util.roundHundreths(UserCode.controller.turnCenter.x), 500, 675);
		g.drawString("turn y "+ Util.roundHundreths(UserCode.controller.turnCenter.y), 500, 700);
		g.drawString("LT volt "+ Util.roundHundreths(Main.robot.leftModule.topMotor.voltage), 500, 725);
		g.drawString("LB volt "+ Util.roundHundreths(Main.robot.leftModule.bottomMotor.voltage), 500, 750);
		g.drawString("RT volt "+ Util.roundHundreths(Main.robot.rightModule.topMotor.voltage), 500, 775);
        g.drawString("RB volt "+ Util.roundHundreths(Main.robot.rightModule.bottomMotor.voltage), 500, 800);
        g.drawString("mode "+ UserCode.controller.robotState.mode, 500, 825);



		// g.drawString("LB motor speed " + Util.roundHundreths(Main.robot.leftModule.bottomMotor.angVelo), 500, 800);
		// g.drawString("LT motor torque " + Util.roundHundreths(Main.robot.leftModule.topMotor.torque), 700, 775);
		// g.drawString("LB motor torque " + Util.roundHundreths(Main.robot.leftModule.bottomMotor.torque), 700, 800);

        double fieldScale = 0.75;
        g2d.scale(fieldScale, fieldScale);
        g.drawImage(fieldImage, 80, 500, this);
        g2d.scale(1/fieldScale, 1/fieldScale);

		//drawing the grid
		g.setColor(Color.GRAY.brighter());
		for(int i = 0; i < screenWidth; i += Constants.DISPLAY_SCALE.getDouble() / Util.metersToFeet(1)){
			g.drawLine(i, 0, i, screenHeight);
		}
		for(int i = 0; i < screenHeight; i += Constants.DISPLAY_SCALE.getDouble() / Util.metersToFeet(1)){
			g.drawLine(0, i, screenWidth, i);
        }
        
		

		int robotCenterX = x + robotDisplayWidth/2;
		int robotCenterY = y + robotDisplayWidth/2;

		g2d.rotate(-Main.robot.heading, robotCenterX, robotCenterY); //angle is negative because g2d counts clockwise positive

		g2d.scale(robotScale, robotScale);
		g.translate((int) (x / robotScale), (int) (y / robotScale));
		g.drawImage(robotImage, 0, 0, this);


		drawFromCenter(g, moduleImage, 0, robotDisplayWidth/2, -Main.robot.leftModule.moduleAngle, this);

        drawFromCenter(g, moduleImage, robotDisplayWidth, robotDisplayWidth/2, -Main.robot.rightModule.moduleAngle, this);

        g.setColor(Color.CYAN);

        // Vector2D vectorL = Main.robot.leftModule.moduleTranslation;
        // Vector2D vectorR = Main.robot.rightModule.moduleTranslation;
        // Vector2D vectorRobot = Main.robot.robotRelTranslation;

        try{
            Vector2D vectorL = new Vector2D(UserCode.controller.targetLeftModuleState.wheelAngVelo, UserCode.controller.targetLeftModuleState.moduleAngle, Type.POLAR);
            Vector2D vectorR = new Vector2D(UserCode.controller.targetRightModuleState.wheelAngVelo, UserCode.controller.targetRightModuleState.moduleAngle, Type.POLAR);
            Vector2D vectorRobot = UserCode.targetRobotState.linVelo;

            vectorL = vectorL.scalarMult(10);
            vectorR = vectorR.scalarMult(10);
            vectorRobot = vectorRobot.scalarMult(10);
    
            g.drawLine(35, 15, (int) (35 + vectorL.x), (int) (15 - vectorL.y));
            g.drawLine(35, 60, (int) (35 + vectorR.x), (int) (60 - vectorR.y));
            g.drawLine(35, 38, (int) (35 + vectorRobot.x), (int) (38 - vectorRobot.y));
        }catch(NullPointerException e){

        }
        


        //star picture
        // Vector2D turnCenter = UserCode.controller.turnCenter;
        // g.drawImage(turnCenterImage, (int) (turnCenter.x * Constants.DISPLAY_SCALE.getDouble()/robotScale)+10, (int) -(turnCenter.y * Constants.DISPLAY_SCALE.getDouble()/robotScale)+10, this);

        



		// g2d.setTransform(defaultTransform);
		// g2d.scale(robotScale, robotScale);

		// g.drawImage(targetImage, (int) (1600 / robotScale), (int) (200 / robotScale), this);

		
    }
    
	public static void init(){
		screenWidth = (int) Toolkit.getDefaultToolkit().getScreenSize().getWidth();
		screenHeight = (int) Toolkit.getDefaultToolkit().getScreenSize().getHeight();
		try {
			robotFile = new File("./src/images/robot.png");
			moduleFile = new File("./src/images/module.png");
            turnCenterFile = new File("./src/images/turnCenter.png");
            fieldFile = new File("./src/images/field.png");

			robotImage = ImageIO.read(robotFile);
            moduleImage = ImageIO.read(moduleFile);
            turnCenterImage = ImageIO.read(turnCenterFile);
            fieldImage = ImageIO.read(fieldFile);


			setDisplayScales(robotFile);

		} catch (IOException e) {
			e.printStackTrace();
		}
		frame = new JFrame("Robot Sim");
		sim = new GraphicSim();
		frame.add(sim);
		frame.setSize((int) screenWidth-200, (int) screenHeight);
		frame.setLocation(200, 0);
		frame.setVisible(true);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}

	private static void setDisplayScales(File file) throws IOException {
		BufferedImage bufferedImage = ImageIO.read(file);
		robotImgHeight = bufferedImage.getHeight();
		robotDisplayWidth = (int) (Constants.DISPLAY_SCALE.getDouble() * Constants.ROBOT_WIDTH.getDouble()); //width of robot in pixels
		robotScale = (double) robotDisplayWidth / robotImgHeight; //scaling robot image to fit display width.
	}

	public static void rescale(){
		try {
			setDisplayScales(robotFile);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void drawFromCenter(Graphics g, BufferedImage image, int x, int y, double rotation, ImageObserver imageObserver){
		Graphics2D g2d = (Graphics2D) g;
		AffineTransform initialTransform = g2d.getTransform();
		g.translate(y + image.getWidth()/2, x + image.getWidth()/2);
		g2d.rotate(rotation);
		g.drawImage(image, -image.getWidth()/2, -image.getWidth()/2, imageObserver);
		g2d.setTransform(initialTransform);
    }

	

    public void mousePressed(MouseEvent e) {
	}

	public void mouseReleased(MouseEvent e) {
	}

	public void mouseEntered(MouseEvent e) {
	}

	public void mouseExited(MouseEvent e) {
	}

	public void mouseClicked(MouseEvent e) {
		System.out.println("mouseClicked");
	}


}