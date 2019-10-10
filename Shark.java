package sh;
import robocode.*;
import robocode.util.*;
import java.awt.*;
import java.awt.geom.*;
public class Shark extends AdvancedRobot{
	boolean movingForward;
	boolean inWall;
	RobotStatus robotStatus;
	public void run() {
		setCores();
		setAdjustRadarForRobotTurn(true);
		setAdjustGunForRobotTurn(true);
		setAdjustRadarForGunTurn(true);
		if (getX() <= 50 || getY() <= 50 || getBattleFieldWidth() - getX() <= 50 || getBattleFieldHeight() - getY() <= 50) {
			this.inWall = true;
		} else {
			this.inWall = false;
		}
		setAhead(40000);
		setTurnRadarRight(360);
		this.movingForward = true;
		while(true) {
			if (getX() > 50 && getY() > 50 && getBattleFieldWidth() - getX() > 50 && getBattleFieldHeight() - getY() > 50 && this.inWall == true){
				this.inWall = false;
			}
			if (getX() <= 50 || getY() <= 50 || getBattleFieldWidth() - getX() <= 50 || getBattleFieldHeight() - getY() <= 50) {
				if (this.inWall == false) {
					reverseDirection();
					inWall = true;
				}
			}
			if (getRadarTurnRemaining() == 0.0){
				setTurnRadarRight(360);
			}
			execute();
		}
	}

	public void onStatus(StatusEvent e) {
		this.robotStatus = e.getStatus();
	}

	public void onScannedRobot(ScannedRobotEvent e) {
		// Replace the next line with any behavior you would like
		double absoluteBearing = getHeading() + e.getBearing();
		double bearingFromGun = Utils.normalRelativeAngleDegrees(absoluteBearing - getGunHeading());
		// double bearingFromRadar = Utils.normalRelativeAngleDegrees(absoluteBearing - getRadarHeading());
		double bulletPower = Math.min(4.5 - Math.abs(bearingFromGun) / 2 - e.getDistance() / 250, getEnergy() - .1);
		
		if (this.movingForward) {
			setTurnRight(Utils.normalRelativeAngleDegrees(e.getBearing() + 80));
		} else {
			setTurnRight(Utils.normalRelativeAngleDegrees(e.getBearing() + 100));
		}
		
		/*if (Math.abs(bearingFromGun) <= 4) {
			setTurnGunRight(bearingFromGun);
			setTurnRadarRight(bearingFromRadar);
			if (getGunHeat() == 0) {
				fire(Math.min(4.5 - Math.abs(bearingFromGun) / 2 - e.getDistance() / 250, getEnergy() - .1));
			}
		} else {
			setTurnGunRight(bearingFromGun);
			setTurnRadarRight(bearingFromRadar);
		}*/
		double angle = Math.toRadians((getHeading() + e.getBearing()) % 360);
		int scannedX = (int)(robotStatus.getX() + Math.sin(angle) * e.getDistance());
		int scannedY = (int)(robotStatus.getY() + Math.cos(angle) * e.getDistance());
		CircularIntercept intercept = new CircularIntercept();
		intercept.calculate(getX(),
							getY(),
							scannedX,
							scannedY,
							e.getHeading(),
							e.getVelocity(),
							bulletPower,
							0);
		// Helper function that converts any angle into
		// an angle between +180 and -180 degrees.
		double turnAngle = Utils.normalRelativeAngle(intercept.bulletHeading_deg - getGunHeading());
		// Move gun to target angle
		setTurnGunRight(turnAngle);
		out.println("Actual position: " + scannedX + ", " + scannedY);
		out.println("calculated position: " + intercept.impactPoint.getX() + ", " + intercept.impactPoint.getY());
		if (Math.abs(turnAngle) <= intercept.angleThreshold) {
			// Ensure that the gun is pointing at the correct angle
			if ((intercept.impactPoint.getX() > 0) &&
				(intercept.impactPoint.getX() < getBattleFieldWidth()) &&
				(intercept.impactPoint.getY() > 0) &&
				(intercept.impactPoint.getY() < getBattleFieldHeight())) {
				// Ensure that the predicted impact point is within
				// the battlefield
 				fire(bulletPower);
 			}
		}
		
		if (bearingFromGun == 0){
			scan();
		}
	}
	public void onHitWall(HitWallEvent e) {
		// Replace the next line with any behavior you would like
		reverseDirection();
	}
	
	public void onHitRobot(HitRobotEvent e) {
		if (e.isMyFault()){
			reverseDirection();
		}
	}
	
	public void setCores() {
		setColors(Color.BLACK, Color.BLACK, Color.GRAY, Color.RED, Color.GRAY);
	}
	
	public void reverseDirection(){
		if (this.movingForward) {
			setBack(40000);
			this.movingForward = false;
		} else {
			setAhead(40000);
			this.movingForward = true;
		}
	}
	
	public class Intercept {
		public Point2D.Double impactPoint = new Point2D.Double(0,0);
		public double bulletHeading_deg;
		protected Point2D.Double bulletStartingPoint = new Point2D.Double();
		protected Point2D.Double targetStartingPoint = new Point2D.Double();
		private double ROBOT_RADIUS = 300;
		public double targetHeading, targetVelocity, bulletPower, angleThreshold, distance, vb;
		protected double impactTime, angularVelocity_rad_per_sec;
		public void calculate (double xb, double yb, double xt, double yt, double tHeading, double vt, double bPower, double angularVelocity_deg_per_sec){
			angularVelocity_rad_per_sec = Math.toRadians(angularVelocity_deg_per_sec);
			bulletStartingPoint.setLocation(xb, yb);
			targetStartingPoint.setLocation(xt, yt);
			targetHeading = tHeading;
			targetVelocity = vt;
			bulletPower = bPower;
			vb = 20-3*bulletPower;
			double dX,dY;
			impactTime = getImpactTime(10, 20, 0.01);
			impactPoint = getEstimatedPosition(impactTime);
			dX = (impactPoint.getX() - bulletStartingPoint.getX());
			dY = (impactPoint.getY() - bulletStartingPoint.getY());
			distance = Math.sqrt(dX*dX+dY*dY);
			bulletHeading_deg = Math.toDegrees(Math.atan2(dX,dY));
			angleThreshold = Math.toDegrees(Math.atan(ROBOT_RADIUS/distance));
		}

		protected Point2D.Double getEstimatedPosition(double time) {
			double x = targetStartingPoint.getX() + targetVelocity * time * Math.sin(Math.toRadians(targetHeading));
			double y = targetStartingPoint.getY() + targetVelocity * time * Math.cos(Math.toRadians(targetHeading));
			return new Point2D.Double(x,y);
		}

		private double f(double time) {
			vb = 20-3*bulletPower;
			Point2D.Double targetPosition = getEstimatedPosition(time);
			double dX = (targetPosition.getX() - bulletStartingPoint.getX());
			double dY = (targetPosition.getY() - bulletStartingPoint.getY());	
			return Math.sqrt(dX*dX + dY*dY) - vb * time;
		}

		private double getImpactTime(double t0, double t1, double accuracy) {
			double X = t1;
			double lastX = t0;
			int iterationCount = 0;
			double lastfX = f(lastX);
			while ((Math.abs(X - lastX) >= accuracy) && (iterationCount < 15)) {
				iterationCount++;
				double fX = f(X);
				if ((fX-lastfX) == 0.0) break;
				double nextX = X - fX*(X-lastX)/(fX-lastfX);
				lastX = X;
				X = nextX;
				lastfX = fX;
			}
			return X;
		}
	}

	public class CircularIntercept extends Intercept {
		protected Point2D.Double getEstimatedPosition(double time) {
			if (Math.abs(angularVelocity_rad_per_sec)<= Math.toRadians(0.1)) return super.getEstimatedPosition(time);
			double initialTargetHeading = Math.toRadians(targetHeading);
			double finalTargetHeading = initialTargetHeading + angularVelocity_rad_per_sec * time;
			double x = targetStartingPoint.getX() - targetVelocity / angularVelocity_rad_per_sec * (Math.cos(finalTargetHeading) - Math.cos(initialTargetHeading));
			double y = targetStartingPoint.getY() - targetVelocity / angularVelocity_rad_per_sec * (Math.sin(initialTargetHeading) - Math.sin(finalTargetHeading));
			return new Point2D.Double(x,y);
		}
	}
}
