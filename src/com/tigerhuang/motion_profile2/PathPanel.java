package com.tigerhuang.motion_profile2;

import javax.swing.JPanel;

import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.Point;
import java.awt.BasicStroke;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.Image;
import java.awt.image.BufferedImage;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseEvent;

import java.util.ArrayList;

/**
 * Path editor panel
 * 
 * @author Tiger Huang
 *
 */
class PathPanel extends JPanel implements MouseListener, MouseMotionListener {
	// Constants
	public final static BasicStroke THIN = new BasicStroke(1);
	public final static BasicStroke THICK = new BasicStroke(2);
	public final static Color PURPLE = new Color(127, 63, 191);

	// Reference to main frame
	private MainFrame mainFrame;

	// Pixels per unit distance
	double scale;

	// The center of the coordinate system is at this location in the view
	double offx;
	double offy;

	// Pan and zoom handling
	private int clickx;
	private int clicky;
	private int clickx0;
	private int clicky0;

	// Inspect handling
	double inspectX1;
	double inspectY1;
	double inspectU1;
	double inspectX2;
	double inspectY2;
	double inspectU2;

	// Parameters
	double timeStep = 0.01;
	double segmentLength = 0.01;
	double v_initial = 0; 
	double v_final = 0;
	double smoothing = 0.01;
	int max_iterations = 1000;
	double adjust_scale = 0.01;
	double adjust_offset = 0.99;
	int filter_length = 10;
	// Robot parameters
	double r_wb = 2.1;
	double r_ow = 2.7;
	double r_ol = 2.7;
	double v_max = 7;
	double v_tau = 0.25;
	double w_max = v_max*2/r_wb;
	double w_tau = v_tau;
	// Image settings
	double imageScale = 20;
	double imageBrightness = 0.5;
	// Zoom scale factor
	double scaleFactor = 1.01;
	// Limit defaults
	double default_length = 1;
	// Waypoint defaults
	double default_v_t = 0;
	double default_v_m = 2;
	double default_a_t = -Math.PI/2;
	double default_a_m = 5;
	// Display settings
	int displaySegments = 100;
	double v_scale = 0.5;
	double a_scale = 0.2;
	double widgetSize = 20;

	// File stuff
	String waypointPath = "";
	String imagePath = "";
	String profilePath = "";
	boolean dirty = false;
	BufferedImage rawImage;
	BufferedImage brightenedImage;
	int imageWidth;
	int imageHeight;

	// Waypoints
	ArrayList<Waypoint> waypoints;
	QuinticBezier splines[];
	double splinePoints[][];
	double splineData[][];
	double timeData[][];
	double times[];
	double timePoints[][];
	private Waypoint currWaypoint;
	private WaypointShape currWaypointShape;

	// Limits
	Limit firstLimit;
	ArrayList<Limit> limits;
	private Limit currLimit;
	private LimitShape currLimitShape;
	int currIndex;

	/**
	 * Constructor
	 */
	public PathPanel(MainFrame mainFrame) {
		this.mainFrame = mainFrame;
		scale = 49;
		offx = 0;
		offy = 0;
		inspectX1 = Double.NaN;
		inspectY1 = Double.NaN;
		inspectU1 = Double.NaN;
		inspectX2 = Double.NaN;
		inspectY2 = Double.NaN;
		inspectU2 = Double.NaN;
		addMouseListener(this);
		addMouseMotionListener(this);
		waypoints = new ArrayList<>();
		limits = new ArrayList<>();
		times = new double[0];
		currIndex = 0;
	}

	/**
	 * Custom drawing
	 */
	@Override
	public void paintComponent(Graphics g0) {
		Graphics2D g = (Graphics2D)g0;
		// Makes the program super slow
		//g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
		//g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);

		// Get width and height
		int width = getWidth();
		int height = getHeight();

		// Draw background and grid
		g.setColor(Color.WHITE);
		g.fillRect(0, 0, width, height);
		if(brightenedImage != null) {
			g.drawImage(brightenedImage, (int)Math.round(width/2.0 + scale*(offx - imageWidth/imageScale/2)),
					(int)Math.round(height/2.0 - scale*(offy - imageHeight/imageScale/2)),
					(int)Math.round(width/2.0 + scale*(offx + imageWidth/imageScale/2)),
					(int)Math.round(height/2.0 - scale*(offy + imageHeight/imageScale/2)),
					0, 0, imageWidth, imageHeight, null);
		}
		g.setColor(Color.BLACK);
		for(double x = -scale + width/2.0 % scale + offx%1 * scale; x <= width; x += scale) {
			g.drawLine((int)Math.round(x), 0, (int)Math.round(x), height);
		}
		for(double y = -scale + height/2.0 % scale - offy%1 * scale; y <= height; y += scale) {
			g.drawLine(0, (int)Math.round(y), width, (int)Math.round(y));
		}

		//Draw the waypoints
		g.setColor(Color.BLUE);
		for(int i = 0; i < waypoints.size(); i++) {
			if(currIndex > 0 && currIndex <= waypoints.size() && currIndex-1 == i) {
				g.setStroke(THICK);
			}
			else {
				g.setStroke(THIN);
			}
			waypoints.get(i).recalculateShapes(scale, widgetSize, offx, offy, width, height, v_scale, a_scale);
			g.draw(waypoints.get(i).pointShape);
			g.draw(waypoints.get(i).tangentShape);
			g.draw(waypoints.get(i).curvatureShape);
		}

		// Draw the limits
		g.setColor(Color.RED);
		g.setStroke(THIN);
		for(int i = 0; i < limits.size(); i++) {
			if(currIndex > waypoints.size() && currIndex-1-waypoints.size() == i) {
				g.setStroke(THICK);
			}
			else {
				g.setStroke(THIN);
			}
			limits.get(i).recalculateShapes(splines, scale, widgetSize, offx, offy, width, height);
			g.draw(limits.get(i).startShape);
			g.draw(limits.get(i).endShape);
		}

		// Draw the splines
		if(splines != null) {
			AffineTransform transform = new AffineTransform();
			transform.translate(width/2.0, height/2.0);
			transform.scale(scale, -scale);
			transform.translate(offx, offy);

			// Center spline
			Limit limit = null;
			if(currIndex > waypoints.size()) {
				limit = limits.get(currIndex-1-waypoints.size());
			}
			double[] points = new double[splinePoints[0].length];
			g.setColor(Color.MAGENTA);
			for(int i = 0; i < splines.length; i++) {
				transform.transform(splinePoints[i], 0, points, 0, displaySegments+1);
				for(int j = 0; j < displaySegments; j++) {
					if(limit != null && limit.t1 <= i+1.0*j/displaySegments && limit.t2 >= i+(j+1.0)/displaySegments) {
						g.setStroke(THICK);
					}
					else {
						g.setStroke(THIN);
					}
					g.drawLine((int)Math.round(points[j*2]), (int)Math.round(points[j*2+1]), (int)Math.round(points[j*2+2]), (int)Math.round(points[j*2+3]));
				}
			}

			// Left wheel spline
			g.setColor(PURPLE);
			g.setStroke(THIN);
			for(int i = 0; i < splines.length; i++) {
				for(int j = 0; j < displaySegments+1; j++) {
					double t = 1.0*j/displaySegments;
					double h = splines[i].getHeading(t);
					points[j*2] = splinePoints[i][j*2] - r_wb/2 * Math.cos(h);
					points[j*2+1] = splinePoints[i][j*2+1] - r_wb/2 * Math.sin(h);
				}
				transform.transform(points, 0, points, 0, displaySegments+1);
				for(int j = 0; j < displaySegments; j++) {
					g.drawLine((int)Math.round(points[j*2]), (int)Math.round(points[j*2+1]), (int)Math.round(points[j*2+2]), (int)Math.round(points[j*2+3]));
				}
			}

			// Right wheel spline
			for(int i = 0; i < splines.length; i++) {
				for(int j = 0; j < displaySegments+1; j++) {
					double t = 1.0*j/displaySegments;
					double h = splines[i].getHeading(t);
					points[j*2] = splinePoints[i][j*2] + r_wb/2 * Math.cos(h);
					points[j*2+1] = splinePoints[i][j*2+1] + r_wb/2 * Math.sin(h);
				}
				transform.transform(points, 0, points, 0, displaySegments+1);
				for(int j = 0; j < displaySegments; j++) {
					g.drawLine((int)Math.round(points[j*2]), (int)Math.round(points[j*2+1]), (int)Math.round(points[j*2+2]), (int)Math.round(points[j*2+3]));
				}
			}

			// Left spline
			g.setColor(Color.GREEN.darker());
			for(int i = 0; i < splines.length; i++) {
				for(int j = 0; j < displaySegments+1; j++) {
					double t = 1.0*j/displaySegments;
					double h = splines[i].getHeading(t);
					points[j*2] = splinePoints[i][j*2] - r_ow/2 * Math.cos(h);
					points[j*2+1] = splinePoints[i][j*2+1] - r_ow/2 * Math.sin(h);
				}
				transform.transform(points, 0, points, 0, displaySegments+1);
				for(int j = 0; j < displaySegments; j++) {
					g.drawLine((int)Math.round(points[j*2]), (int)Math.round(points[j*2+1]), (int)Math.round(points[j*2+2]), (int)Math.round(points[j*2+3]));
				}
			}

			// Right spline
			for(int i = 0; i < splines.length; i++) {
				for(int j = 0; j < displaySegments+1; j++) {
					double t = 1.0*j/displaySegments;
					double h = splines[i].getHeading(t);
					points[j*2] = splinePoints[i][j*2] + r_ow/2 * Math.cos(h);
					points[j*2+1] = splinePoints[i][j*2+1] + r_ow/2 * Math.sin(h);
				}
				transform.transform(points, 0, points, 0, displaySegments+1);
				for(int j = 0; j < displaySegments; j++) {
					g.drawLine((int)Math.round(points[j*2]), (int)Math.round(points[j*2+1]), (int)Math.round(points[j*2+2]), (int)Math.round(points[j*2+3]));
				}
			}

			// Start cap
			double[] points2 = new double[8];
			double x = splines[0].getX(0);
			double y = splines[0].getY(0);
			double h = splines[0].getHeading(0);
			points2[0] = x + r_ow/2 * Math.cos(h);
			points2[1] = y + r_ow/2 * Math.sin(h);
			points2[2] = x + r_ow/2 * Math.cos(h) + r_ol/2 * Math.sin(h);
			points2[3] = y + r_ow/2 * Math.sin(h) - r_ol/2 * Math.cos(h);
			points2[4] = x - r_ow/2 * Math.cos(h) + r_ol/2 * Math.sin(h);
			points2[5] = y - r_ow/2 * Math.sin(h) - r_ol/2 * Math.cos(h);
			points2[6] = x - r_ow/2 * Math.cos(h);
			points2[7] = y - r_ow/2 * Math.sin(h);
			transform.transform(points2, 0, points2, 0, 4);
			for(int j = 0; j < 3; j++) {
				g.drawLine((int)Math.round(points2[j*2]), (int)Math.round(points2[j*2+1]), (int)Math.round(points2[j*2+2]), (int)Math.round(points2[j*2+3]));
			}

			// End cap
			x = splines[splines.length-1].getX(1);
			y = splines[splines.length-1].getY(1);
			h = splines[splines.length-1].getHeading(1);
			points2[0] = x + r_ow/2 * Math.cos(h);
			points2[1] = y + r_ow/2 * Math.sin(h);
			points2[2] = x + r_ow/2 * Math.cos(h) - r_ol/2 * Math.sin(h);
			points2[3] = y + r_ow/2 * Math.sin(h) + r_ol/2 * Math.cos(h);
			points2[4] = x - r_ow/2 * Math.cos(h) - r_ol/2 * Math.sin(h);
			points2[5] = y - r_ow/2 * Math.sin(h) + r_ol/2 * Math.cos(h);
			points2[6] = x - r_ow/2 * Math.cos(h);
			points2[7] = y - r_ow/2 * Math.sin(h);
			transform.transform(points2, 0, points2, 0, 4);
			for(int j = 0; j < 3; j++) {
				g.drawLine((int)Math.round(points2[j*2]), (int)Math.round(points2[j*2+1]), (int)Math.round(points2[j*2+2]), (int)Math.round(points2[j*2+3]));
			}
		}

		// Draw the markers
		if(mainFrame.editState == EditState.INSPECT) {
			g.setStroke(THICK);
			g.setColor(Color.CYAN.darker());
			if(!Double.isNaN(inspectX1)) {
				double centerx = width/2.0 + scale * (inspectX1+offx);
				double centery = height/2.0 - scale * (inspectY1+offy);
				g.drawLine((int)Math.round(centerx - widgetSize), (int)Math.round(centery),
						(int)Math.round(centerx + widgetSize), (int)Math.round(centery));
				g.drawLine((int)Math.round(centerx), (int)Math.round(centery - widgetSize),
						(int)Math.round(centerx), (int)Math.round(centery + widgetSize));
				if(!Double.isNaN(inspectU1)) {
					double x = splines[(int)inspectU1].getX(inspectU1%1);
					double y = splines[(int)inspectU1].getY(inspectU1%1);
					double h = splines[(int)inspectU1].getHeading(inspectU1%1);
					g.drawLine((int)Math.round(width/2.0 + scale * (x+offx+r_wb/2*Math.cos(h))),
							(int)Math.round(height/2.0 - scale * (y+offy+r_wb/2*Math.sin(h))),
							(int)Math.round(width/2.0 + scale * (x+offx-r_wb/2*Math.cos(h))),
							(int)Math.round(height/2.0 - scale * (y+offy-r_wb/2*Math.sin(h))));
				}
			}

			g.setColor(Color.YELLOW.darker());
			if(!Double.isNaN(inspectX2)) {
				double centerx = width/2.0 + scale * (inspectX2+offx);
				double centery = height/2.0 - scale * (inspectY2+offy);
				g.drawLine((int)Math.round(centerx - widgetSize), (int)Math.round(centery),
						(int)Math.round(centerx + widgetSize), (int)Math.round(centery));
				g.drawLine((int)Math.round(centerx), (int)Math.round(centery - widgetSize),
						(int)Math.round(centerx), (int)Math.round(centery + widgetSize));
				if(!Double.isNaN(inspectU2)) {
					double x = splines[(int)inspectU2].getX(inspectU2%1);
					double y = splines[(int)inspectU2].getY(inspectU2%1);
					double h = splines[(int)inspectU2].getHeading(inspectU2%1);
					g.drawLine((int)Math.round(width/2.0 + scale * (x+offx+r_wb/2*Math.cos(h))),
							(int)Math.round(height/2.0 - scale * (y+offy+r_wb/2*Math.sin(h))),
							(int)Math.round(width/2.0 + scale * (x+offx-r_wb/2*Math.cos(h))),
							(int)Math.round(height/2.0 - scale * (y+offy-r_wb/2*Math.sin(h))));
				}
			}
		}
		g.setStroke(THIN);
	}

	/**
	 * Waypoint edit handler
	 */
	private void setWaypointLocation(double mousex, double mousey) {
		double posx = (mousex - getWidth()/2.0)/scale - offx;
		double posy = -(mousey - getHeight()/2.0)/scale - offy;
		if(currWaypoint != null) {
			if(currWaypointShape == WaypointShape.POINT) {
				currWaypoint.x = posx;
				currWaypoint.y = posy;
			}
			else if(currWaypointShape == WaypointShape.TANGENT) {
				currWaypoint.v_m = Math.hypot(posx - currWaypoint.x, posy - currWaypoint.y) / v_scale;
				currWaypoint.v_t = Math.atan2(posy - currWaypoint.y, posx - currWaypoint.x) - Math.PI/2;
				if(currWaypoint.v_t < -Math.PI) currWaypoint.v_t += Math.PI * 2;
				if(currWaypoint.v_t > Math.PI)  currWaypoint.v_t -= Math.PI * 2;
			}
			else if(currWaypointShape == WaypointShape.CURVATURE) {
				currWaypoint.a_m = Math.hypot(posx - currWaypoint.x, posy - currWaypoint.y) / a_scale;
				currWaypoint.a_t = Math.atan2(posy - currWaypoint.y, posx - currWaypoint.x) - Math.PI/2 - currWaypoint.v_t;
				if(currWaypoint.a_t < -Math.PI) currWaypoint.a_t += Math.PI * 2;
				if(currWaypoint.a_t > Math.PI)  currWaypoint.a_t -= Math.PI * 2;
			}
			mainFrame.updateEditPanel();
		}
	}

	/**
	 * Limit edit handler
	 */
	private void setLimitLocation(double mousex, double mousey) {
		double posx = (mousex - getWidth()/2.0)/scale - offx;
		double posy = -(mousey - getHeight()/2.0)/scale - offy;
		if(currLimit != null && currLimit != firstLimit) {
			// Find location closest to spline
			double min_distance = Double.MAX_VALUE;
			int min_index = 0;
			for(int i = 0; i < times.length; i++) {
				double distance = Math.hypot(posx - timePoints[i][0], posy - timePoints[i][1]);
				if(distance < min_distance) {
					min_index = i;
					min_distance = distance;
				}
			}
			double min_time = times[min_index];

			if(currLimitShape == LimitShape.START) {
				if(min_time <= currLimit.t2) {
					currLimit.t1 = min_time;
				}
			}
			else if(currLimitShape == LimitShape.END) {
				if(min_time >= currLimit.t1) {
					currLimit.t2 = times[min_index];
				}
			}
			mainFrame.updateEditPanel();
		}
	}

	/**
	 * Inspect handler
	 */
	private void setInspectLocation(double mousex, double mousey, boolean pressed) {
		double posx = (mousex - getWidth()/2.0)/scale - offx;
		double posy = -(mousey - getHeight()/2.0)/scale - offy;
		if(pressed) {
			inspectX1 = posx;
			inspectY1 = posy;
		}
		inspectX2 = posx;
		inspectY2 = posy;
	}

	/**
	 * Spline rebuild handler
	 */
	public void recalculateSplines() {
		if(waypoints.size() >= 2) {
			// Generate splines
			splines = SplineGenerator.splinesFromWaypoints(waypoints.toArray(new Waypoint[0]));

			// Generate display points
			splinePoints = new double[splines.length][];
			for(int i = 0; i < splines.length; i++) {
				double dt = 1.0/displaySegments;
				splinePoints[i] = new double[(displaySegments + 1) * 2];
				for(int j = 0; j <= displaySegments; j++) {
					double t = j * dt;
					splinePoints[i][j*2] = splines[i].getX(t);
					splinePoints[i][j*2 + 1] = splines[i].getY(t);
				}
			}

			// Generate spline data
			splineData = SplineGenerator.uniformLengthSegmentData(splines, segmentLength);
			times = new double[splineData[2].length];
			System.arraycopy(splineData[0], 0, times, 0, times.length);
			timePoints = new double[times.length][2];
			for(int i = 0; i < times.length; i++) {
				QuinticBezier s = splines[(int)times[i]];
				double t = times[i]%1;
				timePoints[i][0] = s.getX(t);
				timePoints[i][1] = s.getY(t);
			}
		}
		else {
			splines = null;
			splineData = null;
			timeData = null;
			times = new double[0];
			timePoints = null;
		}
	}

	/**
	 * Profile rebuild handler
	 */
	public void recalculateProfile() {
		dirty = true;
		if(splineData != null) {
			SplineGenerator.generateVelocityProfile(splineData, splines, limits.toArray(new Limit[0]),
					r_wb, smoothing, max_iterations, adjust_scale, adjust_offset,
					v_initial, v_final, v_max, v_tau, w_max, w_tau);
			timeData = SplineGenerator.generateTimeParameterizedProfile(splineData, splines,
					timeStep, r_wb, filter_length, v_max, v_tau, w_max, w_tau);

			// Process time graph
			int l = timeData[0].length;
			mainFrame.updateLabel("range_time", l*timeStep);

			// Process liner graph
			double[][] linearData = new double[3][];
			linearData[0] = timeData[0];
			linearData[1] = timeData[1];
			linearData[2] = timeData[2];
			double minPos = Double.MAX_VALUE;
			double maxPos = -Double.MAX_VALUE;
			double minVel = Double.MAX_VALUE;
			double maxVel = -Double.MAX_VALUE;
			double minAcc = Double.MAX_VALUE;
			double maxAcc = -Double.MAX_VALUE;
			for(int i = 0; i < l; i++) {
				minPos = Math.min(minPos, linearData[0][i]);
				maxPos = Math.max(maxPos, linearData[0][i]);
				minVel = Math.min(minVel, linearData[1][i]);
				maxVel = Math.max(maxVel, linearData[1][i]);
				minAcc = Math.min(minAcc, linearData[2][i]);
				maxAcc = Math.max(maxAcc, linearData[2][i]);
			}
			mainFrame.linearGraph.maximums[0] = Math.max(Math.abs(minPos), Math.abs(maxPos));
			mainFrame.updateLabel("range_lin_pos", mainFrame.linearGraph.maximums[0]);
			mainFrame.updateLabel("min_lin_pos", minPos);
			mainFrame.updateLabel("min_lin_vel", minVel);
			mainFrame.updateLabel("min_lin_acc", minAcc);
			mainFrame.updateLabel("max_lin_pos", maxPos);
			mainFrame.updateLabel("max_lin_vel", maxVel);
			mainFrame.updateLabel("max_lin_acc", maxAcc);
			mainFrame.linearGraph.graphs = linearData;

			// Process rotation graph
			double[][] angularData = new double[3][];
			angularData[0] = new double[l];
			angularData[1] = new double[l];
			angularData[2] = new double[l];
			for(int i = 0; i < l; i++) {
				angularData[0][i] = timeData[3][i]*180/Math.PI;
				angularData[1][i] = timeData[4][i]*180/Math.PI;
				angularData[2][i] = timeData[5][i]*180/Math.PI;
			}
			minPos = Double.MAX_VALUE;
			maxPos = -Double.MAX_VALUE;
			minVel = Double.MAX_VALUE;
			maxVel = -Double.MAX_VALUE;
			minAcc = Double.MAX_VALUE;
			maxAcc = -Double.MAX_VALUE;
			for(int i = 0; i < l; i++) {
				minPos = Math.min(minPos, angularData[0][i]);
				maxPos = Math.max(maxPos, angularData[0][i]);
				minVel = Math.min(minVel, angularData[1][i]);
				maxVel = Math.max(maxVel, angularData[1][i]);
				minAcc = Math.min(minAcc, angularData[2][i]);
				maxAcc = Math.max(maxAcc, angularData[2][i]);
			}
			mainFrame.angularGraph.maximums[0] = Math.max(Math.abs(minPos), Math.abs(maxPos));
			mainFrame.updateLabel("range_ang_pos", mainFrame.angularGraph.maximums[0]);
			mainFrame.updateLabel("min_ang_pos", minPos);
			mainFrame.updateLabel("min_ang_vel", minVel);
			mainFrame.updateLabel("min_ang_acc", minAcc);
			mainFrame.updateLabel("max_ang_pos", maxPos);
			mainFrame.updateLabel("max_ang_vel", maxVel);
			mainFrame.updateLabel("max_ang_acc", maxAcc);
			mainFrame.angularGraph.graphs = angularData;

			// Process voltage graph
			double[][] sideData = new double[2][];
			sideData[0] = timeData[6];
			sideData[1] = timeData[7];
			double minLeft = Double.MAX_VALUE;
			double maxLeft = -Double.MAX_VALUE;
			double minRight = Double.MAX_VALUE;
			double maxRight = -Double.MAX_VALUE;
			for(int i = 0; i < l; i++) {
				minLeft = Math.min(minLeft, sideData[0][i]);
				maxLeft = Math.max(maxLeft, sideData[0][i]);
				minRight = Math.min(minRight, sideData[1][i]);
				maxRight = Math.max(maxRight, sideData[1][i]);
			}
			mainFrame.updateLabel("min_left_vol", minLeft);
			mainFrame.updateLabel("max_left_vol", maxLeft);
			mainFrame.updateLabel("min_right_vol", minRight);
			mainFrame.updateLabel("max_right_vol", maxRight);
			mainFrame.sideGraph.graphs = sideData;
		}
		else {
			mainFrame.linearGraph.graphs = null;
			mainFrame.angularGraph.graphs = null;
			mainFrame.sideGraph.graphs = null;
			mainFrame.updateLabel("range_time", Double.NaN);
			mainFrame.updateLabel("range_lin_pos", Double.NaN);
			mainFrame.updateLabel("range_ang_pos", Double.NaN);
			mainFrame.updateLabel("min_lin_pos", Double.NaN);
			mainFrame.updateLabel("min_lin_vel", Double.NaN);
			mainFrame.updateLabel("min_lin_acc", Double.NaN);
			mainFrame.updateLabel("min_ang_pos", Double.NaN);
			mainFrame.updateLabel("min_ang_vel", Double.NaN);
			mainFrame.updateLabel("min_ang_acc", Double.NaN);
			mainFrame.updateLabel("min_left_vol", Double.NaN);
			mainFrame.updateLabel("min_right_vol", Double.NaN);
			mainFrame.updateLabel("max_lin_pos", Double.NaN);
			mainFrame.updateLabel("max_lin_vel", Double.NaN);
			mainFrame.updateLabel("max_lin_acc", Double.NaN);
			mainFrame.updateLabel("max_ang_pos", Double.NaN);
			mainFrame.updateLabel("max_ang_vel", Double.NaN);
			mainFrame.updateLabel("max_ang_acc", Double.NaN);
			mainFrame.updateLabel("max_left_vol", Double.NaN);
			mainFrame.updateLabel("max_right_vol", Double.NaN);
		}
		mainFrame.fixDividers();
	}

	/**
	 * Mouse handlers
	 */
	@Override
	public void mouseClicked(MouseEvent e) {}
	@Override
	public void mouseEntered(MouseEvent e) {}
	@Override
	public void mouseExited(MouseEvent e) {}

	@Override
	public void mouseMoved(MouseEvent e) {
		EditState state = mainFrame.editState;
		int x = e.getX();
		int y = e.getY();

		if(state == EditState.INSPECT) {
			setInspectLocation(x, y, false);
			mainFrame.updateInspect();
			mainFrame.linearGraph.repaint();
			mainFrame.angularGraph.repaint();
			mainFrame.sideGraph.repaint();
			repaint();
		}
	}

	@Override
	public void mousePressed(MouseEvent e) {
		EditState state = mainFrame.editState;
		int x = e.getX();
		int y = e.getY();

		if(state == EditState.INSPECT) {
			setInspectLocation(x, y, true);
			mainFrame.updateInspect();
			mainFrame.linearGraph.repaint();
			mainFrame.angularGraph.repaint();
			mainFrame.sideGraph.repaint();
		}

		else if(state == EditState.PAN || state == EditState.ZOOM) {
			clickx = x;
			clicky = y;
			clickx0 = x;
			clicky0 = y;
		}

		else if(state == EditState.EDITWAYPOINT) {
			Point mousePoint = new Point(x, y);
			for(Waypoint waypoint:waypoints) {
				if(waypoint.curvatureShape.contains(mousePoint)) {
					currWaypoint = waypoint;
					currWaypointShape = WaypointShape.CURVATURE;
					break;
				}
				else if(waypoint.tangentShape.contains(mousePoint)) {
					currWaypoint = waypoint;
					currWaypointShape = WaypointShape.TANGENT;
					break;
				}
				else if(waypoint.pointShape.contains(mousePoint)) {
					currWaypoint = waypoint;
					currWaypointShape = WaypointShape.POINT;
					break;
				}
			}
			setWaypointLocation(x, y);
			recalculateSplines();
			recalculateProfile();
			mainFrame.updateInspect();
			mainFrame.linearGraph.repaint();
			mainFrame.angularGraph.repaint();
			mainFrame.sideGraph.repaint();
			int index = waypoints.indexOf(currWaypoint);
			if(index >= 0) {
				mainFrame.editSelector.setSelectedIndex(1+index);
			}
		}

		else if(state == EditState.EDITLIMIT) {
			Point mousePoint = new Point(x, y);
			for(Limit limit:limits) {
				if(limit.startShape.contains(mousePoint)) {
					currLimit = limit;
					currLimitShape = LimitShape.START;
					break;
				}
				else if(limit.endShape.contains(mousePoint)) {
					currLimit = limit;
					currLimitShape = LimitShape.END;
					break;
				}
			}
			setLimitLocation(x, y);
			recalculateProfile();
			mainFrame.updateInspect();
			mainFrame.linearGraph.repaint();
			mainFrame.angularGraph.repaint();
			mainFrame.sideGraph.repaint();
			int index = limits.indexOf(currLimit);
			if(index >= 0) {
				mainFrame.editSelector.setSelectedIndex(1+waypoints.size()+index);
			}
		}

		repaint();
	}

	@Override
	public void mouseDragged(MouseEvent e) {
		EditState state = mainFrame.editState;
		int x = e.getX();
		int y = e.getY();

		if(state == EditState.INSPECT) {
			setInspectLocation(x, y, true);
			mainFrame.updateInspect();
			mainFrame.linearGraph.repaint();
			mainFrame.angularGraph.repaint();
			mainFrame.sideGraph.repaint();
		}

		else if(state == EditState.PAN) {
			offx += (x-clickx) / scale;
			offy -= (y-clicky) / scale;
			clickx = x;
			clicky = y;
		}

		else if(state == EditState.ZOOM) {
			double posx = (clickx0 - getWidth()/2.0)/scale - offx;
			double posy = -(clicky0 - getHeight()/2.0)/scale - offy;
			scale *= Math.pow(scaleFactor, y-clicky);
			offx = (clickx0 - getWidth()/2.0)/scale - posx;
			offy = -(clicky0 - getHeight()/2.0)/scale - posy;
			clickx = x;
			clicky = y;
		}

		else if(state == EditState.EDITWAYPOINT) {
			setWaypointLocation(x, y);
			recalculateSplines();
			recalculateProfile();
			mainFrame.updateInspect();
			mainFrame.linearGraph.repaint();
			mainFrame.angularGraph.repaint();
			mainFrame.sideGraph.repaint();
		}

		else if(state == EditState.EDITLIMIT) {
			setLimitLocation(x, y);
			recalculateProfile();
			mainFrame.updateInspect();
			mainFrame.linearGraph.repaint();
			mainFrame.angularGraph.repaint();
			mainFrame.sideGraph.repaint();
		}

		repaint();
	}

	@Override
	public void mouseReleased(MouseEvent e) {
		EditState state = mainFrame.editState;
		int x = e.getX();
		int y = e.getY();

		if(state == EditState.INSPECT) {
			setInspectLocation(x, y, true);
			mainFrame.updateInspect();
			mainFrame.linearGraph.repaint();
			mainFrame.angularGraph.repaint();
			mainFrame.sideGraph.repaint();
		}

		else if(state == EditState.PAN) {
			offx += (x-clickx) / scale;
			offy -= (y-clicky) / scale;
		}

		else if(state == EditState.ZOOM) {
			double posx = (clickx0 - getWidth()/2.0)/scale - offx;
			double posy = -(clicky0 - getHeight()/2.0)/scale - offy;
			scale *= Math.pow(scaleFactor, y-clicky);
			offx = (clickx0 - getWidth()/2.0)/scale - posx;
			offy = -(clicky0 - getHeight()/2.0)/scale - posy;
		}

		else if(state == EditState.ADDWAYPOINT) {
			if(x >= 0 && x <= getWidth() && y >= 0 && y <= getHeight()) {
				double posx = (x - getWidth()/2.0)/scale - offx;
				double posy = -(y - getHeight()/2.0)/scale - offy;

				// Create waypoint
				Waypoint waypoint = new Waypoint();
				waypoint.x = posx;
				waypoint.y = posy;
				waypoint.v_t = default_v_t;
				waypoint.v_m = default_v_m;
				waypoint.a_t = default_a_t;
				waypoint.a_m = default_a_m;

				// Insert at location closest to current spline
				double min_distance = Double.MAX_VALUE;
				int min_index = 0;
				for(int i = 0; i < times.length; i++) {
					double distance = Math.hypot(posx - timePoints[i][0], posy - timePoints[i][1]);
					if(distance < min_distance) {
						min_index = i;
						min_distance = distance;
					}
				}
				if(min_index == times.length-1 || times.length == 0) {
					waypoints.add(waypoint);
				}
				else if(min_index == 0) {
					waypoints.add(0, waypoint);
					for(Limit limit:limits) {
						if(limit != firstLimit) {
							limit.t1 += 1;
							limit.t2 += 1;
						}
					}
				}
				else {
					double min_time = times[min_index];
					int rounded_time = (int)min_time;
					waypoints.add(rounded_time+1, waypoint);
					for(Limit limit:limits) {
						if(limit != firstLimit) {
							if(limit.t1 >= rounded_time+1) {
								limit.t1 += 1;
							}
							else if(limit.t1 >= rounded_time) {
								limit.t1 = rounded_time + limit.t1%1*2;
							}
							if(limit.t2 >= rounded_time+1) {
								limit.t2 += 1;
							}
							else if(limit.t2 >= rounded_time) {
								limit.t2 = rounded_time + limit.t2%1*2;
							}
						}
					}
				}

				// Handle first limit
				if(waypoints.size() == 2) {
					firstLimit = new Limit();
					firstLimit.t1 = 0;
					firstLimit.t2 = Math.nextDown(1.0);
					firstLimit.type = LimitType.VOLTAGE;
					firstLimit.limit = 1.0;
					limits.add(firstLimit);
				}
				else if(waypoints.size() > 2) {
					firstLimit.t2 = Math.nextDown(waypoints.size()-1);
				}

				recalculateSplines();
				recalculateProfile();
				mainFrame.suppress = true;
				mainFrame.regenerateEditSelector();
				mainFrame.suppress = false;
				mainFrame.updateInspect();
				mainFrame.linearGraph.repaint();
				mainFrame.angularGraph.repaint();
				mainFrame.sideGraph.repaint();
			}
		}

		else if(state == EditState.EDITWAYPOINT) {
			setWaypointLocation(x, y);
			currWaypoint = null;
			recalculateSplines();
			recalculateProfile();
			mainFrame.updateInspect();
			mainFrame.linearGraph.repaint();
			mainFrame.angularGraph.repaint();
			mainFrame.sideGraph.repaint();
		}

		else if(state == EditState.DELETEWAYPOINT) {
			Point mousePoint = new Point(x, y);
			for(int i = 0; i < waypoints.size(); i++) {
				if(waypoints.get(i).pointShape.contains(mousePoint)) {
					// Handle waypoint
					waypoints.remove(i);

					// Handle limits
					if(waypoints.size() < 2) {
						limits.clear();
					}
					else {
						firstLimit.t2 = Math.nextDown(waypoints.size()-1);
					}
					for(Limit limit:limits) {
						if(limit != firstLimit) {
							if(limit.t1 >= i+1) {
								limit.t1 -= 1;
							}
							else if(limit.t1 >= i-1) {
								limit.t1 = i-1 + (limit.t1-i+1)/2;
							}
							if(limit.t2 >= i+1) {
								limit.t2 -= 1;
							}
							else if(limit.t2 >= i-1) {
								limit.t2 = i-1 + (limit.t2-i+1)/2;
							}
							if(limit.t1 < 0) {
								limit.t1 = 0;
							}
							else if(limit.t1 >= waypoints.size()-1) {
								limit.t1 = Math.nextDown(waypoints.size()-1.0);
							}
							if(limit.t2 < 0) {
								limit.t2 = 0;
							}
							else if(limit.t2 >= waypoints.size()-1) {
								limit.t2 = Math.nextDown(waypoints.size()-1.0);
							}
						}
					}

					recalculateSplines();
					recalculateProfile();
					mainFrame.suppress = true;
					mainFrame.regenerateEditSelector();
					mainFrame.suppress = false;
					mainFrame.updateInspect();
					mainFrame.linearGraph.repaint();
					mainFrame.angularGraph.repaint();
					mainFrame.sideGraph.repaint();
					break;
				}
			}
		}

		else if(state == EditState.ADDLIMIT && times.length > 0) {
			double posx = (x - getWidth()/2.0)/scale - offx;
			double posy = -(y - getHeight()/2.0)/scale - offy;

			// Find location closest to spline
			double min_distance = Double.MAX_VALUE;
			int min_index = 0;
			for(int i = 0; i < times.length; i++) {
				double distance = Math.hypot(posx - timePoints[i][0], posy - timePoints[i][1]);
				if(distance < min_distance) {
					min_index = i;
					min_distance = distance;
				}
			}
			int index1 = min_index - (int)Math.round(default_length/2/segmentLength);
			int index2 = min_index + (int)Math.round(default_length/2/segmentLength);
			if(index1 < 0) {
				index1 = 0;
			}
			if(index2 >= times.length) {
				index2 = times.length-1;
			}

			// Create limit
			Limit limit = new Limit();
			limit.t1 = times[index1];
			limit.t2 = times[index2];
			limit.type = LimitType.VOLTAGE;
			limit.limit = 1;
			limits.add(limit);

			recalculateProfile();
			mainFrame.suppress = true;
			mainFrame.regenerateEditSelector();
			mainFrame.suppress = false;
			mainFrame.updateInspect();
			mainFrame.linearGraph.repaint();
			mainFrame.angularGraph.repaint();
			mainFrame.sideGraph.repaint();
		}

		else if(state == EditState.EDITLIMIT) {
			setLimitLocation(x, y);
			recalculateProfile();
			mainFrame.updateInspect();
			mainFrame.linearGraph.repaint();
			mainFrame.angularGraph.repaint();
			mainFrame.sideGraph.repaint();
			currLimit = null;
		}

		else if(state == EditState.DELETELIMIT) {
			Point mousePoint = new Point(x, y);
			for(Limit limit:limits) {
				if(limit != firstLimit) {
					if(limit.startShape.contains(mousePoint)) {
						limits.remove(limit);
						recalculateProfile();
						mainFrame.suppress = true;
						mainFrame.regenerateEditSelector();
						mainFrame.suppress = false;
						mainFrame.updateInspect();
						mainFrame.linearGraph.repaint();
						mainFrame.angularGraph.repaint();
						mainFrame.sideGraph.repaint();
						break;
					}
					else if(limit.endShape.contains(mousePoint)) {
						limits.remove(limit);
						recalculateProfile();
						mainFrame.suppress = true;
						mainFrame.regenerateEditSelector();
						mainFrame.suppress = false;
						mainFrame.updateInspect();
						mainFrame.linearGraph.repaint();
						mainFrame.angularGraph.repaint();
						mainFrame.sideGraph.repaint();
						break;
					}
				}
			}
		}

		repaint();
	}
}
