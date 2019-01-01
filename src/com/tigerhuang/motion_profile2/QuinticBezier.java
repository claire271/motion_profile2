package com.tigerhuang.motion_profile2;

/**
 * A quintic bezier class used for generating splines with C2 continuity.
 * 
 * Concepts taken from the article:
 * "Planning Motion Trajectories for Mobile Robots Using Splines" by Christoph Sprunk
 * @author Tiger Huang
 *
 */
public class QuinticBezier {
	//Control points
	private double x0, y0;
	private double x1, y1;
	private double x2, y2;
	private double x3, y3;
	private double x4, y4;
	private double x5, y5;

	//Polynomial coefficients
	private double cxt5;
	private double cxt4;
	private double cxt3;
	private double cxt2;
	private double cxt1;
	private double cxt0;

	private double cyt5;
	private double cyt4;
	private double cyt3;
	private double cyt2;
	private double cyt1;
	private double cyt0;

	/**
	 * Creates a QuinticBezier from the specified waypoints.
	 * 
	 * @param w1 the first waypoint
	 * @param w2 the first waypoint
	 */
	public QuinticBezier(Waypoint w1, Waypoint w2) {
		x0 = w1.x;
		y0 = w1.y;

		x1 = 0.2 * w1.v_m * Math.cos(w1.v_t + Math.PI/2) + x0;
		y1 = 0.2 * w1.v_m * Math.sin(w1.v_t + Math.PI/2) + y0;

		x2 = .05 * w1.a_m * Math.cos(w1.a_t + w1.v_t + Math.PI/2) + 2 * x1 - x0;
		y2 = .05 * w1.a_m * Math.sin(w1.a_t + w1.v_t + Math.PI/2) + 2 * y1 - y0;

		x5 = w2.x;
		y5 = w2.y;

		x4 = -0.2 * w2.v_m * Math.cos(w2.v_t + Math.PI/2) + x5;
		y4 = -0.2 * w2.v_m * Math.sin(w2.v_t + Math.PI/2) + y5;

		x3 = .05 * w2.a_m * Math.cos(w2.a_t + w2.v_t + Math.PI/2) + 2 * x4 - x5;
		y3 = .05 * w2.a_m * Math.sin(w2.a_t + w2.v_t + Math.PI/2) + 2 * y4 - y5;

		calculateCoefficients();
	}

	/**
	 * Precomputes the polynomial used for generating position and derivatives from the control points.
	 */
	public void calculateCoefficients() {
		cxt5 = -x0 + 5*x1 - 10*x2 + 10*x3 - 5*x4 + x5;
		cxt4 = 5*x0 - 20*x1 + 30*x2 - 20*x3 + 5*x4;
		cxt3 = -10*x0 + 30*x1 - 30*x2 + 10*x3;
		cxt2 = 10*x0 - 20*x1 + 10*x2;
		cxt1 = -5*x0 + 5*x1;
		cxt0 = x0;

		cyt5 = -y0 + 5*y1 - 10*y2 + 10*y3 - 5*y4 + y5;
		cyt4 = 5*y0 - 20*y1 + 30*y2 - 20*y3 + 5*y4;
		cyt3 = -10*y0 + 30*y1 - 30*y2 + 10*y3;
		cyt2 = 10*y0 - 20*y1 + 10*y2;
		cyt1 = -5*y0 + 5*y1;
		cyt0 = y0;
	}

	/**
	 * Gets the x position at arc parameter t.
	 * 
	 * @param t The position along the arc where 0 &#8804; t &#8804; 1
	 * @return the x position
	 */
	public double getX(double t) {
		return
			cxt5 * t * t * t * t * t +
			cxt4 * t * t * t * t +
			cxt3 * t * t * t +
			cxt2 * t * t +
			cxt1 * t +
			cxt0;
	}

	/**
	 * Gets the y position at arc parameter t.
	 * 
	 * @param t The position along the arc where 0 &#8804; t &#8804; 1
	 * @return the y position
	 */
	public double getY(double t) {
		return
			cyt5 * t * t * t * t * t +
			cyt4 * t * t * t * t +
			cyt3 * t * t * t +
			cyt2 * t * t +
			cyt1 * t +
			cyt0;
	}

	/**
	 * Gets the first derivative of x position at arc parameter t.
	 * 
	 * @param t The position along the arc where 0 &#8804; t &#8804; 1
	 * @return the first derivative of x position
	 */
	public double getDX(double t) {
		return
			5 * cxt5 * t * t * t * t +
			4 * cxt4 * t * t * t +
			3 * cxt3 * t * t +
			2 * cxt2 * t +
				cxt1;
	}

	/**
	 * Gets the first derivative of y position at arc parameter t.
	 * 
	 * @param t The position along the arc where 0 &#8804; t &#8804; 1
	 * @return the first derivative of y position
	 */
	public double getDY(double t) {
		return
			5 * cyt5 * t * t * t * t +
			4 * cyt4 * t * t * t +
			3 * cyt3 * t * t +
			2 * cyt2 * t +
				cyt1;
	}

	/**
	 * Gets the second derivative of x position at arc parameter t.
	 * 
	 * @param t The position along the arc where 0 &#8804; t &#8804; 1
	 * @return the second derivative of x position
	 */
	public double getDDX(double t) {
		return
			20 * cxt5 * t * t * t +
			12 * cxt4 * t * t +
			6  * cxt3 * t +
			2  * cxt2;
	}

	/**
	 * Gets the second derivative of y position at arc parameter t.
	 * 
	 * @param t The position along the arc where 0 &#8804; t &#8804; 1
	 * @return the second derivative of y position
	 */
	public double getDDY(double t) {
		return
			20 * cyt5 * t * t * t +
			12 * cyt4 * t * t +
			6  * cyt3 * t +
			2  * cyt2;
	}

	/**
	 * Gets the heading at arc parameter t.
	 * 
	 * @param t The position along the arc where 0 &#8804; t &#8804; 1
	 * @return the heading (normalized so -PI &#8804; heading &#8804; PI)
	 */
	public double getHeading(double t) {
		double theta = Math.atan2(getDY(t), getDX(t)) - Math.PI/2;
		if(theta < -Math.PI) theta += Math.PI * 2;
		if(theta >  Math.PI) theta -= Math.PI * 2;
		return theta;
	}

	/**
	 * Gets the curvature at arc parameter t.
	 * 
	 * Code derived from http://mathworld.wolfram.com/Curvature.html
	 * 
	 * @param t The position along the arc where 0 &#8804; t &#8804; 1
	 * @return the curvature
	 */
	public double getCurvature(double t) {
		double dx = getDX(t);
		double dy = getDY(t);
		double ddx = getDDX(t);
		double ddy = getDDY(t);
		double d = Math.hypot(dx, dy);
		return (dx*ddy - dy*ddx) / (d*d*d);
	}
}
