package com.tigerhuang.motion_profile2;

import java.awt.Polygon;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;

/**
 * Limits for use with quintic bezeiers.
 * 
 * @author Tiger Huang
 *
 */
public class Limit {
	/**
	 * The start time (unitless)
	 */
	public double t1;
	/**
	 * The end time (unitless)
	 */
	public double t2;
	/**
	 * The type of limit
	 */
	public LimitType type;
	/**
	 * The value of the limit (units depend on type)
	 */
	public double limit;

	// Shapes for drawing and dragging
	public Shape startShape;
	public Shape endShape;

	/**
	 * Creates a default limit
	 */
	public Limit() {
		t1 = 0;
		t2 = 0;
		limit = 0;
		startShape = new Polygon();
		endShape = new Polygon();
	}

	/**
	 * Precomputes the shapes for more efficient drawing and manipulation.
	 * 
	 * @param scale the scale of the distances being drawn on the window
	 * @param widgetSize how large the shapes should be
	 * @param offx the x offset at which to create the shapes
	 * @param offy the y offset at which to create the shapes
	 * @param width the width of the display window
	 * @param height the height of the display window
	 * @param v_scale how much to scale the velocity vector
	 * @param a_scale how much to scale the acceleration vector
	 */
	public void recalculateShapes(
			QuinticBezier[] splines,
			double scale, double widgetSize,
			double offx, double offy,
			double width, double height) {

		// Calculate the start point shape
		QuinticBezier s1 = splines[(int)t1];
		double t1r = t1%1;
		double x1 = s1.getX(t1r);
		double y1 = s1.getY(t1r);
		double h1 = s1.getHeading(t1r);
		double startPoints[] = {
			widgetSize/2, 0,
			-widgetSize/2, 0,
			0, -widgetSize/2,
			widgetSize/2, 0,
			widgetSize/2, widgetSize/4,
			-widgetSize/2, widgetSize/4,
			-widgetSize/2, 0,
		};
		startShape = Limit.transformPolygon(
				startPoints, scale,
				x1 + offx, y1 + offy,
				width, height,
				h1, 0);

		// Calculate the end point shape
		QuinticBezier s2 = splines[(int)t2];
		double t2r = t2%1;
		double x2 = s2.getX(t2r);
		double y2 = s2.getY(t2r);
		double h2 = s2.getHeading(t2r);
		double endPoints[] = {
			widgetSize/2, 0,
			-widgetSize/2, 0,
			0, widgetSize/2,
			widgetSize/2, 0,
			widgetSize/2, -widgetSize/4,
			-widgetSize/2, -widgetSize/4,
			-widgetSize/2, 0,
		};
		endShape = Limit.transformPolygon(
				endPoints, scale,
				x2 + offx, y2 + offy,
				width, height,
				h2, 0);
	}

	/**
	 * Creates a shape to use for drawing and manipulating from the parameters.
	 * 
	 * @param points the points of the polygon<br>
	 * The format is [x0, y0; x1, y1; ...]
	 * @param scale the scale of the distances being drawn on the window
	 * @param offsetx the x position of this polygon
	 * @param offsety the y position of this polygon
	 * @param width the width of the drawing window
	 * @param height the height of the drawing window
	 * @param rotation the direction of the vector being represented
	 * @param magnitude the magnitude of the vector being represented
	 * @return a shape for drawing and manipulation
	 */
	private static Polygon transformPolygon(
			double points[], double scale,
			double offsetx, double offsety,
			double width, double height,
			double rotation, double magnitude) {
		AffineTransform transform = new AffineTransform();
		transform.translate(
				offsetx * scale + width/2,
				-offsety * scale + height/2);
		transform.rotate(-rotation);
		transform.translate(0, -magnitude * scale);
		transform.transform(points, 0, points, 0, points.length/2);
		Polygon polygon = new Polygon();
		for(int j = 0;j < points.length/2;j++) {
			polygon.addPoint(
					(int)Math.round(points[j * 2]),
					(int)Math.round(points[j * 2 + 1]));
		}
		return polygon;
	}
}
