package com.tigerhuang.motion_profile2;

import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;

import javax.swing.JPanel;

/**
 * Panel that displays multiple graphs.
 * 
 * @author Tiger Huang
 *
 */
class GraphPanel extends JPanel {
	double[][] graphs;
	double[] maximums;
	Color[] colors;
	double dt;
	int cursor1;
	int cursor2;

	@Override
	public void paintComponent(Graphics g0) {
		Graphics2D g = (Graphics2D)g0;
		// Temp get width and height
		int width = getWidth()-1;
		int height = getHeight()-1;

		// Draw the background
		g.setColor(Color.WHITE);
		g.fillRect(0, 0, width, height);
		g.drawRect(0, 0, width, height);
		
		// Ensure there are graphs to draw
		if(graphs != null) {
			double tmax = (graphs[0].length-1)*dt;
			int itmax = (int)Math.ceil(tmax);
			
			//Draw the time lines
			g.setColor(Color.LIGHT_GRAY);
			for(int i = 0; i <= itmax * 4; i++) {
				int w = (int)Math.round(1.0*width * i / tmax / 4);
				g.drawLine(w, 0, w, height);
			}
			g.setColor(Color.BLACK);
			for(int i = 0; i <= itmax; i++) {
				int w = (int)Math.round(1.0*width * i / tmax);
				g.drawLine(w, 0, w, height);
			}
			
			//Draw the value lines
			g.setColor(Color.LIGHT_GRAY);
			for(int i = 0; i <= 4; i++) {
				int y = (int)Math.round(1.0*height * i / 4);
				g.drawLine(0, y, width, y);
			}
			g.setColor(Color.BLACK);
			int zeroY = (int)Math.round(1.0*height * 2 / 4);
			g.drawLine(0, zeroY, width, zeroY);
			
			//Draw the graphs
			for(int j = 0; j < graphs.length; j++) {
				g.setColor(colors[j]);
				for(int i = 0; i < graphs[j].length-1; i++) {
					g.drawLine(
							(int)Math.round(1.0*width * i * dt / tmax),
							(int)Math.round(height/2.0 - graphs[j][i] * height/2.0 / maximums[j]),
							(int)Math.round(1.0*width * (i+1) * dt / tmax),
							(int)Math.round(height/2.0 - graphs[j][i+1] * height/2.0 / maximums[j]));
				}
			}

			// Draw the cursors
			if(cursor1 >= 0) {
				g.setColor(Color.CYAN.darker());
				g.drawLine((int)Math.round(1.0*width * cursor1 * dt / tmax), 0, (int)Math.round(1.0*width * cursor1 * dt / tmax), height);
			}
			if(cursor2 >= 0) {
				g.setColor(Color.YELLOW.darker());
				g.drawLine((int)Math.round(1.0*width * cursor2 * dt / tmax), 0, (int)Math.round(1.0*width * cursor2 * dt / tmax), height);
			}
		}
	}
}
