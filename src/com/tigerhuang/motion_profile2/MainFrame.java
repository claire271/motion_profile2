package com.tigerhuang.motion_profile2;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JSplitPane;
import javax.swing.SwingUtilities;
import javax.swing.JMenuBar;
import javax.swing.JMenu;
import javax.swing.JMenuItem;
import javax.swing.JRadioButtonMenuItem;
import javax.swing.JScrollPane;
import javax.swing.ScrollPaneConstants;
import javax.swing.ButtonGroup;
import javax.swing.JLabel;
import javax.swing.JComboBox;
import javax.swing.KeyStroke;
import javax.swing.BoxLayout;
import javax.swing.JSpinner;
import javax.swing.SpinnerNumberModel;
import javax.swing.event.ChangeListener;
import javax.swing.event.ChangeEvent;
import javax.swing.JOptionPane;
import javax.swing.JFileChooser;
import javax.swing.filechooser.FileNameExtensionFilter;
import javax.swing.filechooser.FileFilter;
import javax.imageio.ImageIO;

import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.GridBagLayout;
import java.awt.GridBagConstraints;
import java.awt.CardLayout;
import java.awt.GridLayout;
import java.awt.image.BufferedImage;
import java.awt.image.RescaleOp;
import java.awt.image.ColorConvertOp;
import java.awt.image.ColorModel;
import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.awt.event.InputEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import java.io.File;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.PrintWriter;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Timer;
import java.util.TimerTask;

/**
 * Main frame for graphical offline motion profile generator
 *
 * @author Tiger Huang
 */
class MainFrame extends JFrame {
	// Reference to path planning panel
	PathPanel pathPanel;

	// Shared variables
	EditState editState;
	JComboBox<String> editSelector;
	JSplitPane hsplitLarge;
	JSplitPane hsplitSmall;

	// File choosers
	JFileChooser waypointChooser;
	JFileChooser profileChooser;
	JFileChooser imageChooser;

	// Internal variables
	private JLabel status0;
	private JLabel status1;
	private CardLayout cardLayout;
	private JPanel cardPanel;
	private JRadioButtonMenuItem inspect;
	private JRadioButtonMenuItem editWaypoint;
	private JRadioButtonMenuItem editLimit;
	private JSpinner editX;
	private JSpinner editY;
	private JSpinner editVT;
	private JSpinner editVM;
	private JSpinner editAT;
	private JSpinner editAM;
	private JSpinner editT1;
	private JSpinner editT2;
	private JComboBox<String> editType;
	private JSpinner editLim;
	boolean suppress;

	// Inspect
	private JLabel freeX1;
	private JLabel freeY1;
	private JLabel pathX1;
	private JLabel pathY1;
	private JLabel pathTime1;
	private JLabel linPos1;
	private JLabel linVel1;
	private JLabel linAcc1;
	private JLabel angPos1;
	private JLabel angVel1;
	private JLabel angAcc1;
	private JLabel leftVol1;
	private JLabel rightVol1;
	private JLabel freeX2;
	private JLabel freeY2;
	private JLabel pathX2;
	private JLabel pathY2;
	private JLabel pathTime2;
	private JLabel linPos2;
	private JLabel linVel2;
	private JLabel linAcc2;
	private JLabel angPos2;
	private JLabel angVel2;
	private JLabel angAcc2;
	private JLabel leftVol2;
	private JLabel rightVol2;
	private JLabel freeX3;
	private JLabel freeY3;
	private JLabel freeHypot3;
	private JLabel pathX3;
	private JLabel pathY3;
	private JLabel pathHypot3;
	private JLabel pathTime3;
	private JLabel linPos3;
	private JLabel linVel3;
	private JLabel linAcc3;
	private JLabel angPos3;
	private JLabel angVel3;
	private JLabel angAcc3;
	private JLabel leftVol3;
	private JLabel rightVol3;

	// Settings
	private JSpinner settingsTimeStep;
	private JSpinner settingsSegmentLength;
	private JSpinner settingsVelocityInitial;
	private JSpinner settingsVelocityFinal;
	private JSpinner settingsPlanSmoothing;
	private JSpinner settingsMaxIterations;
	private JSpinner settingsAdjustScale;
	private JSpinner settingsAdjustOffset;
	private JSpinner settingsFilterLength;
	private JSpinner settingsWheelbase;
	private JSpinner settingsWidth;
	private JSpinner settingsLength;
	private JSpinner settingsLinVelMax;
	private JSpinner settingsLinVelTau;
	private JSpinner settingsAngVelMax;
	private JSpinner settingsAngVelTau;
	private JSpinner settingsImageScale;
	private JSpinner settingsImageBrightness;
	private JSpinner settingsZoomScaleFactor;
	private JSpinner settingsIntervalLength;
	private JSpinner settingsTangentAngle;
	private JSpinner settingsTangentMagnitude;
	private JSpinner settingsCurvatureAngle;
	private JSpinner settingsCurvatureMagnitude;
	private JSpinner settingsDisplaySegments;
	private JSpinner settingsTangentScale;
	private JSpinner settingsCurvatureScale;
	private JSpinner settingsWidgetSize;

	// Graphs
	GraphPanel linearGraph;
	GraphPanel angularGraph;
	GraphPanel sideGraph;
	private JLabel rangeTime;
	private JLabel rangeLinPos;
	private JLabel rangeLinVel;
	private JLabel rangeLinAcc;
	private JLabel rangeAngPos;
	private JLabel rangeAngVel;
	private JLabel rangeAngAcc;
	private JLabel rangeLeftVol;
	private JLabel rangeRightVol;
	private JLabel minLinPos;
	private JLabel minLinVel;
	private JLabel minLinAcc;
	private JLabel minAngPos;
	private JLabel minAngVel;
	private JLabel minAngAcc;
	private JLabel minLeftVol;
	private JLabel minRightVol;
	private JLabel maxLinPos;
	private JLabel maxLinVel;
	private JLabel maxLinAcc;
	private JLabel maxAngPos;
	private JLabel maxAngVel;
	private JLabel maxAngAcc;
	private JLabel maxLeftVol;
	private JLabel maxRightVol;

	/**
	 * Constructor
	 */
	public MainFrame() {
		// File choosers
		initFileChoosers();

		// Menu bar
		setJMenuBar(initMenuBar());

		// Content panels
		pathPanel = new PathPanel(this);
		pathPanel.setFocusable(true);

		// Vertical splits
		JSplitPane vsplitLeft = new JSplitPane(JSplitPane.VERTICAL_SPLIT, true, initEditPanel(), initSettingsPanel());
		JSplitPane vsplitRight = new JSplitPane(JSplitPane.VERTICAL_SPLIT, true, initGraphsPanel(), initGraphsDescriptionPanel());
		vsplitLeft.setOneTouchExpandable(true);
		vsplitRight.setOneTouchExpandable(true);
		vsplitLeft.setResizeWeight(1.0);
		vsplitRight.setResizeWeight(1.0);

		// Left, center, right panes
		hsplitSmall = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, true, vsplitLeft, pathPanel);
		hsplitLarge = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, true, hsplitSmall, vsplitRight);
		hsplitSmall.setOneTouchExpandable(true);
		hsplitLarge.setOneTouchExpandable(true);
		hsplitSmall.setResizeWeight(0.0);
		hsplitLarge.setResizeWeight(1.0);
		setContentPane(hsplitLarge);

		// Frame initialization
		setExtendedState(JFrame.MAXIMIZED_BOTH);
		setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
		setTitle("Offline Motion Profile Generator Version 2.0");
		setVisible(true);
		pack();

		// Handle default splits
		Timer timer = new Timer();
		timer.schedule(new TimerTask() {
			@Override
			public void run() {
				SwingUtilities.invokeLater(new Runnable() {
					@Override
					public void run() {
						vsplitLeft.setDividerLocation(vsplitLeft.getMinimumDividerLocation());
						setReady(true);
					}
				});
			}
		}, 100);
	}

	/**
	 * File choosers
	 */
	private void initFileChoosers() {
		FileNameExtensionFilter waypointFilter = new FileNameExtensionFilter("CSV File", "csv");
		FileNameExtensionFilter imageFilter = new FileNameExtensionFilter("All Supported Images", ImageIO.getReaderFileSuffixes());
		waypointChooser = new JFileChooser();
		waypointChooser.setFileFilter(waypointFilter);
		profileChooser = new JFileChooser();
		profileChooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
		imageChooser = new JFileChooser();
		imageChooser.setFileFilter(imageFilter);
	}

	/**
	 * Menu bar
	 */
	private JMenuBar initMenuBar() {
		// Create buttons
		JMenuItem newButton = new JMenuItem("New");
		JMenuItem openButton = new JMenuItem("Open");
		JMenuItem openImageButton = new JMenuItem("Open Image");
		JMenuItem saveButton = new JMenuItem("Save");
		JMenuItem saveAsButton = new JMenuItem("Save As");
		JMenuItem exportButton = new JMenuItem("Export");
		JMenuItem exportAsButton = new JMenuItem("Export As");
		JMenuItem removeImageButton = new JMenuItem("Remove Image");
		JMenuItem exitButton = new JMenuItem("Exit");

		// Add accelerators
		newButton.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_N, InputEvent.CTRL_DOWN_MASK));
		openButton.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_O, InputEvent.CTRL_DOWN_MASK));
		openImageButton.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_I, InputEvent.CTRL_DOWN_MASK));
		saveButton.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_S, InputEvent.CTRL_DOWN_MASK));
		saveAsButton.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_S, InputEvent.CTRL_DOWN_MASK | InputEvent.SHIFT_DOWN_MASK));
		exportButton.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_E, InputEvent.CTRL_DOWN_MASK));
		exportAsButton.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_E, InputEvent.CTRL_DOWN_MASK | InputEvent.SHIFT_DOWN_MASK));
		removeImageButton.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_K, InputEvent.CTRL_DOWN_MASK));
		exitButton.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_Q, InputEvent.CTRL_DOWN_MASK));

		// Create window listener
		MainFrame self = this;
		WindowAdapter windowListener = new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent e) {
				if(pathPanel.dirty) {
					int result = JOptionPane.showConfirmDialog(self, "Waypoints not saved.\nSave them now?");
					if(result == JOptionPane.YES_OPTION) {
						saveButton.doClick();
						windowClosing(e);
					}
					else if(result == JOptionPane.NO_OPTION) {
						pathPanel.dirty = false;
						windowClosing(e);
					}
				}
				else {
					System.exit(0);
				}
			}
		};
		self.addWindowListener(windowListener);

		// Create listener
		ActionListener listenerActions = new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				JMenuItem source = (JMenuItem)e.getSource();

				if(source == newButton) {
					if(pathPanel.dirty) {
						int result = JOptionPane.showConfirmDialog(self, "Waypoints not saved.\nSave them now?");
						if(result == JOptionPane.YES_OPTION) {
							saveButton.doClick();
							newButton.doClick();
						}
						else if(result == JOptionPane.NO_OPTION) {
							pathPanel.dirty = false;
							newButton.doClick();
						}
					}
					else {
						// New
						pathPanel.scale = 49;
						pathPanel.offx = 0;
						pathPanel.offy = 0;
						pathPanel.inspectX1 = Double.NaN;
						pathPanel.inspectY1 = Double.NaN;
						pathPanel.inspectU1 = Double.NaN;
						pathPanel.inspectX2 = Double.NaN;
						pathPanel.inspectY2 = Double.NaN;
						pathPanel.inspectU2 = Double.NaN;
						pathPanel.waypointPath = "";
						pathPanel.profilePath = "";
						pathPanel.imagePath = "";
						pathPanel.rawImage = null;
						pathPanel.brightenedImage = null;
						pathPanel.waypoints.clear();
						pathPanel.limits.clear();
						pathPanel.recalculateSplines();
						pathPanel.recalculateProfile();
						regenerateEditSelector();
						updateInspect();
						linearGraph.repaint();
						angularGraph.repaint();
						sideGraph.repaint();
						pathPanel.repaint();
						pathPanel.dirty = false;
					}
				}

				else if(source == openButton) {
					if(pathPanel.dirty) {
						int result = JOptionPane.showConfirmDialog(self, "Waypoints not saved.\nSave them now?");
						if(result == JOptionPane.YES_OPTION) {
							saveButton.doClick();
							openButton.doClick();
						}
						else if(result == JOptionPane.NO_OPTION) {
							pathPanel.dirty = false;
							openButton.doClick();
						}
					}
					else {
						int result = waypointChooser.showDialog(self, "Open Waypoints");
						if(result == JOptionPane.YES_OPTION) {
							newButton.doClick();
							// Open
							setReady(false);
							SwingUtilities.invokeLater(new Runnable() {
								@Override
								public void run() {
									File file = waypointChooser.getSelectedFile();
									if(file.exists() && file.isFile()) {
										pathPanel.waypointPath = file.getAbsolutePath();
										BufferedReader in = null;
										try {
											in = new BufferedReader(new FileReader(file));
										}
										catch(FileNotFoundException ex) {
											JOptionPane.showMessageDialog(self, "Unable to find waypoints");
										}
										if(in != null) {
											String line = null;
											for(;;) {
												try {
													line = in.readLine();
												}
												catch(IOException ex) {
													JOptionPane.showMessageDialog(self, "Unable to read waypoints");
													break;
												}
												if(line == null) {
													break;
												}
												String[] parts = line.split(",");
												for(int i = 0; i < parts.length; i++) {
													parts[i] = parts[i].trim();
												}
												if(parts.length >= 1) {
													switch(parts[0]) {
														case "generation":
															if(parts.length > 9) {
																pathPanel.timeStep = Double.valueOf(parts[1]);
																pathPanel.segmentLength = Double.valueOf(parts[2]);
																pathPanel.v_initial = Double.valueOf(parts[3]);
																pathPanel.v_final = Double.valueOf(parts[4]);
																pathPanel.smoothing = Double.valueOf(parts[5]);
																pathPanel.max_iterations = Integer.valueOf(parts[6]);
																pathPanel.adjust_scale = Double.valueOf(parts[7]);
																pathPanel.adjust_offset = Double.valueOf(parts[8]);
																pathPanel.filter_length = Integer.valueOf(parts[9]);
															}
															break;
														case "robot":
															if(parts.length > 7) {
																pathPanel.r_wb = Double.valueOf(parts[1]);
																pathPanel.r_ow = Double.valueOf(parts[2]);
																pathPanel.r_ol = Double.valueOf(parts[3]);
																pathPanel.v_max = Double.valueOf(parts[4]);
																pathPanel.v_tau = Double.valueOf(parts[5]);
																pathPanel.w_max = Math.PI/180*Double.valueOf(parts[6]);
																pathPanel.w_tau = Double.valueOf(parts[7]);
															}
															break;
														case "image":
															if(parts.length > 2) {
																pathPanel.imageScale = Double.valueOf(parts[1]);
																pathPanel.imageBrightness = Double.valueOf(parts[2]);
															}
															break;
														case "editor":
															if(parts.length > 1) {
																pathPanel.scaleFactor = Double.valueOf(parts[1]);
															}
															break;
														case "limit_settings":
															if(parts.length > 1) {
																pathPanel.default_length = Double.valueOf(parts[1]);
															}
															break;
														case "waypoint_settings":
															if(parts.length > 4) {
																pathPanel.default_v_t = Math.PI/180*Double.valueOf(parts[1]);
																pathPanel.default_v_m = Double.valueOf(parts[2]);
																pathPanel.default_a_t = Math.PI/180*Double.valueOf(parts[3]);
																pathPanel.default_a_m = Double.valueOf(parts[4]);
															}
															break;
														case "display":
															if(parts.length > 4) {
																pathPanel.displaySegments = Integer.valueOf(parts[1]);
																pathPanel.v_scale = Double.valueOf(parts[2]);
																pathPanel.a_scale = Double.valueOf(parts[3]);
																pathPanel.widgetSize = Double.valueOf(parts[4]);
															}
															break;
														case "image_path":
															if(parts.length > 1) {
																Path waypointPath = (new File(pathPanel.waypointPath)).toPath().getParent();
																pathPanel.imagePath = waypointPath.resolve(
																		parts[1].replace('/', File.separatorChar).replace('\\', File.separatorChar)).normalize().toString();
																imageChooser.setSelectedFile(new File(pathPanel.imagePath));
																// Open image
																File file2 = new File(pathPanel.imagePath);
																if(file2.exists() && file2.isFile()) {
																	try {
																		// Read image and convert colorspace
																		BufferedImage image = ImageIO.read(file2);
																		pathPanel.rawImage = new BufferedImage(image.getWidth(), image.getHeight(), BufferedImage.TYPE_INT_RGB);
																		ColorConvertOp colorOp = new ColorConvertOp(null);
																		colorOp.filter(image, pathPanel.rawImage);
																		// Process image
																		processImage(true);
																		pathPanel.repaint();
																	}
																	catch(IOException ex) {
																		JOptionPane.showMessageDialog(self, "Unable to open image");
																	}
																}
																else {
																	JOptionPane.showMessageDialog(self, "Unable to find image");
																}
															}
															break;
														case "profile_path":
															if(parts.length > 1) {
																Path waypointPath = (new File(pathPanel.waypointPath)).toPath().getParent();
																pathPanel.profilePath = waypointPath.resolve(
																		parts[1].replace('/', File.separatorChar).replace('\\', File.separatorChar)).normalize().toString();
																profileChooser.setSelectedFile(new File(pathPanel.profilePath));
															}
															break;
														case "waypoint":
															if(parts.length > 6) {
																Waypoint waypoint = new Waypoint();
																waypoint.x = Double.valueOf(parts[1]);
																waypoint.y = Double.valueOf(parts[2]);
																waypoint.v_t = Math.PI/180*Double.valueOf(parts[3]);
																waypoint.v_m = Double.valueOf(parts[4]);
																waypoint.a_t = Math.PI/180*Double.valueOf(parts[5]);
																waypoint.a_m = Double.valueOf(parts[6]);
																pathPanel.waypoints.add(waypoint);
															}
															break;
														case "limit":
															if(parts.length > 4) {
																Limit limit = new Limit();
																limit.t1 = Double.valueOf(parts[1]);
																limit.t2 = Double.valueOf(parts[2]);
																limit.type = LimitType.valueOf(parts[3]);
																limit.limit = Double.valueOf(parts[4]);
																if(limit.t1 >= pathPanel.waypoints.size()-1) {
																	limit.t1 = Math.nextDown(pathPanel.waypoints.size()-1);
																}
																if(limit.t2 >= pathPanel.waypoints.size()-1) {
																	limit.t2 = Math.nextDown(pathPanel.waypoints.size()-1);
																}
																if(pathPanel.limits.size() == 0) {
																	pathPanel.firstLimit = limit;
																}
																pathPanel.limits.add(limit);
															}
															break;
														default:
															System.out.println("Unknown record");
															break;
													}
												}
											}
											try {
												in.close();
											}
											catch(IOException ex) {
												JOptionPane.showMessageDialog(self, "Unable to read waypoints");
											}
											// Update UI
											settingsTimeStep.setValue(pathPanel.timeStep);
											settingsSegmentLength.setValue(pathPanel.segmentLength);
											settingsVelocityInitial.setValue(pathPanel.v_initial);
											settingsVelocityFinal.setValue(pathPanel.v_final);
											settingsPlanSmoothing.setValue(pathPanel.smoothing);
											settingsMaxIterations.setValue(pathPanel.max_iterations);
											settingsAdjustScale.setValue(pathPanel.adjust_scale);
											settingsAdjustOffset.setValue(pathPanel.adjust_offset);
											settingsFilterLength.setValue(pathPanel.filter_length);
											settingsWheelbase.setValue(pathPanel.r_wb);
											settingsWidth.setValue(pathPanel.r_ow);
											settingsLength.setValue(pathPanel.r_ol);
											settingsLinVelMax.setValue(pathPanel.v_max);
											settingsLinVelTau.setValue(pathPanel.v_tau);
											settingsAngVelMax.setValue(pathPanel.w_max*180/Math.PI);
											settingsAngVelTau.setValue(pathPanel.w_tau);
											settingsImageScale.setValue(pathPanel.imageScale);
											settingsImageBrightness.setValue(pathPanel.imageBrightness);
											settingsZoomScaleFactor.setValue(pathPanel.scaleFactor);
											settingsIntervalLength.setValue(pathPanel.default_length);
											settingsTangentAngle.setValue(pathPanel.default_v_t*180/Math.PI);
											settingsTangentMagnitude.setValue(pathPanel.default_v_m);
											settingsCurvatureAngle.setValue(pathPanel.default_a_t*180/Math.PI);
											settingsCurvatureMagnitude.setValue(pathPanel.default_a_m);
											settingsDisplaySegments.setValue(pathPanel.displaySegments);
											settingsTangentScale.setValue(pathPanel.v_scale);
											settingsCurvatureScale.setValue(pathPanel.a_scale);
											settingsWidgetSize.setValue(pathPanel.widgetSize);
											// Calculate profiles
											pathPanel.recalculateSplines();
											pathPanel.recalculateProfile();
											regenerateEditSelector();
											updateInspect();
											linearGraph.repaint();
											angularGraph.repaint();
											sideGraph.repaint();
											pathPanel.repaint();
											pathPanel.dirty = false;
										}
									}
									else {
										JOptionPane.showMessageDialog(self, "Unable to find waypoints");
									}
								}
							});
							SwingUtilities.invokeLater(new Runnable() {
								@Override
								public void run() {
									setReady(true);
								}
							});
						}
					}
				}

				else if(source == openImageButton) {
					int result = imageChooser.showDialog(self, "Open Image");
					if(result == JFileChooser.APPROVE_OPTION) {
						// Open image
						setReady(false);
						SwingUtilities.invokeLater(new Runnable() {
							@Override
							public void run() {
								File file = imageChooser.getSelectedFile();
								if(file.exists() && file.isFile()) {
									try {
										// Read image and convert colorspace
										pathPanel.imagePath = file.getAbsolutePath();
										BufferedImage image = ImageIO.read(file);
										pathPanel.rawImage = new BufferedImage(image.getWidth(), image.getHeight(), BufferedImage.TYPE_INT_RGB);
										ColorConvertOp colorOp = new ColorConvertOp(null);
										colorOp.filter(image, pathPanel.rawImage);
										// Process image
										processImage(true);
										pathPanel.repaint();
										pathPanel.dirty = true;
									}
									catch(IOException ex) {
										JOptionPane.showMessageDialog(self, "Unable to open image");
									}
								}
								else {
									JOptionPane.showMessageDialog(self, "Unable to find image");
								}
								setReady(true);
							}
						});
					}
				}

				else if(source == saveButton) {
					if(pathPanel.waypointPath.isEmpty()) {
						saveAsButton.doClick();
					}
					else {
						// Saving
						PrintWriter out = null;
						try {
							out = new PrintWriter(new BufferedWriter(new FileWriter(pathPanel.waypointPath)));
						}
						catch(IOException ex) {
							JOptionPane.showMessageDialog(self, "Unable to write waypoints");
						}
						if(out != null) {
							out.format("generation,%f,%f,%f,%f,%f,%d,%f,%f,%d\n",
									pathPanel.timeStep, pathPanel.segmentLength, pathPanel.v_initial,
									pathPanel.v_final, pathPanel.smoothing, pathPanel.max_iterations,
									pathPanel.adjust_scale, pathPanel.adjust_offset, pathPanel.filter_length);
							out.format("robot,%f,%f,%f,%f,%f,%f,%f\n", pathPanel.r_wb,
									pathPanel.r_ow, pathPanel.r_ol, pathPanel.v_max,
									pathPanel.v_tau, 180/Math.PI*pathPanel.w_max, pathPanel.w_tau);
							out.format("image,%f,%f\n", pathPanel.imageScale, pathPanel.imageBrightness);
							out.format("editor,%f\n", pathPanel.scaleFactor);
							out.format("limit_settings,%f\n", pathPanel.default_length);
							out.format("waypoint_settings,%f,%f,%f,%f\n", 180/Math.PI*pathPanel.default_v_t,
									pathPanel.default_v_m, 180/Math.PI*pathPanel.default_a_t, pathPanel.default_a_m);
							out.format("display,%d,%f,%f,%f\n", pathPanel.displaySegments,
									pathPanel.v_scale, pathPanel.a_scale, pathPanel.widgetSize);
							Path waypointPath = (new File(pathPanel.waypointPath)).toPath().getParent();
							out.format("image_path,%s\n", pathPanel.imagePath.isEmpty() ? "" :
									"." + File.separator + waypointPath.relativize((new File(pathPanel.imagePath).toPath())));
							out.format("profile_path,%s\n", pathPanel.profilePath.isEmpty() ? "" :
									"." + File.separator + waypointPath.relativize((new File(pathPanel.profilePath)).toPath()));
							for(Waypoint waypoint : pathPanel.waypoints) {
								out.format("waypoint,%f,%f,%f,%f,%f,%f\n",
										waypoint.x, waypoint.y, 180/Math.PI*waypoint.v_t,
										waypoint.v_m, 180/Math.PI*waypoint.a_t, waypoint.a_m);
							}
							for(Limit limit : pathPanel.limits) {
								out.format("limit,%f,%f,%s,%f\n",
										limit.t1, limit.t2, limit.type, limit.limit);
							}
							out.close();
							pathPanel.dirty = false;
						}
					}
				}

				else if(source == saveAsButton) {
					int result = waypointChooser.showDialog(self, "Save Waypoints");
					if(result == JFileChooser.APPROVE_OPTION) {
						FileFilter filter = waypointChooser.getFileFilter();
						File file = waypointChooser.getSelectedFile();
						String path = null;
						if(!(filter instanceof FileNameExtensionFilter) || filter.accept(file)) {
							path = file.getAbsolutePath();
						}
						else {
							path = file.getAbsolutePath() + "." + ((FileNameExtensionFilter)filter).getExtensions()[0];
						}
						if((new File(path)).exists()) {
							result = JOptionPane.showConfirmDialog(self, "File exists. Overwrite it?");
							if(result == JOptionPane.YES_OPTION) {
								pathPanel.waypointPath = path;
								saveButton.doClick();
							}
						}
						else {
							pathPanel.waypointPath = path;
							saveButton.doClick();
						}
					}
				}

				else if(source == exportButton) {
					if(pathPanel.profilePath.isEmpty()) {
						exportAsButton.doClick();
					}
					else {
						File file = new File(pathPanel.profilePath);
						if(!file.exists()) {
							int result = JOptionPane.showConfirmDialog(self, "Directory does not exist. Create it?");
							if(result == JOptionPane.YES_OPTION) {
								file.mkdirs();
								exportButton.doClick();
							}
						}
						else {
							if(pathPanel.timeData == null) {
								JOptionPane.showMessageDialog(self, "No profile to export");
							}
							else {
								// Export
								String baseName = new File(pathPanel.waypointPath).getName();
								int position = baseName.lastIndexOf(".");
								if(position > 0) {
									baseName = baseName.substring(0, position);
								}
								Path path = file.toPath();
								// Forward, normal
								PrintWriter out = null;
								try {
									out = new PrintWriter(new BufferedWriter(new FileWriter(path.resolve(baseName + "_fwd_nor.csv").toString())));
								}
								catch(IOException ex) {
									JOptionPane.showMessageDialog(self, "Unable to write forward normal profile");
								}
								if(out != null) {
									out.format("%d, %d\n", 2, pathPanel.timeData[0].length);
									for(int i = 0; i < pathPanel.timeData[0].length; i++) {
										out.format("%f, %f, %f, %f, %f, %f\n",
												pathPanel.timeData[0][i], pathPanel.timeData[1][i], pathPanel.timeData[2][i],
												pathPanel.timeData[3][i], pathPanel.timeData[4][i], pathPanel.timeData[5][i]);
									}
									out.close();
								}
								// Forward, mirrored
								out = null;
								try {
									out = new PrintWriter(new BufferedWriter(new FileWriter(path.resolve(baseName + "_fwd_mir.csv").toString())));
								}
								catch(IOException ex) {
									JOptionPane.showMessageDialog(self, "Unable to write forward mirrored profile");
								}
								if(out != null) {
									out.format("%d, %d\n", 2, pathPanel.timeData[0].length);
									for(int i = 0; i < pathPanel.timeData[0].length; i++) {
										out.format("%f, %f, %f, %f, %f, %f\n",
												pathPanel.timeData[0][i], pathPanel.timeData[1][i], pathPanel.timeData[2][i],
												-pathPanel.timeData[3][i], -pathPanel.timeData[4][i], -pathPanel.timeData[5][i]);
									}
									out.close();
								}
								// Reverse, normal
								out = null;
								try {
									out = new PrintWriter(new BufferedWriter(new FileWriter(path.resolve(baseName + "_rev_nor.csv").toString())));
								}
								catch(IOException ex) {
									JOptionPane.showMessageDialog(self, "Unable to write reverse normal profile");
								}
								if(out != null) {
									out.format("%d, %d\n", 2, pathPanel.timeData[0].length);
									for(int i = 0; i < pathPanel.timeData[0].length; i++) {
										out.format("%f, %f, %f, %f, %f, %f\n",
												-pathPanel.timeData[0][i], -pathPanel.timeData[1][i], -pathPanel.timeData[2][i],
												pathPanel.timeData[3][i], pathPanel.timeData[4][i], pathPanel.timeData[5][i]);
									}
									out.close();
								}
								// Reverse, mirrored
								out = null;
								try {
									out = new PrintWriter(new BufferedWriter(new FileWriter(path.resolve(baseName + "_rev_mir.csv").toString())));
								}
								catch(IOException ex) {
									JOptionPane.showMessageDialog(self, "Unable to write reverse mirrored profile");
								}
								if(out != null) {
									out.format("%d, %d\n", 2, pathPanel.timeData[0].length);
									for(int i = 0; i < pathPanel.timeData[0].length; i++) {
										out.format("%f, %f, %f, %f, %f, %f\n",
												-pathPanel.timeData[0][i], -pathPanel.timeData[1][i], -pathPanel.timeData[2][i],
												-pathPanel.timeData[3][i], -pathPanel.timeData[4][i], -pathPanel.timeData[5][i]);
									}
									out.close();
								}
								JOptionPane.showMessageDialog(self, "Profiles exported");
							}
						}
					}
				}

				else if(source == exportAsButton) {
					if(pathPanel.waypointPath.isEmpty()) {
						JOptionPane.showMessageDialog(self, "Please save waypoints before exporting profiles");
					}
					else {
						int result = profileChooser.showDialog(self, "Export Profiles");
						if(result == JFileChooser.APPROVE_OPTION) {
							pathPanel.profilePath = profileChooser.getSelectedFile().getAbsolutePath();
							pathPanel.dirty = true;
							exportButton.doClick();
						}
					}
				}

				else if(source == removeImageButton) {
					// Removal
					pathPanel.rawImage = null;
					pathPanel.brightenedImage = null;
					pathPanel.imagePath = "";
					pathPanel.repaint();
					pathPanel.dirty = true;
				}

				else if(e.getSource() == exitButton) {
					windowListener.windowClosing(null);
				}
			}
		};

		// Add listener
		newButton.addActionListener(listenerActions);
		openButton.addActionListener(listenerActions);
		openImageButton.addActionListener(listenerActions);
		saveButton.addActionListener(listenerActions);
		saveAsButton.addActionListener(listenerActions);
		exportButton.addActionListener(listenerActions);
		exportAsButton.addActionListener(listenerActions);
		removeImageButton.addActionListener(listenerActions);
		exitButton.addActionListener(listenerActions);

		// Create buttons
		inspect = new JRadioButtonMenuItem("Inspect", true);
		JRadioButtonMenuItem pan = new JRadioButtonMenuItem("Pan");
		JRadioButtonMenuItem zoom = new JRadioButtonMenuItem("Zoom");
		JRadioButtonMenuItem addWaypoint = new JRadioButtonMenuItem("Add Waypoint");
		editWaypoint = new JRadioButtonMenuItem("Edit Waypoint");
		JRadioButtonMenuItem deleteWaypoint = new JRadioButtonMenuItem("Delete Waypoint");
		JRadioButtonMenuItem addLimit = new JRadioButtonMenuItem("Add Limit");
		editLimit = new JRadioButtonMenuItem("Edit Limit");
		JRadioButtonMenuItem deleteLimit = new JRadioButtonMenuItem("Delete Limit");

		// Add accelerators
		inspect.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_F1, 0));
		pan.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_F2, 0));
		zoom.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_F3, 0));
		addWaypoint.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_A, 0));
		editWaypoint.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_E, 0));
		deleteWaypoint.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_D, 0));
		addLimit.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_A, InputEvent.SHIFT_DOWN_MASK));
		editLimit.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_E, InputEvent.SHIFT_DOWN_MASK));
		deleteLimit.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_D, InputEvent.SHIFT_DOWN_MASK));

		// Assign to single group
		ButtonGroup group = new ButtonGroup();
		group.add(inspect);
		group.add(pan);
		group.add(zoom);
		group.add(addWaypoint);
		group.add(editWaypoint);
		group.add(deleteWaypoint);
		group.add(addLimit);
		group.add(editLimit);
		group.add(deleteLimit);

		// Create listener
		editState = EditState.INSPECT;
		ActionListener listenerStates = new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				JRadioButtonMenuItem source = (JRadioButtonMenuItem)e.getSource();
				if(source == inspect) {
					editState = EditState.INSPECT;
					suppress = true;
					editSelector.setSelectedIndex(0);
					suppress = false;
				}
				else if(source == pan) {
					editState = EditState.PAN;
				}
				else if(source == zoom) {
					editState = EditState.ZOOM;
				}
				else if(source == addWaypoint) {
					editState = EditState.ADDWAYPOINT;
				}
				else if(source == editWaypoint) {
					editState = EditState.EDITWAYPOINT;
				}
				else if(source == deleteWaypoint) {
					editState = EditState.DELETEWAYPOINT;
				}
				else if(source == addLimit) {
					editState = EditState.ADDLIMIT;
				}
				else if(source == editLimit) {
					editState = EditState.EDITLIMIT;
				}
				else if(source == deleteLimit) {
					editState = EditState.DELETELIMIT;
				}
				status0.setText(" [" + source.getText() + "] ");
				updateInspect();
				pathPanel.repaint();
				linearGraph.repaint();
				angularGraph.repaint();
				sideGraph.repaint();
			}
		};

		// Add listener
		inspect.addActionListener(listenerStates);
		pan.addActionListener(listenerStates);
		zoom.addActionListener(listenerStates);
		addWaypoint.addActionListener(listenerStates);
		editWaypoint.addActionListener(listenerStates);
		deleteWaypoint.addActionListener(listenerStates);
		addLimit.addActionListener(listenerStates);
		editLimit.addActionListener(listenerStates);
		deleteLimit.addActionListener(listenerStates);

		// Create status fields
		status0 = new JLabel(" [Inspect] ");
		status1 = new JLabel();
		setReady(false);

		// Create menu bar
		JMenuBar bar = new JMenuBar();
		JMenu file = new JMenu("File");
		file.add(newButton);
		file.add(openButton);
		file.add(openImageButton);
		file.add(saveButton);
		file.add(saveAsButton);
		file.add(exportButton);
		file.add(exportAsButton);
		file.add(removeImageButton);
		file.add(exitButton);
		bar.add(file);
		JMenu navigate = new JMenu("Navigate");
		navigate.add(inspect);
		navigate.add(pan);
		navigate.add(zoom);
		bar.add(navigate);
		JMenu waypoints = new JMenu("Waypoints");
		waypoints.add(addWaypoint);
		waypoints.add(editWaypoint);
		waypoints.add(deleteWaypoint);
		bar.add(waypoints);
		JMenu limits = new JMenu("Limits");
		limits.add(addLimit);
		limits.add(editLimit);
		limits.add(deleteLimit);
		bar.add(limits);
		JPanel spacer = new JPanel();
		spacer.setOpaque(false);
		bar.add(spacer);
		bar.add(status1);
		bar.add(status0);
		return bar;
	}

	/**
	 * Edit panel for waypoints and limits
	 */
	private JPanel initEditPanel() {
		// Edit selector
		editSelector = new JComboBox<>();
		regenerateEditSelector();

		// Create inspect outputs
		freeX1 = new JLabel("--");
		freeY1 = new JLabel("--");
		pathX1 = new JLabel("--");
		pathY1 = new JLabel("--");
		pathTime1 = new JLabel("--");
		linPos1 = new JLabel("--");
		linVel1 = new JLabel("--");
		linAcc1 = new JLabel("--");
		angPos1 = new JLabel("--");
		angVel1 = new JLabel("--");
		angAcc1 = new JLabel("--");
		leftVol1 = new JLabel("--");
		rightVol1 = new JLabel("--");
		freeX2 = new JLabel("--");
		freeY2 = new JLabel("--");
		pathX2 = new JLabel("--");
		pathY2 = new JLabel("--");
		pathTime2 = new JLabel("--");
		linPos2 = new JLabel("--");
		linVel2 = new JLabel("--");
		linAcc2 = new JLabel("--");
		angPos2 = new JLabel("--");
		angVel2 = new JLabel("--");
		angAcc2 = new JLabel("--");
		leftVol2 = new JLabel("--");
		rightVol2 = new JLabel("--");
		freeX3 = new JLabel("--");
		freeY3 = new JLabel("--");
		freeHypot3 = new JLabel("--");
		pathX3 = new JLabel("--");
		pathY3 = new JLabel("--");
		pathHypot3 = new JLabel("--");
		pathTime3 = new JLabel("--");
		linPos3 = new JLabel("--");
		linVel3 = new JLabel("--");
		linAcc3 = new JLabel("--");
		angPos3 = new JLabel("--");
		angVel3 = new JLabel("--");
		angAcc3 = new JLabel("--");
		leftVol3 = new JLabel("--");
		rightVol3 = new JLabel("--");
		updateInspect();

		// Layout inspect
		JPanel inspectPanel = new JPanel(new GridBagLayout());
		GridBagConstraints c = new GridBagConstraints();
		c.anchor = GridBagConstraints.FIRST_LINE_START;
		c.fill = GridBagConstraints.BOTH;
		c.gridy = 1;
		inspectPanel.add(new JLabel("Free X (len):"), c);
		c.gridy++;
		inspectPanel.add(new JLabel("Free Y (len):"), c);
		c.gridy++;
		inspectPanel.add(new JLabel("Free Hypot (len):"), c);
		c.gridy++;
		inspectPanel.add(new JLabel("Path X (len):"), c);
		c.gridy++;
		inspectPanel.add(new JLabel("Path Y (len):"), c);
		c.gridy++;
		inspectPanel.add(new JLabel("Path Hypot (len):"), c);
		c.gridy++;
		inspectPanel.add(new JLabel("Path Time (len):"), c);
		c.gridy++;
		inspectPanel.add(new JLabel("Lin Pos (len):"), c);
		c.gridy++;
		inspectPanel.add(new JLabel("Lin Vel (len/s):"), c);
		c.gridy++;
		inspectPanel.add(new JLabel("Lin Acc (len/s\u00B2):"), c);
		c.gridy++;
		inspectPanel.add(new JLabel("Ang Pos (\u00B0):"), c);
		c.gridy++;
		inspectPanel.add(new JLabel("Ang Vel (\u00B0/s):"), c);
		c.gridy++;
		inspectPanel.add(new JLabel("Ang Acc (\u00B0/s\u00B2):"), c);
		c.gridy++;
		inspectPanel.add(new JLabel("Left Vol (V/V):"), c);
		c.gridy++;
		inspectPanel.add(new JLabel("Right Vol (V/V):"), c);

		// Mark 1
		c.gridy = 0;
		c.gridx++;
		inspectPanel.add(new JPanel(), c);
		c.gridx++;
		JLabel label = new JLabel("Mark 1");
		label.setForeground(Color.CYAN.darker());
		inspectPanel.add(label, c);
		c.gridy++;
		inspectPanel.add(freeX1, c);
		c.gridy++;
		inspectPanel.add(freeY1, c);
		c.gridy+=2;
		inspectPanel.add(pathX1, c);
		c.gridy++;
		inspectPanel.add(pathY1, c);
		c.gridy+=2;
		inspectPanel.add(pathTime1, c);
		c.gridy++;
		inspectPanel.add(linPos1, c);
		c.gridy++;
		inspectPanel.add(linVel1, c);
		c.gridy++;
		inspectPanel.add(linAcc1, c);
		c.gridy++;
		inspectPanel.add(angPos1, c);
		c.gridy++;
		inspectPanel.add(angVel1, c);
		c.gridy++;
		inspectPanel.add(angAcc1, c);
		c.gridy++;
		inspectPanel.add(leftVol1, c);
		c.gridy++;
		inspectPanel.add(rightVol1, c);

		// Mark 2
		c.gridy = 0;
		c.gridx++;
		inspectPanel.add(new JPanel(), c);
		c.gridx++;
		label = new JLabel("Mark 2");
		label.setForeground(Color.YELLOW.darker());
		inspectPanel.add(label, c);
		c.gridy++;
		inspectPanel.add(freeX2, c);
		c.gridy++;
		inspectPanel.add(freeY2, c);
		c.gridy+=2;
		inspectPanel.add(pathX2, c);
		c.gridy++;
		inspectPanel.add(pathY2, c);
		c.gridy+=2;
		inspectPanel.add(pathTime2, c);
		c.gridy++;
		inspectPanel.add(linPos2, c);
		c.gridy++;
		inspectPanel.add(linVel2, c);
		c.gridy++;
		inspectPanel.add(linAcc2, c);
		c.gridy++;
		inspectPanel.add(angPos2, c);
		c.gridy++;
		inspectPanel.add(angVel2, c);
		c.gridy++;
		inspectPanel.add(angAcc2, c);
		c.gridy++;
		inspectPanel.add(leftVol2, c);
		c.gridy++;
		inspectPanel.add(rightVol2, c);

		// Difference
		c.gridy = 0;
		c.gridx++;
		inspectPanel.add(new JPanel(), c);
		c.gridx++;
		label = new JLabel("Diff");
		label.setForeground(PathPanel.PURPLE);
		inspectPanel.add(label, c);
		c.gridy++;
		inspectPanel.add(freeX3, c);
		c.gridy++;
		inspectPanel.add(freeY3, c);
		c.gridy++;
		inspectPanel.add(freeHypot3, c);
		c.gridy++;
		inspectPanel.add(pathX3, c);
		c.gridy++;
		inspectPanel.add(pathY3, c);
		c.gridy++;
		inspectPanel.add(pathHypot3, c);
		c.gridy++;
		inspectPanel.add(pathTime3, c);
		c.gridy++;
		inspectPanel.add(linPos3, c);
		c.gridy++;
		inspectPanel.add(linVel3, c);
		c.gridy++;
		inspectPanel.add(linAcc3, c);
		c.gridy++;
		inspectPanel.add(angPos3, c);
		c.gridy++;
		inspectPanel.add(angVel3, c);
		c.gridy++;
		inspectPanel.add(angAcc3, c);
		c.gridy++;
		inspectPanel.add(leftVol3, c);
		c.gridy++;
		inspectPanel.add(rightVol3, c);
		c.gridy++;
		c.weighty = 1.0;
		c.weightx = 1.0;
		JPanel spacer = new JPanel();
		inspectPanel.add(spacer, c);

		// Create waypoint controls
		editX = new JSpinner(new SpinnerNumberModel(0.0, null, null, 0.1));
		editY = new JSpinner(new SpinnerNumberModel(0.0, null, null, 0.1));
		editVT = new JSpinner(new SpinnerNumberModel(0.0, null, null, 1.0));
		editVM = new JSpinner(new SpinnerNumberModel(0.0, null, null, 0.1));
		editAT = new JSpinner(new SpinnerNumberModel(0.0, null, null, 1.0));
		editAM = new JSpinner(new SpinnerNumberModel(0.0, null, null, 0.1));

		// Layout waypoint settings
		JPanel waypointPanel = new JPanel(new GridBagLayout());
		c.weightx = 0.0;
		c.weighty = 0.0;
		c.gridx = 0;
		c.gridy = 0;
		label = new JLabel("Position");
		label.setForeground(PathPanel.PURPLE);
		waypointPanel.add(label, c);
		c.gridy = 1;
		waypointPanel.add(new JLabel("X (len)"), c);
		c.gridy = 2;
		waypointPanel.add(new JLabel("Y (len)"), c);
		c.gridy = 3;
		c.gridwidth = 2;
		label = new JLabel("Tangent Vector");
		label.setForeground(PathPanel.PURPLE);
		waypointPanel.add(label, c);
		c.gridy = 4;
		c.gridwidth = 1;
		waypointPanel.add(new JLabel("Angle (\u00B0)"), c);
		c.gridy = 5;
		waypointPanel.add(new JLabel("Magnitude (len)"), c);
		c.gridy = 6;
		c.gridwidth = 2;
		label = new JLabel("Curvature Vector");
		label.setForeground(PathPanel.PURPLE);
		waypointPanel.add(label, c);
		c.gridy = 7;
		c.gridwidth = 1;
		waypointPanel.add(new JLabel("Angle (\u00B0)"), c);
		c.gridy = 8;
		waypointPanel.add(new JLabel("Magnitude (len)"), c);
		c.gridx = 1;
		c.gridy = 1;
		waypointPanel.add(editX, c);
		c.gridy = 2;
		waypointPanel.add(editY, c);
		c.gridy = 4;
		waypointPanel.add(editVT, c);
		c.gridy = 5;
		waypointPanel.add(editVM, c);
		c.gridy = 7;
		waypointPanel.add(editAT, c);
		c.gridy = 8;
		waypointPanel.add(editAM, c);
		c.gridy = 9;
		c.weightx = 1.0;
		c.weighty = 1.0;
		spacer = new JPanel();
		waypointPanel.add(spacer, c);

		// Create listener
		ChangeListener waypointListener = new ChangeListener() {
			@Override
			public void stateChanged(ChangeEvent e) {
				if(!suppress) {
					Waypoint waypoint = pathPanel.waypoints.get(pathPanel.currIndex-1);
					waypoint.x = ((Double)editX.getValue()).doubleValue();
					waypoint.y = ((Double)editY.getValue()).doubleValue();
					double v_t = ((Double)editVT.getValue()).doubleValue() * Math.PI/180;
					while(v_t < -Math.PI) {
						v_t += Math.PI*2;
					}
					while(v_t > Math.PI) {
						v_t -= Math.PI*2;
					}
					waypoint.v_t = v_t;
					waypoint.v_m = ((Double)editVM.getValue()).doubleValue();
					double a_t = ((Double)editAT.getValue()).doubleValue() * Math.PI/180;
					while(a_t < -Math.PI) {
						a_t += Math.PI*2;
					}
					while(a_t > Math.PI) {
						a_t -= Math.PI*2;
					}
					waypoint.a_t = a_t;
					waypoint.a_m = ((Double)editAM.getValue()).doubleValue();
					pathPanel.recalculateSplines();
					pathPanel.recalculateProfile();
					pathPanel.repaint();
					linearGraph.repaint();
					angularGraph.repaint();
					sideGraph.repaint();
				}
			}
		};

		// Add listener
		editX.addChangeListener(waypointListener);
		editY.addChangeListener(waypointListener);
		editVT.addChangeListener(waypointListener);
		editVM.addChangeListener(waypointListener);
		editAT.addChangeListener(waypointListener);
		editAM.addChangeListener(waypointListener);

		// Create limit controls
		editT1 = new JSpinner(new SpinnerNumberModel(0.0, null, null, 0.01));
		editT2 = new JSpinner(new SpinnerNumberModel(0.0, null, null, 0.01));
		editType = new JComboBox<>();
		editType.addItem("Voltage/Voltage Max");
		editType.addItem("Linear Velocity");
		editType.addItem("Linear Acceleration");
		editLim = new JSpinner(new SpinnerNumberModel(0.0, 0.0, null, 0.1));

		// Layout limit settings
		JPanel limitPanel = new JPanel(new GridBagLayout());
		c.weightx = 0.0;
		c.weighty = 0.0;
		c.gridx = 0;
		c.gridy = 0;
		limitPanel.add(new JLabel("Start (ul)"), c);
		c.gridy = 1;
		limitPanel.add(new JLabel("End (ul)"), c);
		c.gridy = 2;
		limitPanel.add(new JLabel("Type"), c);
		c.gridy = 3;
		limitPanel.add(new JLabel("Limit"), c);
		c.gridx = 1;
		c.gridy = 0;
		limitPanel.add(editT1, c);
		c.gridy = 1;
		limitPanel.add(editT2, c);
		c.gridy = 2;
		limitPanel.add(editType, c);
		c.gridy = 3;
		limitPanel.add(editLim, c);
		c.gridy = 4;
		c.weighty = 1.0;
		c.weightx = 1.0;
		spacer = new JPanel();
		limitPanel.add(spacer, c);

		// Create listener
		ChangeListener limitListener = new ChangeListener() {
			@Override
			public void stateChanged(ChangeEvent e) {
				if(!suppress) {
					Limit limit = pathPanel.limits.get(pathPanel.currIndex-1-pathPanel.waypoints.size());
					double t1 = ((Double)editT1.getValue()).doubleValue();
					double t2 = ((Double)editT2.getValue()).doubleValue();
					if(t1 < 0) {
						t1 = 0;
					}
					if(t1 >= pathPanel.waypoints.size()-1) {
						t1 = Math.nextDown(pathPanel.waypoints.size()-1);
					}
					if(t2 < 0) {
						t2 = 0;
					}
					if(t2 >= pathPanel.waypoints.size()-1) {
						t2 = Math.nextDown(pathPanel.waypoints.size()-1);
					}
					if(t1 <= t2) {
						limit.t1 = t1;
						limit.t2 = t2;
					}
					switch(editType.getSelectedIndex()) {
						case 0:
							limit.type = LimitType.VOLTAGE;
							break;
						case 1:
							limit.type = LimitType.VELOCITY;
							break;
						case 2:
							limit.type = LimitType.ACCELERATION;
							break;
					}
					limit.limit = ((Double)editLim.getValue()).doubleValue();
					pathPanel.recalculateProfile();
					pathPanel.repaint();
					linearGraph.repaint();
					angularGraph.repaint();
					sideGraph.repaint();
				}
			}
		};
		ActionListener limitActionListener = new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				limitListener.stateChanged(null);
			}
		};

		// Add listener
		editT1.addChangeListener(limitListener);
		editT2.addChangeListener(limitListener);
		editType.addActionListener(limitActionListener);
		editLim.addChangeListener(limitListener);

		JPanel container = new JPanel();
		container.setLayout(new BoxLayout(container, BoxLayout.PAGE_AXIS));
		container.add(editSelector);
		cardLayout = new CardLayout();
		cardPanel = new JPanel(cardLayout);
		cardPanel.add(inspectPanel, "inspect");
		cardPanel.add(waypointPanel, "waypoint");
		cardPanel.add(limitPanel, "limit");
		ActionListener editSelectorListener = new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				int index = editSelector.getSelectedIndex();
				if(index >= 0) {
					pathPanel.currIndex = index;
					updateEditPanel();
					updateInspect();
					pathPanel.repaint();
					linearGraph.repaint();
					angularGraph.repaint();
					sideGraph.repaint();
				}
			}
		};
		editSelector.addActionListener(editSelectorListener);
		container.add(cardPanel);
		return container;
	}

	/**
	 * Edit panel for settings
	 */
	private JScrollPane initSettingsPanel() {
		// Create setting controls
		settingsTimeStep           = new JSpinner(new SpinnerNumberModel(pathPanel.timeStep, 0.001, 1.0, 0.001));
		settingsSegmentLength      = new JSpinner(new SpinnerNumberModel(pathPanel.segmentLength, 0.001, 1.0, 0.001));
		settingsVelocityInitial    = new JSpinner(new SpinnerNumberModel(pathPanel.v_initial, 0.0, null, 0.1));
		settingsVelocityFinal      = new JSpinner(new SpinnerNumberModel(pathPanel.v_final, 0.0, null, 0.1));
		settingsPlanSmoothing      = new JSpinner(new SpinnerNumberModel(pathPanel.smoothing, 0.0, 1.0, 0.001));
		settingsMaxIterations      = new JSpinner(new SpinnerNumberModel(pathPanel.max_iterations, 0, null, 100));
		settingsAdjustScale        = new JSpinner(new SpinnerNumberModel(pathPanel.adjust_scale, 0.0, 1.0, 0.001));
		settingsAdjustOffset       = new JSpinner(new SpinnerNumberModel(pathPanel.adjust_offset, 0.9, 1.0, 0.001));
		settingsFilterLength       = new JSpinner(new SpinnerNumberModel(pathPanel.filter_length, 0, null, 1));
		settingsWheelbase          = new JSpinner(new SpinnerNumberModel(pathPanel.r_wb, 0.1, null, 0.1));
		settingsWidth              = new JSpinner(new SpinnerNumberModel(pathPanel.r_ow, 0.1, null, 0.1));
		settingsLength             = new JSpinner(new SpinnerNumberModel(pathPanel.r_ol, 0.1, null, 0.1));
		settingsLinVelMax          = new JSpinner(new SpinnerNumberModel(pathPanel.v_max, 0.01, null, 0.01));
		settingsLinVelTau          = new JSpinner(new SpinnerNumberModel(pathPanel.v_tau, 0.01, null, 0.01));
		settingsAngVelMax          = new JSpinner(new SpinnerNumberModel(pathPanel.w_max*180/Math.PI, 0.01, null, 0.01));
		settingsAngVelTau          = new JSpinner(new SpinnerNumberModel(pathPanel.w_tau, 0.01, null, 0.01));
		settingsImageScale         = new JSpinner(new SpinnerNumberModel(pathPanel.imageScale, 0.1, null, 0.1));
		settingsImageBrightness    = new JSpinner(new SpinnerNumberModel(pathPanel.imageBrightness, 0.0, 1.0, 0.1));
		settingsZoomScaleFactor    = new JSpinner(new SpinnerNumberModel(pathPanel.scaleFactor, 0.9, 1.1, 0.001));
		settingsIntervalLength     = new JSpinner(new SpinnerNumberModel(pathPanel.default_length, 0.1, null, 0.1));
		settingsTangentAngle       = new JSpinner(new SpinnerNumberModel(pathPanel.default_v_t*180/Math.PI, -180.0, 180.0, 1.0));
		settingsTangentMagnitude   = new JSpinner(new SpinnerNumberModel(pathPanel.default_v_m, 0.0, null, 0.1));
		settingsCurvatureAngle     = new JSpinner(new SpinnerNumberModel(pathPanel.default_a_t*180/Math.PI, -180.0, 180.0, 1.0));
		settingsCurvatureMagnitude = new JSpinner(new SpinnerNumberModel(pathPanel.default_a_m, 0.0, null, 0.1));
		settingsDisplaySegments    = new JSpinner(new SpinnerNumberModel(pathPanel.displaySegments, 1, null, 1));
		settingsTangentScale       = new JSpinner(new SpinnerNumberModel(pathPanel.v_scale, 0.01, null, 0.01));
		settingsCurvatureScale     = new JSpinner(new SpinnerNumberModel(pathPanel.a_scale, 0.01, null, 0.01));
		settingsWidgetSize         = new JSpinner(new SpinnerNumberModel(pathPanel.widgetSize, 1.0, null, 1.0));

		// Layout settings
		JPanel panel = new JPanel(new GridBagLayout());
		// Labels
		GridBagConstraints c = new GridBagConstraints();
		c.anchor = GridBagConstraints.FIRST_LINE_START;
		c.fill = GridBagConstraints.BOTH;
		c.gridx = 0;
		c.gridy = 0;
		c.gridwidth = 2;
		JLabel label = new JLabel("Generation Parameters");
		label.setForeground(PathPanel.PURPLE);
		panel.add(label, c);
		c.gridwidth = 1;
		c.gridy++;
		panel.add(new JLabel("Time Step (s)"), c);
		c.gridy++;
		panel.add(new JLabel("Segment Length (len)"), c);
		c.gridy++;
		panel.add(new JLabel("Initial Max Vel (len/s)"), c);
		c.gridy++;
		panel.add(new JLabel("Final Max Vel (len/s)"), c);
		c.gridy++;
		panel.add(new JLabel("Plan Smoothing (ul)"), c);
		c.gridy++;
		panel.add(new JLabel("Max Iterations (ul)"), c);
		c.gridy++;
		panel.add(new JLabel("Adjust Scale (ul)"), c);
		c.gridy++;
		panel.add(new JLabel("Adjust Offset (ul)"), c);
		c.gridy++;
		panel.add(new JLabel("Filter Length (ul)"), c);
		c.gridy++;
		c.gridwidth = 2;
		label = new JLabel("Robot Parameters");
		label.setForeground(PathPanel.PURPLE);
		panel.add(label, c);
		c.gridwidth = 1;
		c.gridy++;
		panel.add(new JLabel("Wheelbase (len)"), c);
		c.gridy++;
		panel.add(new JLabel("Width (len)"), c);
		c.gridy++;
		panel.add(new JLabel("Length (len)"), c);
		c.gridy++;
		panel.add(new JLabel("Lin Vel Max (len/s)"), c);
		c.gridy++;
		panel.add(new JLabel("Lin Vel Tau (s)"), c);
		c.gridy++;
		panel.add(new JLabel("Ang Vel Max (\u00B0/s)"), c);
		c.gridy++;
		panel.add(new JLabel("Ang Vel Tau (s)"), c);
		c.gridy++;
		c.gridwidth = 2;
		label = new JLabel("Image Settings");
		label.setForeground(PathPanel.PURPLE);
		panel.add(label, c);
		c.gridwidth = 1;
		c.gridy++;
		panel.add(new JLabel("Scale (px/len)"), c);
		c.gridy++;
		panel.add(new JLabel("Brightness (ul)"), c);
		c.gridy++;
		c.gridwidth = 2;
		label = new JLabel("Editor Settings");
		label.setForeground(PathPanel.PURPLE);
		panel.add(label, c);
		c.gridwidth = 1;
		c.gridy++;
		panel.add(new JLabel("Zoom Scale Factor (ul)"), c);
		c.gridy++;
		c.gridwidth = 2;
		label = new JLabel("Limit Defaults");
		label.setForeground(PathPanel.PURPLE);
		panel.add(label, c);
		c.gridwidth = 1;
		c.gridy++;
		panel.add(new JLabel("Interval Length (len)"), c);
		c.gridy++;
		c.gridwidth = 2;
		label = new JLabel("Waypoint Defaults");
		label.setForeground(PathPanel.PURPLE);
		panel.add(label, c);
		c.gridwidth = 1;
		c.gridy++;
		panel.add(new JLabel("Tangent Angle (\u00B0)"), c);
		c.gridy++;
		panel.add(new JLabel("Tangent Mag (len)"), c);
		c.gridy++;
		panel.add(new JLabel("Curvature Angle (\u00B0)"), c);
		c.gridy++;
		panel.add(new JLabel("Curvature Mag (len)"), c);
		c.gridy++;
		c.gridwidth = 2;
		label = new JLabel("Display Settings");
		label.setForeground(PathPanel.PURPLE);
		panel.add(label, c);
		c.gridwidth = 1;
		c.gridy++;
		panel.add(new JLabel("Display Segments (ul)"), c);
		c.gridy++;
		panel.add(new JLabel("Tangent Scale (ul)"), c);
		c.gridy++;
		panel.add(new JLabel("Curvature Scale (ul)"), c);
		c.gridy++;
		panel.add(new JLabel("Widget Size (px)"), c);

		// Controls
		c.gridx = 1;
		c.gridy = 1;
		panel.add(settingsTimeStep, c);
		c.gridy++;
		panel.add(settingsSegmentLength, c);
		c.gridy++;
		panel.add(settingsVelocityInitial, c);
		c.gridy++;
		panel.add(settingsVelocityFinal, c);
		c.gridy++;
		panel.add(settingsPlanSmoothing, c);
		c.gridy++;
		panel.add(settingsMaxIterations, c);
		c.gridy++;
		panel.add(settingsAdjustScale, c);
		c.gridy++;
		panel.add(settingsAdjustOffset, c);
		c.gridy++;
		panel.add(settingsFilterLength, c);
		c.gridy+=2;
		panel.add(settingsWheelbase, c);
		c.gridy++;
		panel.add(settingsWidth, c);
		c.gridy++;
		panel.add(settingsLength, c);
		c.gridy++;
		panel.add(settingsLinVelMax, c);
		c.gridy++;
		panel.add(settingsLinVelTau, c);
		c.gridy++;
		panel.add(settingsAngVelMax, c);
		c.gridy++;
		panel.add(settingsAngVelTau, c);
		c.gridy+=2;
		panel.add(settingsImageScale, c);
		c.gridy++;
		panel.add(settingsImageBrightness, c);
		c.gridy+=2;
		panel.add(settingsZoomScaleFactor, c);
		c.gridy+=2;
		panel.add(settingsIntervalLength, c);
		c.gridy+=2;
		panel.add(settingsTangentAngle, c);
		c.gridy++;
		panel.add(settingsTangentMagnitude, c);
		c.gridy++;
		panel.add(settingsCurvatureAngle, c);
		c.gridy++;
		panel.add(settingsCurvatureMagnitude, c);
		c.gridy+=2;
		panel.add(settingsDisplaySegments, c);
		c.gridy++;
		panel.add(settingsTangentScale, c);
		c.gridy++;
		panel.add(settingsCurvatureScale, c);
		c.gridy++;
		panel.add(settingsWidgetSize, c);

		// Spacer
		c.gridy++;
		c.weighty = 1.0;
		c.weightx = 1.0;
		JPanel spacer = new JPanel();
		panel.add(spacer, c);

		// Create listener
		ChangeListener listener = new ChangeListener() {
			@Override
			public void stateChanged(ChangeEvent e) {
				Object source = e.getSource();
				if(e.getSource() == settingsTimeStep) {
					pathPanel.timeStep = ((Double)settingsTimeStep.getValue()).doubleValue();
					linearGraph.dt = pathPanel.timeStep;
					angularGraph.dt = pathPanel.timeStep;
					sideGraph.dt = pathPanel.timeStep;
				}
				if(e.getSource() == settingsSegmentLength) {
					pathPanel.segmentLength = ((Double)settingsSegmentLength.getValue()).doubleValue();
				}
				if(e.getSource() == settingsVelocityInitial) {
					pathPanel.v_initial = ((Double)settingsVelocityInitial.getValue()).doubleValue();
				}
				if(e.getSource() == settingsVelocityFinal) {
					pathPanel.v_final = ((Double)settingsVelocityFinal.getValue()).doubleValue();
				}
				if(e.getSource() == settingsPlanSmoothing) {
					pathPanel.smoothing = ((Double)settingsPlanSmoothing.getValue()).doubleValue();
				}
				if(e.getSource() == settingsMaxIterations) {
					pathPanel.max_iterations = ((Integer)settingsMaxIterations.getValue()).intValue();
				}
				if(e.getSource() == settingsAdjustScale) {
					pathPanel.adjust_scale = ((Double)settingsAdjustScale.getValue()).doubleValue();
				}
				if(e.getSource() == settingsAdjustOffset) {
					pathPanel.adjust_offset = ((Double)settingsAdjustOffset.getValue()).doubleValue();
				}
				if(e.getSource() == settingsFilterLength) {
					pathPanel.filter_length = ((Integer)settingsFilterLength.getValue()).intValue();
				}
				if(e.getSource() == settingsWheelbase) {
					pathPanel.r_wb = ((Double)settingsWheelbase.getValue()).doubleValue();
				}
				if(e.getSource() == settingsWidth) {
					pathPanel.r_ow = ((Double)settingsWidth.getValue()).doubleValue();
				}
				if(e.getSource() == settingsLength) {
					pathPanel.r_ol = ((Double)settingsLength.getValue()).doubleValue();
				}
				if(e.getSource() == settingsLinVelMax) {
					pathPanel.v_max = ((Double)settingsLinVelMax.getValue()).doubleValue();
					linearGraph.maximums[1] = pathPanel.v_max;
					linearGraph.maximums[2] = pathPanel.v_max/pathPanel.v_tau;
					updateLabel("range_lin_vel", pathPanel.v_max);
					updateLabel("range_lin_acc", pathPanel.v_max/pathPanel.v_tau);
				}
				if(e.getSource() == settingsLinVelTau) {
					pathPanel.v_tau = ((Double)settingsLinVelTau.getValue()).doubleValue();
					linearGraph.maximums[2] = pathPanel.v_max/pathPanel.v_tau;
					updateLabel("range_lin_acc", pathPanel.v_max/pathPanel.v_tau);
				}
				if(e.getSource() == settingsAngVelMax) {
					pathPanel.w_max = ((Double)settingsAngVelMax.getValue()).doubleValue()*Math.PI/180;
					angularGraph.maximums[1] = 180/Math.PI*pathPanel.w_max;
					angularGraph.maximums[2] = 180/Math.PI*pathPanel.w_max/pathPanel.w_tau;
					updateLabel("range_ang_vel", 180/Math.PI*pathPanel.w_max);
					updateLabel("range_ang_acc", 180/Math.PI*pathPanel.w_max/pathPanel.w_tau);
				}
				if(e.getSource() == settingsAngVelTau) {
					pathPanel.w_tau = ((Double)settingsAngVelTau.getValue()).doubleValue();
					angularGraph.maximums[2] = 180/Math.PI*pathPanel.w_max/pathPanel.w_tau;
					updateLabel("range_ang_acc", 180/Math.PI*pathPanel.w_max/pathPanel.w_tau);
				}
				if(e.getSource() == settingsImageScale) {
					pathPanel.imageScale = ((Double)settingsImageScale.getValue()).doubleValue();
				}
				if(e.getSource() == settingsImageBrightness) {
					pathPanel.imageBrightness = ((Double)settingsImageBrightness.getValue()).doubleValue();
					processImage(false);
				}
				if(e.getSource() == settingsZoomScaleFactor) {
					pathPanel.scaleFactor = ((Double)settingsZoomScaleFactor.getValue()).doubleValue();
				}
				if(e.getSource() == settingsIntervalLength) {
					pathPanel.default_length = ((Double)settingsIntervalLength.getValue()).doubleValue();
				}
				if(e.getSource() == settingsTangentAngle) {
					pathPanel.default_v_t = ((Double)settingsTangentAngle.getValue()).doubleValue() * Math.PI/180;
				}
				if(e.getSource() == settingsTangentMagnitude) {
					pathPanel.default_v_m = ((Double)settingsTangentMagnitude.getValue()).doubleValue();
				}
				if(e.getSource() == settingsCurvatureAngle) {
					pathPanel.default_a_t = ((Double)settingsCurvatureAngle.getValue()).doubleValue() * Math.PI/180;
				}
				if(e.getSource() == settingsCurvatureMagnitude) {
					pathPanel.default_a_m = ((Double)settingsCurvatureMagnitude.getValue()).doubleValue();
				}
				if(e.getSource() == settingsDisplaySegments) {
					pathPanel.displaySegments = ((Integer)settingsDisplaySegments.getValue()).intValue();
				}
				if(e.getSource() == settingsTangentScale) {
					pathPanel.v_scale = ((Double)settingsTangentScale.getValue()).doubleValue();
				}
				if(e.getSource() == settingsCurvatureScale) {
					pathPanel.a_scale = ((Double)settingsCurvatureScale.getValue()).doubleValue();
				}
				if(e.getSource() == settingsWidgetSize) {
					pathPanel.widgetSize = ((Double)settingsWidgetSize.getValue()).doubleValue();
				}
				pathPanel.recalculateSplines();
				pathPanel.recalculateProfile();
				updateInspect();
				pathPanel.repaint();
				linearGraph.repaint();
				angularGraph.repaint();
				sideGraph.repaint();
			}
		};

		// Add listener
		settingsTimeStep.addChangeListener(listener);
		settingsSegmentLength.addChangeListener(listener);
		settingsVelocityInitial.addChangeListener(listener);
		settingsVelocityFinal.addChangeListener(listener);
		settingsPlanSmoothing.addChangeListener(listener);
		settingsMaxIterations.addChangeListener(listener);
		settingsAdjustScale.addChangeListener(listener);
		settingsAdjustOffset.addChangeListener(listener);
		settingsFilterLength.addChangeListener(listener);
		settingsWheelbase.addChangeListener(listener);
		settingsWheelbase.addChangeListener(listener);
		settingsWheelbase.addChangeListener(listener);
		settingsWidth.addChangeListener(listener);
		settingsLength.addChangeListener(listener);
		settingsLinVelMax.addChangeListener(listener);
		settingsLinVelTau.addChangeListener(listener);
		settingsAngVelMax.addChangeListener(listener);
		settingsAngVelTau.addChangeListener(listener);
		settingsImageScale.addChangeListener(listener);
		settingsImageBrightness.addChangeListener(listener);
		settingsZoomScaleFactor.addChangeListener(listener);
		settingsIntervalLength.addChangeListener(listener);
		settingsTangentAngle.addChangeListener(listener);
		settingsTangentMagnitude.addChangeListener(listener);
		settingsCurvatureAngle.addChangeListener(listener);
		settingsCurvatureMagnitude.addChangeListener(listener);
		settingsDisplaySegments.addChangeListener(listener);
		settingsTangentScale.addChangeListener(listener);
		settingsCurvatureScale.addChangeListener(listener);
		settingsWidgetSize.addChangeListener(listener);

		JScrollPane scroll = new JScrollPane(panel);
		return scroll;
	}

	/**
	 * Panel for graphs
	 */
	private JPanel initGraphsPanel() {
		linearGraph = new GraphPanel();
		linearGraph.colors = new Color[]{Color.BLUE, Color.RED, Color.GREEN.darker()};
		linearGraph.maximums = new double[]{0, pathPanel.v_max, pathPanel.v_max/pathPanel.v_tau};
		linearGraph.dt = pathPanel.timeStep;
		angularGraph = new GraphPanel();
		angularGraph.colors = new Color[]{Color.BLUE, Color.RED, Color.GREEN.darker()};
		angularGraph.maximums = new double[]{0, 180/Math.PI*pathPanel.w_max, 180/Math.PI*pathPanel.w_max/pathPanel.w_tau};
		angularGraph.dt = pathPanel.timeStep;
		sideGraph = new GraphPanel();
		sideGraph.colors = new Color[]{Color.BLUE, Color.RED};
		sideGraph.maximums = new double[]{1.0, 1.0};
		sideGraph.dt = pathPanel.timeStep;
		GridLayout layout = new GridLayout(0, 1);
		layout.setVgap(2);
		JPanel panel = new JPanel(layout);
		panel.setBackground(Color.BLACK);
		panel.add(linearGraph);
		panel.add(angularGraph);
		panel.add(sideGraph);
		return panel;
	}

	/**
	 * Panel for graph descriptions
	 */
	private JPanel initGraphsDescriptionPanel() {
		// Create outputs
		rangeTime = new JLabel();
		rangeLinPos = new JLabel();
		rangeLinVel = new JLabel();
		rangeLinAcc = new JLabel();
		rangeAngPos = new JLabel();
		rangeAngVel = new JLabel();
		rangeAngAcc = new JLabel();
		rangeLeftVol = new JLabel();
		rangeRightVol = new JLabel();
		minLinPos = new JLabel();
		minLinVel = new JLabel();
		minLinAcc = new JLabel();
		minAngPos = new JLabel();
		minAngVel = new JLabel();
		minAngAcc = new JLabel();
		minLeftVol = new JLabel();
		minRightVol = new JLabel();
		maxLinPos = new JLabel();
		maxLinVel = new JLabel();
		maxLinAcc = new JLabel();
		maxAngPos = new JLabel();
		maxAngVel = new JLabel();
		maxAngAcc = new JLabel();
		maxLeftVol = new JLabel();
		maxRightVol = new JLabel();

		// Set initial values
		updateLabel("range_time", Double.NaN);
		updateLabel("range_lin_pos", Double.NaN);
		updateLabel("range_lin_vel", pathPanel.v_max);
		updateLabel("range_lin_acc", pathPanel.v_max/pathPanel.v_tau);
		updateLabel("range_ang_pos", Double.NaN);
		updateLabel("range_ang_vel", 180/Math.PI*pathPanel.w_max);
		updateLabel("range_ang_acc", 180/Math.PI*pathPanel.w_max/pathPanel.w_tau);
		updateLabel("range_left_vol", 1.0);
		updateLabel("range_right_vol", 1.0);
		updateLabel("min_lin_pos", Double.NaN);
		updateLabel("min_lin_vel", Double.NaN);
		updateLabel("min_lin_acc", Double.NaN);
		updateLabel("min_ang_pos", Double.NaN);
		updateLabel("min_ang_vel", Double.NaN);
		updateLabel("min_ang_acc", Double.NaN);
		updateLabel("min_left_vol", Double.NaN);
		updateLabel("min_right_vol", Double.NaN);
		updateLabel("max_lin_pos", Double.NaN);
		updateLabel("max_lin_vel", Double.NaN);
		updateLabel("max_lin_acc", Double.NaN);
		updateLabel("max_ang_pos", Double.NaN);
		updateLabel("max_ang_vel", Double.NaN);
		updateLabel("max_ang_acc", Double.NaN);
		updateLabel("max_left_vol", Double.NaN);
		updateLabel("max_right_vol", Double.NaN);

		// Layout
		JPanel panel = new JPanel(new GridBagLayout());
		GridBagConstraints c = new GridBagConstraints();
		c.anchor = GridBagConstraints.FIRST_LINE_START;
		c.fill = GridBagConstraints.BOTH;

		// Labels
		c.gridx = 0;
		c.gridy = 1;
		c.gridwidth = 6;
		JLabel label = new JLabel("Time Scale: All Graphs");
		label.setForeground(PathPanel.PURPLE);
		panel.add(label, c);
		c.gridy++;
		panel.add(new JLabel("Dark marks are at 1s intervals"), c);
		c.gridwidth = 1;
		c.gridy++;
		panel.add(new JLabel("Time:"), c);
		c.gridy++;
		c.gridwidth = 6;
		label = new JLabel("Top Graph: Linear");
		label.setForeground(PathPanel.PURPLE);
		panel.add(label, c);
		c.gridwidth = 1;
		c.gridy++;
		label = new JLabel("Position:");
		label.setForeground(Color.BLUE);
		panel.add(label, c);
		c.gridy++;
		label = new JLabel("Velocity:");
		label.setForeground(Color.RED);
		panel.add(label, c);
		c.gridy++;
		label = new JLabel("Acceleration:");
		label.setForeground(Color.GREEN.darker());
		panel.add(label, c);
		c.gridy++;
		c.gridwidth = 6;
		label = new JLabel("Center Graph: Rotational");
		label.setForeground(PathPanel.PURPLE);
		panel.add(label, c);
		c.gridwidth = 1;
		c.gridy++;
		label = new JLabel("Position:");
		label.setForeground(Color.BLUE);
		panel.add(label, c);
		c.gridy++;
		label = new JLabel("Velocity:");
		label.setForeground(Color.RED);
		panel.add(label, c);
		c.gridy++;
		label = new JLabel("Acceleration:");
		label.setForeground(Color.GREEN.darker());
		panel.add(label, c);
		c.gridy++;
		c.gridwidth = 6;
		label = new JLabel("Bottom Graph: Voltage");
		label.setForeground(PathPanel.PURPLE);
		panel.add(label, c);
		c.gridy++;
		panel.add(new JLabel("Graphed as Voltage/Max Voltage"), c);
		c.gridwidth = 1;
		c.gridy++;
		label = new JLabel("Left Side:");
		label.setForeground(Color.BLUE);
		panel.add(label, c);
		c.gridy++;
		label = new JLabel("Right Side:");
		label.setForeground(Color.RED);
		panel.add(label, c);

		// Range display
		c.gridy = 0;
		c.gridx++;
		panel.add(new JPanel(), c);
		c.gridx++;
		label = new JLabel("Graph Range");
		label.setForeground(Color.MAGENTA);
		panel.add(label, c);
		c.gridy+=3;
		panel.add(rangeTime, c);
		c.gridy+=2;
		panel.add(rangeLinPos, c);
		c.gridy++;
		panel.add(rangeLinVel, c);
		c.gridy++;
		panel.add(rangeLinAcc, c);
		c.gridy+=2;
		panel.add(rangeAngPos, c);
		c.gridy++;
		panel.add(rangeAngVel, c);
		c.gridy++;
		panel.add(rangeAngAcc, c);
		c.gridy+=3;
		panel.add(rangeLeftVol, c);
		c.gridy++;
		panel.add(rangeRightVol, c);

		// Min display
		c.gridy = 0;
		c.gridx++;
		panel.add(new JPanel(), c);
		c.gridx++;
		label = new JLabel("Minimum");
		label.setForeground(Color.MAGENTA);
		panel.add(label, c);
		c.gridy+=5;
		panel.add(minLinPos, c);
		c.gridy++;
		panel.add(minLinVel, c);
		c.gridy++;
		panel.add(minLinAcc, c);
		c.gridy+=2;
		panel.add(minAngPos, c);
		c.gridy++;
		panel.add(minAngVel, c);
		c.gridy++;
		panel.add(minAngAcc, c);
		c.gridy+=3;
		panel.add(minLeftVol, c);
		c.gridy++;
		panel.add(minRightVol, c);

		// Max display
		c.gridy = 0;
		c.gridx++;
		panel.add(new JPanel(), c);
		c.gridx++;
		label = new JLabel("Maximum");
		label.setForeground(Color.MAGENTA);
		panel.add(label, c);
		c.gridy+=5;
		panel.add(maxLinPos, c);
		c.gridy++;
		panel.add(maxLinVel, c);
		c.gridy++;
		panel.add(maxLinAcc, c);
		c.gridy+=2;
		panel.add(maxAngPos, c);
		c.gridy++;
		panel.add(maxAngVel, c);
		c.gridy++;
		panel.add(maxAngAcc, c);
		c.gridy+=3;
		panel.add(maxLeftVol, c);
		c.gridy++;
		panel.add(maxRightVol, c);

		// Spacer
		c.gridy++;
		c.weighty = 1.0;
		c.weightx = 1.0;
		JPanel spacer = new JPanel();
		panel.add(spacer, c);
		return panel;
	}

	/**
	 * Regenerates combo box for fine editing
	 */
	public void regenerateEditSelector() {
		editSelector.removeAllItems();
		editSelector.addItem("Inspect");
		for(int i = 0; i < pathPanel.waypoints.size(); i++) {
			editSelector.addItem("Waypoint " + (i+1));
		}
		for(int i = 0; i < pathPanel.limits.size(); i++) {
			editSelector.addItem("Limit " + (i+1));
		}
	}

	/**
	 * Updates the fine editing panel with current data
	 */
	public void updateEditPanel() {
		if(pathPanel.currIndex == 0) {
			cardLayout.show(cardPanel, "inspect");
			if(!suppress) {
				if(editState != EditState.INSPECT) {
					inspect.doClick();
				}
			}
		}
		else if(pathPanel.currIndex <= pathPanel.waypoints.size()) {
			cardLayout.show(cardPanel, "waypoint");
			if(editState != EditState.EDITWAYPOINT) {
				editWaypoint.doClick();
			}

			// Update fields
			Waypoint waypoint = pathPanel.waypoints.get(pathPanel.currIndex-1);
			suppress = true;
			editX.setValue(waypoint.x);
			editY.setValue(waypoint.y);
			editVT.setValue(waypoint.v_t*180/Math.PI);
			editVM.setValue(waypoint.v_m);
			editAT.setValue(waypoint.a_t*180/Math.PI);
			editAM.setValue(waypoint.a_m);
			suppress = false;
		}
		else {
			cardLayout.show(cardPanel, "limit");
			if(editState != EditState.EDITLIMIT) {
				editLimit.doClick();
			}

			// Update fields
			Limit limit = pathPanel.limits.get(pathPanel.currIndex-1-pathPanel.waypoints.size());
			boolean notFirst = !(limit == pathPanel.firstLimit);
			suppress = true;
			editT1.setEnabled(notFirst);
			editT2.setEnabled(notFirst);
			editT1.setValue(limit.t1);
			editT2.setValue(limit.t2);
			switch(limit.type) {
				case VOLTAGE:
					editType.setSelectedIndex(0);
					break;
				case VELOCITY:
					editType.setSelectedIndex(1);
					break;
				case ACCELERATION:
					editType.setSelectedIndex(2);
					break;
			}
			editLim.setValue(limit.limit);
			suppress = false;
		}
	}

	/**
	 * Updates a numerical status label
	 */
	public void updateLabel(String name, double value) {
		switch(name) {
			case "range_time":
				rangeTime.setText(Double.isNaN(value) ? "--" : String.format("0 s - %.2f s", value));
				break;
			case "range_lin_pos":
				rangeLinPos.setText(Double.isNaN(value) ? "--" : String.format("\u00B1%.4g len", value));
				break;
			case "range_lin_vel":
				rangeLinVel.setText(Double.isNaN(value) ? "--" : String.format("\u00B1%.4g len/s", value));
				break;
			case "range_lin_acc":
				rangeLinAcc.setText(Double.isNaN(value) ? "--" : String.format("\u00B1%.4g len/s\u00B2", value));
				break;
			case "range_ang_pos":
				rangeAngPos.setText(Double.isNaN(value) ? "--" : String.format("\u00B1%.4g \u00B0", value));
				break;
			case "range_ang_vel":
				rangeAngVel.setText(Double.isNaN(value) ? "--" : String.format("\u00B1%.4g \u00B0/s", value));
				break;
			case "range_ang_acc":
				rangeAngAcc.setText(Double.isNaN(value) ? "--" : String.format("\u00B1%.4g \u00B0/s\u00B2", value));
				break;
			case "range_left_vol":
				rangeLeftVol.setText(Double.isNaN(value) ? "--" : String.format("\u00B1%.4g V/V", value));
				break;
			case "range_right_vol":
				rangeRightVol.setText(Double.isNaN(value) ? "--" : String.format("\u00B1%.4g V/V", value));
				break;
			case "min_lin_pos":
				minLinPos.setText(Double.isNaN(value) ? "--" : String.format("%.4g len", value));
				break;
			case "min_lin_vel":
				minLinVel.setText(Double.isNaN(value) ? "--" : String.format("%.4g len/s", value));
				break;
			case "min_lin_acc":
				minLinAcc.setText(Double.isNaN(value) ? "--" : String.format("%.4g len/s\u00B2", value));
				break;
			case "min_ang_pos":
				minAngPos.setText(Double.isNaN(value) ? "--" : String.format("%.4g\u00B0", value));
				break;
			case "min_ang_vel":
				minAngVel.setText(Double.isNaN(value) ? "--" : String.format("%.4g\u00B0/s", value));
				break;
			case "min_ang_acc":
				minAngAcc.setText(Double.isNaN(value) ? "--" : String.format("%.4g\u00B0/s\u00B2", value));
				break;
			case "min_left_vol":
				minLeftVol.setText(Double.isNaN(value) ? "--" : String.format("%.4g V/V", value));
				break;
			case "min_right_vol":
				minRightVol.setText(Double.isNaN(value) ? "--" : String.format("%.4g V/V", value));
				break;
			case "max_lin_pos":
				maxLinPos.setText(Double.isNaN(value) ? "--" : String.format("%.4g len", value));
				break;
			case "max_lin_vel":
				maxLinVel.setText(Double.isNaN(value) ? "--" : String.format("%.4g len/s", value));
				break;
			case "max_lin_acc":
				maxLinAcc.setText(Double.isNaN(value) ? "--" : String.format("%.4g len/s\u00B2", value));
				break;
			case "max_ang_pos":
				maxAngPos.setText(Double.isNaN(value) ? "--" : String.format("%.4g\u00B0", value));
				break;
			case "max_ang_vel":
				maxAngVel.setText(Double.isNaN(value) ? "--" : String.format("%.4g\u00B0/s", value));
				break;
			case "max_ang_acc":
				maxAngAcc.setText(Double.isNaN(value) ? "--" : String.format("%.4g\u00B0/s\u00B2", value));
				break;
			case "max_left_vol":
				maxLeftVol.setText(Double.isNaN(value) ? "--" : String.format("%.4g V/V", value));
				break;
			case "max_right_vol":
				maxRightVol.setText(Double.isNaN(value) ? "--" : String.format("%.4g V/V", value));
				break;
			default:
				System.out.println("Unsupported Label");
				break;
		};
	}

	/**
	 * Fixes the dividers
	 */
	public void fixDividers() {
		int location = hsplitLarge.getDividerLocation();
		int maxLocation = hsplitLarge.getMaximumDividerLocation();
		if(location < hsplitLarge.getWidth()-hsplitLarge.getDividerSize()*2 && location > maxLocation) {
			hsplitLarge.setDividerLocation(maxLocation);
		}
	}

	/**
	 * Sets the ready indicator
	 */
	public void setReady(boolean ready) {
		status1.setText(ready ? " <Ready> " : " <Busy> ");
	}

	/**
	 * Updates the inspect panel
	 */
	public void updateInspect() {
		// Mouse moved over area
		if(editState == EditState.INSPECT && !Double.isNaN(pathPanel.inspectX2)) {
			freeX2.setText(String.format("%.4g", pathPanel.inspectX2));
			freeY2.setText(String.format("%.4g", pathPanel.inspectY2));
			double pathX2Value = Double.NaN;
			double pathY2Value = Double.NaN;
			double pathTime2Value = Double.NaN;
			double linPos2Value = Double.NaN;
			double linVel2Value = Double.NaN;
			double linAcc2Value = Double.NaN;
			double angPos2Value = Double.NaN;
			double angVel2Value = Double.NaN;
			double angAcc2Value = Double.NaN;
			double leftVol2Value = Double.NaN;
			double rightVol2Value = Double.NaN;

			// Path exists
			if(pathPanel.times.length > 0) {
				// Find location closest to spline
				double min_distance = Double.MAX_VALUE;
				int min_index = 0;
				for(int i = 0; i < pathPanel.times.length; i++) {
					double distance = Math.hypot(pathPanel.inspectX2 - pathPanel.timePoints[i][0], pathPanel.inspectY2 - pathPanel.timePoints[i][1]);
					if(distance < min_distance) {
						min_index = i;
						min_distance = distance;
					}
				}
				pathPanel.inspectU2 = pathPanel.times[min_index];
				pathX2Value = pathPanel.timePoints[min_index][0];
				pathY2Value = pathPanel.timePoints[min_index][1];
				pathX2.setText(String.format("%.4g", pathX2Value));
				pathY2.setText(String.format("%.4g", pathY2Value));

				// Find time paramaterized point closest to location
				double s = 0;
				for(int i = 0; i < min_index; i++) {
					s += pathPanel.splineData[1][i];
				}
				min_index = 0;
				double min_diff = Double.MAX_VALUE;
				for(int i = 0; i < pathPanel.timeData[0].length; i++) {
					double diff = Math.abs(s - pathPanel.timeData[0][i]);
					if(diff < min_diff) {
						min_diff = diff;
						min_index = i;
					}
				}
				if(linearGraph != null) {
					linearGraph.cursor2 = min_index;
					angularGraph.cursor2 = min_index;
					sideGraph.cursor2 = min_index;
				}
				pathTime2Value = min_index*pathPanel.timeStep;
				linPos2Value = linearGraph.graphs[0][min_index];
				linVel2Value = linearGraph.graphs[1][min_index];
				linAcc2Value = linearGraph.graphs[2][min_index];
				angPos2Value = angularGraph.graphs[0][min_index];
				angVel2Value = angularGraph.graphs[1][min_index];
				angAcc2Value = angularGraph.graphs[2][min_index];
				leftVol2Value = sideGraph.graphs[0][min_index];
				rightVol2Value = sideGraph.graphs[1][min_index];
				pathTime2.setText(String.format("%.2f", pathTime2Value));
				linPos2.setText(String.format("%.4g", linPos2Value));
				linVel2.setText(String.format("%.4g", linVel2Value));
				linAcc2.setText(String.format("%.4g", linAcc2Value));
				angPos2.setText(String.format("%.4g", angPos2Value));
				angVel2.setText(String.format("%.4g", angVel2Value));
				angAcc2.setText(String.format("%.4g", angAcc2Value));
				leftVol2.setText(String.format("%.4g", leftVol2Value));
				rightVol2.setText(String.format("%.4g", rightVol2Value));
			}

			// Path does not exist
			else {
				pathPanel.inspectU2 = Double.NaN;
				if(linearGraph != null) {
					linearGraph.cursor2 = -1;
					angularGraph.cursor2 = -1;
					sideGraph.cursor2 = -1;
				}
				pathX2.setText("--");
				pathY2.setText("--");
				pathTime2.setText("--");
				linPos2.setText("--");
				linVel2.setText("--");
				linAcc2.setText("--");
				angPos2.setText("--");
				angVel2.setText("--");
				angAcc2.setText("--");
				leftVol2.setText("--");
				rightVol2.setText("--");
			}

			// Mouse pressed
			if(!Double.isNaN(pathPanel.inspectX1)) {
				freeX1.setText(String.format("%.4g", pathPanel.inspectX1));
				freeY1.setText(String.format("%.4g", pathPanel.inspectY1));
				freeX3.setText(String.format("%.4g", pathPanel.inspectX2 - pathPanel.inspectX1));
				freeY3.setText(String.format("%.4g", pathPanel.inspectY2 - pathPanel.inspectY1));
				freeHypot3.setText(String.format("%.4g", Math.hypot(pathPanel.inspectX2 - pathPanel.inspectX1, pathPanel.inspectY2 - pathPanel.inspectY1)));

				// Path exists
				if(pathPanel.times.length > 0) {
					// Find location closest to spline
					double min_distance = Double.MAX_VALUE;
					int min_index = 0;
					for(int i = 0; i < pathPanel.times.length; i++) {
						double distance = Math.hypot(pathPanel.inspectX1 - pathPanel.timePoints[i][0], pathPanel.inspectY1 - pathPanel.timePoints[i][1]);
						if(distance < min_distance) {
							min_index = i;
							min_distance = distance;
						}
					}
					pathPanel.inspectU1 = pathPanel.times[min_index];
					double pathX1Value = pathPanel.timePoints[min_index][0];
					double pathY1Value = pathPanel.timePoints[min_index][1];
					pathX1.setText(String.format("%.4g", pathX1Value));
					pathY1.setText(String.format("%.4g", pathY1Value));
					pathX3.setText(String.format("%.4g", pathX2Value - pathX1Value));
					pathY3.setText(String.format("%.4g", pathY2Value - pathY1Value));
					pathHypot3.setText(String.format("%.4g", Math.hypot(pathX2Value - pathX1Value, pathY2Value - pathY1Value)));

					// Find time paramaterized point closest to location
					double s = 0;
					for(int i = 0; i < min_index; i++) {
						s += pathPanel.splineData[1][i];
					}
					min_index = 0;
					double min_diff = Double.MAX_VALUE;
					for(int i = 0; i < pathPanel.timeData[0].length; i++) {
						double diff = Math.abs(s - pathPanel.timeData[0][i]);
						if(diff < min_diff) {
							min_diff = diff;
							min_index = i;
						}
					}
					if(linearGraph != null) {
						linearGraph.cursor1 = min_index;
						angularGraph.cursor1 = min_index;
						sideGraph.cursor1 = min_index;
					}
					double pathTime1Value = min_index*pathPanel.timeStep;
					double linPos1Value = linearGraph.graphs[0][min_index];
					double linVel1Value = linearGraph.graphs[1][min_index];
					double linAcc1Value = linearGraph.graphs[2][min_index];
					double angPos1Value = angularGraph.graphs[0][min_index];
					double angVel1Value = angularGraph.graphs[1][min_index];
					double angAcc1Value = angularGraph.graphs[2][min_index];
					double leftVol1Value = sideGraph.graphs[0][min_index];
					double rightVol1Value = sideGraph.graphs[1][min_index];
					pathTime1.setText(String.format("%.2f", pathTime1Value));
					linPos1.setText(String.format("%.4g", linPos1Value));
					linVel1.setText(String.format("%.4g", linVel1Value));
					linAcc1.setText(String.format("%.4g", linAcc1Value));
					angPos1.setText(String.format("%.4g", angPos1Value));
					angVel1.setText(String.format("%.4g", angVel1Value));
					angAcc1.setText(String.format("%.4g", angAcc1Value));
					leftVol1.setText(String.format("%.4g", leftVol1Value));
					rightVol1.setText(String.format("%.4g", rightVol1Value));
					pathTime3.setText(String.format("%.2f", pathTime2Value - pathTime1Value));
					linPos3.setText(String.format("%.4g", linPos2Value - linPos1Value));
					linVel3.setText(String.format("%.4g", linVel2Value - linVel1Value));
					linAcc3.setText(String.format("%.4g", linAcc2Value - linAcc1Value));
					angPos3.setText(String.format("%.4g", angPos2Value - angPos1Value));
					angVel3.setText(String.format("%.4g", angVel2Value - angVel1Value));
					angAcc3.setText(String.format("%.4g", angAcc2Value - angAcc1Value));
					leftVol3.setText(String.format("%.4g", leftVol2Value - leftVol1Value));
					rightVol3.setText(String.format("%.4g", rightVol2Value - rightVol1Value));
				}

				// Path does not exist
				else {
					pathPanel.inspectU1 = Double.NaN;
					if(linearGraph != null) {
						linearGraph.cursor1 = -1;
						angularGraph.cursor1 = -1;
						sideGraph.cursor1 = -1;
					}
					pathX1.setText("--");
					pathY1.setText("--");
					pathTime1.setText("--");
					linPos1.setText("--");
					linVel1.setText("--");
					linAcc1.setText("--");
					angPos1.setText("--");
					angVel1.setText("--");
					angAcc1.setText("--");
					leftVol1.setText("--");
					rightVol1.setText("--");
					freeHypot3.setText("--");
					pathX3.setText("--");
					pathY3.setText("--");
					pathHypot3.setText("--");
					pathTime3.setText("--");
					linPos3.setText("--");
					linVel3.setText("--");
					linAcc3.setText("--");
					angPos3.setText("--");
					angVel3.setText("--");
					angAcc3.setText("--");
					leftVol3.setText("--");
					rightVol3.setText("--");
				}
			}

			// Mouse not pressed
			else {
				pathPanel.inspectU1 = Double.NaN;
				if(linearGraph != null) {
					linearGraph.cursor1 = -1;
					angularGraph.cursor1 = -1;
					sideGraph.cursor1 = -1;
				}
				freeX1.setText("--");
				freeY1.setText("--");
				pathX1.setText("--");
				pathY1.setText("--");
				pathTime1.setText("--");
				linPos1.setText("--");
				linVel1.setText("--");
				linAcc1.setText("--");
				angPos1.setText("--");
				angVel1.setText("--");
				angAcc1.setText("--");
				leftVol1.setText("--");
				rightVol1.setText("--");
				freeX3.setText("--");
				freeY3.setText("--");
				freeHypot3.setText("--");
				pathX3.setText("--");
				pathY3.setText("--");
				pathHypot3.setText("--");
				pathTime3.setText("--");
				linPos3.setText("--");
				linVel3.setText("--");
				linAcc3.setText("--");
				angPos3.setText("--");
				angVel3.setText("--");
				angAcc3.setText("--");
				leftVol3.setText("--");
				rightVol3.setText("--");
			}

			if(hsplitSmall != null) {
				int location = hsplitSmall.getDividerLocation();
				int minLocation = hsplitSmall.getMinimumDividerLocation();
				if(location > hsplitSmall.getDividerSize() && location < minLocation) {
					hsplitSmall.setDividerLocation(minLocation);
				}
			}
		}

		// Mouse not moved over area
		else {
			pathPanel.inspectU2 = Double.NaN;
			pathPanel.inspectU1 = Double.NaN;
			if(linearGraph != null) {
				linearGraph.cursor2 = -1;
				angularGraph.cursor2 = -1;
				sideGraph.cursor2 = -1;
				linearGraph.cursor1 = -1;
				angularGraph.cursor1 = -1;
				sideGraph.cursor1 = -1;
			}
			freeX1.setText("--");
			freeY1.setText("--");
			pathX1.setText("--");
			pathY1.setText("--");
			pathTime1.setText("--");
			linPos1.setText("--");
			linVel1.setText("--");
			linAcc1.setText("--");
			angPos1.setText("--");
			angVel1.setText("--");
			angAcc1.setText("--");
			leftVol1.setText("--");
			rightVol1.setText("--");
			freeX2.setText("--");
			freeY2.setText("--");
			pathX2.setText("--");
			pathY2.setText("--");
			pathTime2.setText("--");
			linPos2.setText("--");
			linVel2.setText("--");
			linAcc2.setText("--");
			angPos2.setText("--");
			angVel2.setText("--");
			angAcc2.setText("--");
			leftVol2.setText("--");
			rightVol2.setText("--");
			freeX3.setText("--");
			freeY3.setText("--");
			freeHypot3.setText("--");
			pathX3.setText("--");
			pathY3.setText("--");
			pathHypot3.setText("--");
			pathTime3.setText("--");
			linPos3.setText("--");
			linVel3.setText("--");
			linAcc3.setText("--");
			angPos3.setText("--");
			angVel3.setText("--");
			angAcc3.setText("--");
			leftVol3.setText("--");
			rightVol3.setText("--");
		}
	}

	/**
	 * Do brightening as little as possible
	 */
	private void processImage(boolean create) {
		if(pathPanel.rawImage != null) {
			RescaleOp rescaleOp = new RescaleOp((float)(1-pathPanel.imageBrightness), (float)(255*pathPanel.imageBrightness), null);
			if(create) {
				pathPanel.brightenedImage = rescaleOp.createCompatibleDestImage(pathPanel.rawImage, null);
				pathPanel.imageWidth = pathPanel.rawImage.getWidth();
				pathPanel.imageHeight = pathPanel.rawImage.getHeight();
			}
			rescaleOp.filter(pathPanel.rawImage, pathPanel.brightenedImage);
		}
	}

	/**
	 * Entry point
	 */
	public static void main(String[] args) {
		System.setProperty("awt.useSystemAAFontSettings", "on");
		SwingUtilities.invokeLater(new Runnable() {
			@Override
			public void run() {
				new MainFrame();
			}
		});
	}
}
