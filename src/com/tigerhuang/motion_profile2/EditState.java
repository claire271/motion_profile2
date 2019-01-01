package com.tigerhuang.motion_profile2;

/**
 * Possible states for the path editor
 * 
 * @author Tiger Huang
 *
 */
enum EditState {
	INSPECT, PAN, ZOOM,
	ADDWAYPOINT, EDITWAYPOINT, DELETEWAYPOINT,
	ADDLIMIT, EDITLIMIT, DELETELIMIT
};
