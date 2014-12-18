package se.oru.aass.lucia_meta_csp_lecture.exercises;


import java.util.Vector;

import se.oru.aass.lucia_meta_csp_lecture.util.PanelFactory;
import se.oru.aass.lucia_meta_csp_lecture.util.RobotFactory;

import org.metacsp.framework.Variable;
import org.metacsp.spatial.geometry.GeometricConstraint;
import org.metacsp.spatial.geometry.GeometricConstraintSolver;
import org.metacsp.spatial.geometry.Polygon;

import org.metacsp.spatial.geometry.Vec2;
import org.metacsp.utility.UI.PolygonFrame;

public class Ex4  {
	
	public static void main(String[] args) {
		
		GeometricConstraintSolver spatialSolver = new GeometricConstraintSolver();
		
		//Creating two polygon representing two FoVs of a panel
		Vec2 p1 = new Vec2(-0.033f, -2.105f);
		Vec2 p2 = new Vec2(-0.311f, -2.463f);
		Variable[] panelVars = PanelFactory.createPolygonVariables("panel1", p1, p2, spatialSolver);
		
		//creating "turtlebot_1" polygon
		Vec2 robot_center = new Vec2(0.0f, 0.0f);
		Variable turtlebot_1 = RobotFactory.createPolygonVariable("turtlebot_1", robot_center, 0.0f, spatialSolver);
		
		//creating "wall" polugon
		Variable wall = spatialSolver.createVariable("wall");
		Vector<Vec2> vecs1 = new Vector<Vec2>();
		vecs1.add(new Vec2(-2.136f, -0.982f));
		vecs1.add(new Vec2(-3.430f, -0.943f));
		vecs1.add(new Vec2(-3.467f, -1.572f));
		vecs1.add(new Vec2(-2.156f, -1.542f));
		((Polygon)wall).setDomain(vecs1.toArray(new Vec2[vecs1.size()]));
		((Polygon)wall).setMovable(false);

		//visualization
		PolygonFrame pf = new PolygonFrame("Polygon Constraint Network", spatialSolver.getConstraintNetwork());

		try { Thread.sleep(2000); }
		catch (InterruptedException e) { e.printStackTrace(); }

		//turtlebot_1 is Disconnected from the obstacle
		//here adding DC constraint between turtlebot_1 and wall
		GeometricConstraint dc = new GeometricConstraint(GeometricConstraint.Type.DC);
		dc.setFrom(turtlebot_1);
		dc.setTo(wall);
		System.out.println("Added? " + spatialSolver.addConstraint(dc));

		try { Thread.sleep(2000); }
		catch (InterruptedException e) { e.printStackTrace(); }

		//TODO: adding a spatial constraint representing turtlebot_1 should be Inside a FoV of the panel
		GeometricConstraint inside = new GeometricConstraint(GeometricConstraint.Type.INSIDE);
		inside.setFrom(turtlebot_1);
		inside.setTo(panelVars[0]);
		System.out.println("Added? " + spatialSolver.addConstraint(inside));
		
	}

}

