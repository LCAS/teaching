package se.oru.aass.lucia_meta_csp_lecture;

import java.util.Random;
import java.util.Vector;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.multi.symbols.SymbolicVariableConstraintSolver;
import org.metacsp.spatial.geometry.GeometricConstraint;
import org.metacsp.spatial.geometry.GeometricConstraintSolver;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.UI.PolygonFrame;
import org.metacsp.utility.timelinePlotting.TimelinePublisher;
import org.metacsp.utility.timelinePlotting.TimelineVisualizer;

import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.AssignmentMetaConstraint;
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.LuciaMetaConstraintSolver;
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.ObservabilityMetaConstraint;
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.SimpleMoveBasePlanner;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;
import se.oru.aass.lucia_meta_csp_lecture.util.PanelFactory;
import se.oru.aass.lucia_meta_csp_lecture.util.RobotFactory;

public class SimpleTestPanelsRobots {
		
	public static void main(String[] args) {
		
		//MetaCSPLogging.setLevel(Level.FINE);
		//Symbols represent panels seen by robots
		int numPanels = 4;
		String[] panels = new String[numPanels];
		String[] symbols = new String[numPanels+1];
		for (int i = 0; i < numPanels; i++) {
			panels[i] = "P"+i;
			symbols[i] = "P"+i;
		}
		//Another symbol ("None") represents the fact that a robot sees no panel
		symbols[numPanels] = "None";
		
		LuciaMetaConstraintSolver metaSolver = new LuciaMetaConstraintSolver(0,100000,500,symbols);
		SpatioTemporalSetNetworkSolver spatioTemporalSetSolver = (SpatioTemporalSetNetworkSolver)metaSolver.getConstraintSolvers()[0];
		ActivityNetworkSolver activitySolver = spatioTemporalSetSolver.getActivitySolver();
		GeometricConstraintSolver geometricSolver = spatioTemporalSetSolver.getGeometricSolver();
		SymbolicVariableConstraintSolver setSolver = spatioTemporalSetSolver.getSetSolver();
		
		Variable p1 = PanelFactory.createPolygonVariables(panels[0], new Vec2(7.0f,-9.0f), new Vec2(9.0f,-7.0f), geometricSolver)[0];
		Variable robot = RobotFactory.createSpatioTemporalSetVariable("A Robot", new Vec2(0.0f,0.0f), 0.0f, spatioTemporalSetSolver);
		
		Polygon poly1 = ((SpatioTemporalSet)robot).getPolygon();
		Polygon poly2 = (Polygon)p1;
		
		if (!GeometricConstraintSolver.getRelation(poly1, poly2).equals(GeometricConstraint.Type.INSIDE)) {
			GeometricConstraint inside = new GeometricConstraint(GeometricConstraint.Type.INSIDE);
			inside.setFrom(poly1);
			inside.setTo(poly2);
			System.out.println("Added? " + geometricSolver.addConstraint(inside));
		}
		
		PolygonFrame pf = new PolygonFrame("Nothing", geometricSolver.getConstraintNetwork());
		
		System.out.println(GeometricConstraintSolver.getRelation(poly1, poly2));
		
	}


}
