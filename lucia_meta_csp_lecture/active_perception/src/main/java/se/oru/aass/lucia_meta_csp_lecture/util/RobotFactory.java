package se.oru.aass.lucia_meta_csp_lecture.util;

import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.Variable;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;

import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;

public class RobotFactory {
	
	private static final int NUM_VERTS = 6;
	private static final float RADIUS = 0.2f;

	public static Variable createSpatioTemporalSetVariable(String id, Vec2 p, float theta, ConstraintSolver cs) {
		Vec2[] verts = new Vec2[NUM_VERTS];
		for (int i = 0; i < NUM_VERTS; i++) {
			verts[i] = new Vec2((float)(RADIUS*Math.cos(2*Math.PI*i/NUM_VERTS+theta)+p.x), (float)(RADIUS*Math.sin(2*Math.PI*i/NUM_VERTS+theta)+p.y));
		}
		Variable ret = (SpatioTemporalSet)cs.createVariable(id);
		((SpatioTemporalSet)ret).getPolygon().setDomain(verts);
		((SpatioTemporalSet)ret).getPolygon().setMovable(true);
		return ret;
	}
	
	public static Variable createPolygonVariable(String id, Vec2 p, float theta, ConstraintSolver cs) {
		Vec2[] verts = new Vec2[NUM_VERTS];
		for (int i = 0; i < NUM_VERTS; i++) {
			verts[i] = new Vec2((float)(RADIUS*Math.cos(2*Math.PI*i/NUM_VERTS+theta)+p.x), (float)(RADIUS*Math.sin(2*Math.PI*i/NUM_VERTS+theta)+p.y));
		}
		Variable ret = (Polygon)cs.createVariable(id);
		((Polygon)ret).setDomain(verts);
		((Polygon)ret).setMovable(true);
		return ret;
	}
	
	public static Vec2[] getVerticesByCenter(Vec2 p){
		float theta = 0.0f;
		Vec2[] verts = new Vec2[NUM_VERTS];
		for (int i = 0; i < NUM_VERTS; i++) {
			verts[i] = new Vec2((float)(RADIUS*Math.cos(2*Math.PI*i/NUM_VERTS+theta)+p.x), (float)(RADIUS*Math.sin(2*Math.PI*i/NUM_VERTS+theta)+p.y));
		}
		
		return verts;
	}

}
