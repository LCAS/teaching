package se.oru.aass.lucia_meta_csp_lecture.util;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Vector;

import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.Variable;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;

public class PanelFactory {

	//private static final float d1 = 0.4f;
	//private static final float d2 = 2f;
	//private static final float teta = 0.5236f; //30 degrees
	private static final float d1 = 0.1f;
	private static final float d2 = 0.7f;
	private static final float teta = 0.3491f; //20 degrees
	//radii of circle
	private static final float r1 = (float)(d1/Math.cos(teta));
	private static final float r2 = (float)(d2/Math.cos(teta));
	private static HashMap<String, Vec2> panelCenters = new HashMap<String, Vec2>();
	

	public static Variable[] createPolygonVariables(String id, Vec2 p1, Vec2 p2, ConstraintSolver cs) {
		return createPolygonVariables(id, p1, p2, cs, false, false);
	}
	
	public static Vec2 getPanelCenterById(String id){
		return panelCenters.get(id);
	}
	
	public static Variable[] createPolygonVariables(String id, Vec2 p1, Vec2 p2, ConstraintSolver cs, boolean skipFirst, boolean skipSecond) {
		Vector<Vec2> rights = getTrapazoid(d1, d2, p1, p2, r1, r2);        
		Vector<Vec2> lefts = getTrapazoid(-d1, -d2, p1, p2, r1, r2);
		Polygon poly1 = null;
		Polygon poly2 = null;
		
		panelCenters.put(id, new Vec2((p1.x + (p2.x - p1.x)),(p1.y + (p2.y - p1.y)))); 
		
		Vector<Variable> ret = new Vector<Variable>();
		if (!skipFirst) {
			poly1 = (Polygon)cs.createVariable(id);
			poly1.setDomain(lefts.toArray(new Vec2[lefts.size()]));
			poly1.setMovable(false);
			poly1.setOrientation((float)(Math.atan2(p2.y-p1.y, p2.x-p1.x)+Math.PI/2));
			ret.add(poly1);
		}
		if (!skipSecond) {
			poly2 = (Polygon)cs.createVariable(id);
			poly2.setDomain(rights.toArray(new Vec2[rights.size()]));
			poly2.setMovable(false);
			poly2.setOrientation((float)(Math.atan2(p2.y-p1.y, p2.x-p1.x)-Math.PI/2));
			ret.add(poly2);
		}
		return ret.toArray(new Variable[ret.size()]);
	}

	private static Vector<Vec2> getTrapazoid(float d1, float d2, Vec2 a1, Vec2 a2, float r1, float r2) {
		Vector<Vec2> ret = new Vector<Vec2>();
		float A = 0;
		float B  = 0;
		float C = 0;
		//First define QR line equation in the form of Ax + By + c = 0
		if(a1.y == a2.y){                       
			A = 0;
			B  = 1;
			C = -a1.y;                      
		}
		else if(a1.x == a2.x){
			A = 1;
			B  = 0;
			C = -a1.x;
		}
		else{
			//slope
			float m = (float)(a1.y - a2.y)/(a1.x - a2.x);         
			//First define QR line equation in the form of Ax + By + c = 0
			A = 1;
			//B = -1/m
			B  = (float)-1/m;
			//C = (-mx1 + y1)/m
			C = (float)(-m*a1.x + a1.y)/m; 
		}

		//The distance from a point (m, n) to the line Ax + By + C = 0 is given by:
		//|Ax+By+c| = d* SQRT(A^2+B^2)

		Vec2 p1 = getVec2sinParalellLine(d1, A, B, C , 2.0f);
		Vec2 p2 = getVec2sinParalellLine(d1, A, B, C , 3.0f);
		ret.addAll(getCircleLineIntersectionVec2(p1, p2, a1, r1));
		ret.addAll(getCircleLineIntersectionVec2(p1, p2, a2, r1));

		Vec2 pp1 = getVec2sinParalellLine(d2, A, B, C , 2.0f);
		Vec2 pp2 = getVec2sinParalellLine(d2, A, B, C , 3.0f);
		ret.addAll(getCircleLineIntersectionVec2(pp1, pp2, a1, r2));
		ret.addAll(getCircleLineIntersectionVec2(pp1, pp2, a2, r2));


		return ret;
	}

	private static Vec2 getVec2sinParalellLine(float d1, float A, float B, float C , float y1){
		if(B == 0.0){
			Vec2 ret = new Vec2(-C + d1, y1);             
			return ret;
		}else if(A == 0.0){
			Vec2 ret = new Vec2(y1, -C + d1);             
			return ret;

		}else{
			float x1 = (float)(d1 * Math.sqrt(Math.pow(A, 2) + Math.pow(B, 2)) - (B * y1) - C) / A;               
			Vec2 ret = new Vec2(x1, y1);          
			return ret;
		}
	}

	private static List<Vec2> getCircleLineIntersectionVec2(Vec2 pointA, Vec2 pointB, Vec2 center, float radius) {
		float baX = pointB.x - pointA.x;
		float baY = pointB.y - pointA.y;
		float caX = center.x - pointA.x;
		float caY = center.y - pointA.y;

		float a = baX * baX + baY * baY;
		float bBy2 = baX * caX + baY * caY;
		float c = caX * caX + caY * caY - radius * radius;

		float pBy2 = bBy2 / a;
		float q = c / a;

		float disc = pBy2 * pBy2 - q;
		if (disc < 0) {
			return Collections.emptyList();
		}
		// if disc == 0 ... dealt with later
		float tmpSqrt = (float)Math.sqrt(disc);
		float abScalingFactor1 = -pBy2 + tmpSqrt;
		float abScalingFactor2 = -pBy2 - tmpSqrt;

		Vec2 p1 = new Vec2(pointA.x - baX * abScalingFactor1, pointA.y - baY * abScalingFactor1);
		if (disc == 0) { // abScalingFactor1 == abScalingFactor2
			return Collections.singletonList(p1);
		}
		Vec2 p2 = new Vec2(pointA.x - baX * abScalingFactor2, pointA.y - baY * abScalingFactor2);
		return Arrays.asList(p1, p2);
	}


}
