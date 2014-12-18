package se.oru.aass.lucia_meta_csp_lecture;

import java.util.Vector;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.spatial.geometry.GeometricConstraint;
import org.metacsp.spatial.geometry.GeometricConstraintSolver;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;
import org.metacsp.utility.UI.PolygonFrame;


public class TestGeometricConstraintSolver {

	public static void main(String[] args) {

		GeometricConstraintSolver solver = new GeometricConstraintSolver();
		Variable[] vars = solver.createVariables(3);
		
		Polygon p0 = (Polygon)vars[0];
		Vector<Vec2> vecs1 = new Vector<Vec2>();
		vecs1.add(new Vec2(100,87));
		vecs1.add(new Vec2(60,30));
		vecs1.add(new Vec2(220,60));
		vecs1.add(new Vec2(180,120));
		p0.setDomain(vecs1.toArray(new Vec2[vecs1.size()]));
		p0.setMovable(true);
		
		Polygon p1 = (Polygon)vars[1];		
		Vector<Vec2> vecs = new Vector<Vec2>();
		vecs.add(new Vec2(180,90));
		vecs.add(new Vec2(100,350));
		vecs.add(new Vec2(340,350));
		vecs.add(new Vec2(290,125));
		p1.setDomain(vecs.toArray(new Vec2[vecs.size()]));
		p1.setMovable(false);
		
		Polygon p2 = (Polygon)vars[2];		
		Vector<Vec2> vecs2 = new Vector<Vec2>();
		vecs2.add(new Vec2(180,190));
		vecs2.add(new Vec2(100,50));
		vecs2.add(new Vec2(240,138));
		vecs2.add(new Vec2(190,225));
		p2.setDomain(vecs2.toArray(new Vec2[vecs2.size()]));
		p2.setMovable(true);
		
		ConstraintNetwork.draw(solver.getConstraintNetwork());
		PolygonFrame pf = new PolygonFrame("Polygon Constraint Network", solver.getConstraintNetwork());
		
		try { Thread.sleep(1000); }
		catch (InterruptedException e) { e.printStackTrace(); }

		GeometricConstraint inside = new GeometricConstraint(GeometricConstraint.Type.INSIDE);
		inside.setFrom(p0);
		inside.setTo(p1);
		System.out.println("Added? " + solver.addConstraint(inside));

		try { Thread.sleep(1000); }
		catch (InterruptedException e) { e.printStackTrace(); }

		GeometricConstraint dc1 = new GeometricConstraint(GeometricConstraint.Type.DC);
		dc1.setFrom(p2);
		dc1.setTo(p1);
		System.out.println("Added? " + solver.addConstraint(dc1));

		try { Thread.sleep(1000); }
		catch (InterruptedException e) { e.printStackTrace(); }

		GeometricConstraint inside1 = new GeometricConstraint(GeometricConstraint.Type.INSIDE);
		inside1.setFrom(p2);
		inside1.setTo(p0);
		System.out.println("Added? " + solver.addConstraint(inside1));

//		try { Thread.sleep(1000); }
//		catch (InterruptedException e) { e.printStackTrace(); }
//
//		solver.removeConstraint(inside);

		try { Thread.sleep(1000); }
		catch (InterruptedException e) { e.printStackTrace(); }

		GeometricConstraint dc = new GeometricConstraint(GeometricConstraint.Type.DC);
		dc.setFrom(p0);
		dc.setTo(p1);
		System.out.println("Added? " + solver.addConstraint(dc));
		
	}

}
