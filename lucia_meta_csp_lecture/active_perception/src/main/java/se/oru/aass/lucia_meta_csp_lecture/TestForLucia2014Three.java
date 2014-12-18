/*******************************************************************************
 * Copyright (c) 2010-2013 Federico Pecora <federico.pecora@oru.se>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************/
package se.oru.aass.lucia_meta_csp_lecture;

import java.util.HashSet;
import java.util.Random;
import java.util.Vector;
import java.util.logging.Level;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.multi.symbols.SymbolicValueConstraint.Type;
import org.metacsp.multi.symbols.SymbolicVariableConstraintSolver;
import org.metacsp.utility.logging.MetaCSPLogging;

public class TestForLucia2014Three {
	
	public static void main(String[] args) {
		MetaCSPLogging.setLevel(Level.INFO);
		
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
		
		//We make a constraint network to hold variables representing
		// robots and what they see
		ActivityNetworkSolver solver = new ActivityNetworkSolver(0, 1000000, symbols);
		SymbolicVariableConstraintSolver groundSolver = ((SymbolicVariableConstraintSolver)solver.getConstraintSolvers()[1]);
		groundSolver.setSingleValue(false);
		groundSolver.setEnumerateSets(false);
		
		//Vars representing robots and what panels (if any) they see
		int numRobots = 5;
		Variable[] robots = new Variable[numRobots];
		for (int i = 0; i < numRobots; i++) robots[i] = solver.createVariable("Robot"+i+" sees");

		//Randomly choose robots (as many as there are panels)
		Random rand = new Random(1234431);
		HashSet<Variable> chosenRobots = new HashSet<Variable>();
		for (int i = 0; i < panels.length; i++)
			while (!chosenRobots.add(robots[rand.nextInt(numRobots)])) {}
		
		//Force every chosen robot to see one of the panels (w/o deciding which panel)		
		Vector<Constraint> cons = new Vector<Constraint>();
		for (Variable robot : chosenRobots) {
			SymbolicValueConstraint con = new SymbolicValueConstraint(Type.VALUESUBSET);
			con.setValue(panels);
			con.setFrom(robot);
			con.setTo(robot);
			cons.add(con);
		}
		
		//Force all chosen robots to see a different panel
		SymbolicValueConstraint con = new SymbolicValueConstraint(Type.DIFFERENT);
		con.setScope(chosenRobots.toArray(new Variable[chosenRobots.size()]));
		cons.add(con);
		
		ConstraintNetwork.draw(solver.getConstraintNetwork());
		
		System.out.println("Added constraints? " + solver.addConstraints(cons.toArray(new Constraint[cons.size()])));
		System.out.println("Done.");
		
		SymbolicValueConstraint aRobotSeesPanel3 = new SymbolicValueConstraint(Type.VALUEEQUALS);
		Variable aRobot = chosenRobots.iterator().next();
		aRobotSeesPanel3.setValue(panels[2]);
		aRobotSeesPanel3.setFrom(aRobot);
		aRobotSeesPanel3.setTo(aRobot);
		solver.addConstraint(aRobotSeesPanel3);

		
//		System.out.println(solver.getDescription());

	}

}
