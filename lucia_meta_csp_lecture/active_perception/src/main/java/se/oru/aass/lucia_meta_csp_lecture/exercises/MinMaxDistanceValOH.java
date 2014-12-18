package se.oru.aass.lucia_meta_csp_lecture.exercises;


import java.awt.geom.Point2D;
import java.util.Collections;
import java.util.HashMap;
import java.util.Vector;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Variable;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.spatial.geometry.Vec2;

import se.oru.aass.lucia_meta_csp_lecture.executionMonitoring.ROSTopicSensor;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia_meta_csp_lecture.util.PanelFactory;

public class MinMaxDistanceValOH extends ValueOrderingH{
	
	
	private Vector<ROSTopicSensor> sensors;
	private ConstraintNetwork noGoodCN = null;

	@Override
	public int compare(ConstraintNetwork arg0, ConstraintNetwork arg1) {		
		//arg0 and arg1 are constraints networks representing meta-values of the assignment meta-constraint 
		//meta-values of an assignment meta-constraint are all sets of expected future observations s.t. all set variables in each set are Equal to a different panel
		Constraint[] cons0 = arg0.getConstraints();
		Constraint[] cons1 = arg1.getConstraints();
		
		
		//getAssignments() method gives you a hash map of robots to their associated panels given an array of constraints  
		//E.g., Robot1 -> P3, Robot2 -> P1, Robot3 -> P2 

		
		//TODO: implement the rest ...
		//if the first arg0 is preferable than arg1, return -1
		//if the first arg1 is preferable than arg0, return 1
		//else return 0;
		
		return getMaxDist(cons0) > getMaxDist(cons1) ? 1 : getMaxDist(cons0) < getMaxDist(cons1) ? -1 : 0;
	}
	
	private double getMaxDist(Constraint[] cons){
		Vector<Double> maxDis0 = new Vector<Double>();

		for (int i = 0; i < cons.length; i++) {
			if(cons[i] instanceof SymbolicValueConstraint){
				if(((SymbolicValueConstraint)cons[i]).getType().equals(SymbolicValueConstraint.Type.VALUEEQUALS)){					
					String PanelId = ((SymbolicValueConstraint)cons[i]).getValue()[0];
					Variable var = ((SymbolicValueConstraint)cons[i]).getScope()[0];
					String robotName = ((SpatioTemporalSet)var).getActivity().getComponent();
					Vec2 panelPose = PanelFactory.getPanelCenterById(PanelId);
					Vec2 robotCurentPose = null;
					for (int j = 0; j < sensors.size(); j++) {
						if(sensors.get(j).getRobotName().equals(robotName)){
							robotCurentPose = sensors.get(j).getRobotCurrentPose();  
						}							
					}
					maxDis0.add(Point2D.Float.distance(panelPose.x, panelPose.y, robotCurentPose.x, robotCurentPose.y));
//					System.out.println(robotName + " "+ PanelId + " "+ Point2D.Float.distance(panelPose.x, panelPose.y, robotCurentPose.x, robotCurentPose.y));
				}
			}				
		}
		return Collections.max(maxDis0);
	}
	
	//gives you a hash map of robots to their associated panels given an array of constraints  
	//E.g., Robot1 -> P3, Robot2 -> P1, Robot3 -> P2 
	private HashMap<String, String> getAssignments(Constraint[] cons){
		HashMap<String, String> robotToPanel = new HashMap<String, String>();
		for (int i = 0; i < cons.length; i++) {
			if(cons[i] instanceof SymbolicValueConstraint){
				if(((SymbolicValueConstraint)cons[i]).getType().equals(SymbolicValueConstraint.Type.VALUEEQUALS)){					
					String PanelId = ((SymbolicValueConstraint)cons[i]).getValue()[0];
					Variable var = ((SymbolicValueConstraint)cons[i]).getScope()[0];
					String robotName = ((SpatioTemporalSet)var).getActivity().getComponent();
					robotToPanel.put(robotName, PanelId);
				}
			}				
		}
		return robotToPanel;
	}
	
	public void setSensors(Vector<ROSTopicSensor> sensors) {
		this.sensors = sensors;
	}
	
	public void setNoGoodSolution(ConstraintNetwork cn){
		this.noGoodCN = cn;
	}


}

