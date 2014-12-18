package se.oru.aass.lucia_meta_csp_lecture.exercises;



import services.sendGoal;
import services.sendGoalRequest;
import services.sendGoalResponse;

import org.apache.commons.logging.Log;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.sensing.ConstraintNetworkAnimator;
import org.metacsp.sensing.InferenceCallback;
import org.metacsp.spatial.geometry.GeometricConstraint;
import org.metacsp.spatial.geometry.GeometricConstraintSolver;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.timelinePlotting.TimelinePublisher;
import org.metacsp.utility.timelinePlotting.TimelineVisualizer;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import se.oru.aass.lucia_meta_csp_lecture.executionMonitoring.ROSDispatchingFunction;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;
import se.oru.aass.lucia_meta_csp_lecture.util.PanelFactory;
import se.oru.aass.lucia_meta_csp_lecture.util.RobotFactory;

public class Ex5  extends AbstractNodeMain {


	private ConnectedNode connectedNode;
	private final String nodeName = "Ex5";
	
	private SpatioTemporalSetNetworkSolver spatioTemporalSetSolver;
	private ActivityNetworkSolver temporalSolver;
	private GeometricConstraintSolver spatialSolver;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(nodeName);
	}

	@Override
	public void onStart(ConnectedNode cn) {
		
		this.connectedNode = cn;
		
		//waiting for ConnectedNode to be up in order to get ROS current time
		while (true) {
			try {
				this.connectedNode.getCurrentTime();
				break;
			}
			catch(NullPointerException e) { }
		}
		
		final Log log = connectedNode.getLog();
		log.info("Lucia CSP Node starting...");
		long origin = connectedNode.getCurrentTime().totalNsecs()/1000000;

		
		//#################################################################################
		//creating solvers
		//#################################################################################
		spatioTemporalSetSolver = new SpatioTemporalSetNetworkSolver(origin,origin+1000000,500,new String[]{});
		//Initialize temporal solver
		temporalSolver = spatioTemporalSetSolver.getActivitySolver();
		//Initialize spatial solver
		spatialSolver = spatioTemporalSetSolver.getGeometricSolver();
		
		
		//#################################################################################
		//creating variables for robots and FoVs of a panel
		//#################################################################################
		//create polygons representing FoVs of a panel
		Vec2 p1 = new Vec2(-0.129f, 1.284f);
		Vec2 p2 = new Vec2(-0.135f, 0.916f);
		Variable[] panelVars = PanelFactory.createPolygonVariables("panel1", p1, p2, spatialSolver);
		
		//creating spatio-temporal variables 
		Vec2 robot1_center = new Vec2(0.0f, 0.0f);
		SpatioTemporalSet turtlebot_1 = (SpatioTemporalSet)RobotFactory.createSpatioTemporalSetVariable("turtlebot_1", robot1_center, 0.0f, spatioTemporalSetSolver);
		turtlebot_1.getActivity().setSymbolicDomain("move_base");
		
		Vec2 robot2_center = new Vec2(-2.0f, -1.0f);
		SpatioTemporalSet turtlebot_2 = (SpatioTemporalSet)RobotFactory.createSpatioTemporalSetVariable("turtlebot_2", robot2_center, 0.0f, spatioTemporalSetSolver);
		turtlebot_2.getActivity().setSymbolicDomain("move_base");

		
		//#################################################################################
		//creating spatial constraints
		//#################################################################################
		//TODO: add constraints to the spatial solver modeling that the robots should be inside of FoVs of panels
		//getPolygon() method of a spatio-temporal variable gives the associated polygon
		GeometricConstraint inside1 = new GeometricConstraint(GeometricConstraint.Type.INSIDE);
		inside1.setFrom(turtlebot_1.getPolygon());
		inside1.setTo(panelVars[0]);
		System.out.println("Added? " + spatialSolver.addConstraint(inside1));
		GeometricConstraint inside2 = new GeometricConstraint(GeometricConstraint.Type.INSIDE);
		inside2.setFrom(turtlebot_2.getPolygon());
		inside2.setTo(panelVars[1]);
		System.out.println("Added? " + spatialSolver.addConstraint(inside2));

		
		InferenceCallback cb = new InferenceCallback() {
			@Override
			public void doInference(long timeNow) { }
		};	
		ConstraintNetworkAnimator animator = new ConstraintNetworkAnimator(temporalSolver, 1000, cb);
		
		/*
		creating ROSDispatchingFunction for turtlebot_1 : triggers a ROS service call when a dispatchable variable appears in the constraint network
		triggers a ROS service call when a dispatchable variable
		appears in the constraint network
		 */
		ROSDispatchingFunction robotDispatchingFunction1 = new ROSDispatchingFunction("turtlebot_1", temporalSolver, this.connectedNode) {
			
			@Override
			public boolean skip(SymbolicVariableActivity act) {
				// TODO Auto-generated method stub
				return false;
			}
			
			@Override
			public void dispatch(SymbolicVariableActivity act) {	
				currentAct = act;
				//TODO: compute a destination for the robot from the component Polygons of the variables
				//getPosition() method of the Polygon class gives the center of the polygon
				Polygon destinationPoly = spatioTemporalSetSolver.getPolygonByActivity(act);
				sendGoal("turtlebot_1", destinationPoly.getPosition().x, destinationPoly.getPosition().y, destinationPoly.getOrientation());
		
			}
		};
		
		/*
		creating ROSDispatchingFunction for turtlebot_2 : triggers a ROS service call when a dispatchable variable appears in the constraint network
		triggers a ROS service call when a dispatchable variable
		appears in the constraint network
		 */
		ROSDispatchingFunction robotDispatchingFunction2 = new ROSDispatchingFunction("turtlebot_2", temporalSolver, this.connectedNode) {
			@Override
			public boolean skip(SymbolicVariableActivity act) {
				// TODO Auto-generated method stub
				return false;
			}
			
			@Override
			public void dispatch(SymbolicVariableActivity act) {
				currentAct = act;
				//TODO: compute a destination for the robot from the component Polygons of the variables
				//getPosition() method of the Polygon class gives the center of the polygon
				Polygon destinationPoly = spatioTemporalSetSolver.getPolygonByActivity(act);				
				sendGoal("turtlebot_2", destinationPoly.getPosition().x, destinationPoly.getPosition().y, destinationPoly.getOrientation());
			}
		};
		//adding dispatchers to the animator
		animator.addDispatchingFunctions(temporalSolver, robotDispatchingFunction1, robotDispatchingFunction2);

		//visualize
		TimelinePublisher tp = new TimelinePublisher((ActivityNetworkSolver)temporalSolver, new Bounds(0,120000), true, "turtlebot_1", "turtlebot_2");
		TimelineVisualizer tv = new TimelineVisualizer(tp);
		tv.startAutomaticUpdate(1000);
		
	}
	
	/*
	Implements the sendGoal services for each robot (e.g., "/turtlebot_1/sendGoal")
	service request is x, y and theta representing the goal pose and orientation (in radian)
	 */
	private void sendGoal(String robot, float x, float y, float theta) {
		ServiceClient<sendGoalRequest, sendGoalResponse> serviceClient = null;
		try { serviceClient = connectedNode.newServiceClient("/"+robot+"/sendGoal", sendGoal._TYPE); }
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		final sendGoalRequest request = serviceClient.newMessage();
		request.setX(x);
		request.setY(y);
		request.setTheta(theta);
		request.setRotationAfter((byte)0);
		serviceClient.call(request, new ServiceResponseListener<sendGoalResponse>() {

			@Override
			public void onSuccess(sendGoalResponse arg0) {System.out.println("Goal sent");}
			@Override
			public void onFailure(RemoteException arg0) { }
		});		
	}

}

