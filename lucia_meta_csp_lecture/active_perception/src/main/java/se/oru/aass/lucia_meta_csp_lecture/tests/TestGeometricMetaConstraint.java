package se.oru.aass.lucia_meta_csp_lecture.tests;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.List;
import java.util.Vector;
import java.util.logging.Level;

import services.getPanel;
import services.getPanelRequest;
import services.getPanelResponse;

import org.apache.commons.logging.Log;
import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.framework.meta.MetaVariable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.multi.symbols.SymbolicVariableConstraintSolver;
import org.metacsp.sensing.ConstraintNetworkAnimator;
import org.metacsp.sensing.InferenceCallback;
import org.metacsp.spatial.geometry.GeometricConstraintSolver;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.UI.PolygonFrame;
import org.metacsp.utility.logging.MetaCSPLogging;
import org.metacsp.utility.timelinePlotting.TimelinePublisher;
import org.metacsp.utility.timelinePlotting.TimelineVisualizer;
import org.ros.concurrent.CancellableLoop;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.Duration;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;

import se.oru.aass.lucia_meta_csp_lecture.executionMonitoring.ROSDispatchingFunction;
import se.oru.aass.lucia_meta_csp_lecture.executionMonitoring.ROSTopicSensor;
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.AssignmentMetaConstraint;
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.LuciaMetaConstraintSolver;
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.ObservabilityMetaConstraint;
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.SimpleMoveBasePlanner;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;
import se.oru.aass.lucia_meta_csp_lecture.util.PanelFactory;
import se.oru.aass.lucia_meta_csp_lecture.util.PanelMarkerPublisher;
import se.oru.aass.lucia_meta_csp_lecture.util.RobotFactory;
import se.oru.aass.lucia_meta_csp_lecture.*;
import visualization_msgs.Marker;
import visualization_msgs.MarkerArray;


public class TestGeometricMetaConstraint extends AbstractNodeMain {

	private ConnectedNode connectedNode;
	private final String nodeName = "lucia_meta_csp_lecture";
	
	private LuciaMetaConstraintSolver metaSolver;
	private SpatioTemporalSetNetworkSolver spatioTemporalSetSolver;
	private ActivityNetworkSolver temporalSolver;
	private GeometricConstraintSolver spatialSolver;
	private SymbolicVariableConstraintSolver setSolver;
	private ParameterTree params;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(nodeName);
	}

	private void getPanelsFromROSService(final String[] panelNames) {
		ServiceClient<getPanelRequest, getPanelResponse> serviceClient = null;
		
		boolean print = false;
		while (true)
		{
			try {
				serviceClient = connectedNode.newServiceClient("getPanel", getPanel._TYPE);			}
			catch (org.ros.exception.ServiceNotFoundException e) {
				System.out.println("waiting for service 'getPanel'...");
				print = true;
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e1) {
				}
				continue;
			}

			break;
		}
		if (print)
			System.out.println("... done waiting for getPanel service.");
		

		final getPanelRequest request = serviceClient.newMessage();
		request.setRead((byte) 0);
		serviceClient.call(request, new ServiceResponseListener<getPanelResponse>() {
			
			@Override
			public void onSuccess(getPanelResponse arg0) {
				
				//Build the panel polygons
				final ArrayList<Polygon> polygons = new ArrayList<Polygon>();
				final HashMap<Polygon,String> polygonsToPanelNamespaces = new HashMap<Polygon,String>();
				
				List<Integer> hiddenPanelSideParam = (List<Integer>)params.getList("/" + nodeName + "/hidden_panel_sides");
				for(int i = 1; i <= panelNames.length; i++) {
					try {
						boolean skipFirst = false;
						boolean skipSecond = false;
						String panelNumber = panelNames[i-1].substring(1);
						int panelNumberInt = Integer.parseInt(panelNumber);
						if (hiddenPanelSideParam.get(panelNumberInt-1) == 1) { skipFirst = true; }
						else if (hiddenPanelSideParam.get(panelNumberInt-1) == 2) { skipSecond = true; }
						double x1 = (Double) arg0.getClass().getMethod("getPanel" + panelNumber + "X1", new Class[]{}).invoke(arg0, new Object[]{});
						double y1 = (Double) arg0.getClass().getMethod("getPanel" + panelNumber + "Y1", new Class[]{}).invoke(arg0, new Object[]{});
						double x2 = (Double) arg0.getClass().getMethod("getPanel" + panelNumber + "X2", new Class[]{}).invoke(arg0, new Object[]{});
						double y2 = (Double) arg0.getClass().getMethod("getPanel" + panelNumber + "Y2", new Class[]{}).invoke(arg0, new Object[]{});
						Variable[] polyVars = PanelFactory.createPolygonVariables(panelNames[i-1], new Vec2((float)x1,(float)y1), new Vec2((float)x2,(float)y2), spatialSolver, skipFirst, skipSecond);
						polygons.add((Polygon)polyVars[0]);
						if (skipFirst) polygonsToPanelNamespaces.put((Polygon)polyVars[0], "Panel " + panelNames[i-1] + " FoV 2");
						else polygonsToPanelNamespaces.put((Polygon)polyVars[0], "Panel " + panelNames[i-1] + " FoV 1");
						if (!skipSecond) {
							if (skipFirst) {
								polygons.add((Polygon)polyVars[0]);
								polygonsToPanelNamespaces.put((Polygon)polyVars[0], "Panel " + panelNames[i-1] + " FoV 2");
							}
							else {
								polygons.add((Polygon)polyVars[1]);
								polygonsToPanelNamespaces.put((Polygon)polyVars[1], "Panel " + panelNames[i-1] + " FoV 2");
							}
						}
					}
					catch (Exception e) { e.printStackTrace(); }
				}
				new PanelMarkerPublisher(polygons, polygonsToPanelNamespaces, connectedNode);
			}
			
			@Override
			public void onFailure(RemoteException arg0) {
				throw new RosRuntimeException(arg0);				
			}
		});
	}

	@Override
	public void onStart(ConnectedNode cn) {
		this.connectedNode = cn;
		
//		MetaCSPLogging.setLevel(LuciaMetaConstraintSolver.class, Level.FINE);
		
		while (true) {
			try {
				this.connectedNode.getCurrentTime();
				break;
			}
			catch(NullPointerException e) { }
		}
		
		final Log log = connectedNode.getLog();
		log.info("Lucia CSP Node starting...");

		params = connectedNode.getParameterTree();
				
		//Make symbol names (including panels)
		List<Integer> panelParam = (List<Integer>)params.getList("/" + nodeName + "/used_panels");
		String[] panelNames = new String[panelParam.size()];
		String[] symbols = new String[panelParam.size()+1];
		for (int i = 0; i < panelParam.size(); i++) {
			panelNames[i] = "P"+panelParam.get(i);
			symbols[i] = "P"+panelParam.get(i);
		}
		//Another symbol ("None") represents the fact that a robot sees no panel
		symbols[panelParam.size()] = "None";
		
		long origin = connectedNode.getCurrentTime().totalNsecs()/1000000;
		metaSolver = new LuciaMetaConstraintSolver(origin,origin+1000000,500,symbols);
		spatioTemporalSetSolver = (SpatioTemporalSetNetworkSolver)metaSolver.getConstraintSolvers()[0];
		temporalSolver = spatioTemporalSetSolver.getActivitySolver();
		spatialSolver = spatioTemporalSetSolver.getGeometricSolver();
		setSolver = spatioTemporalSetSolver.getSetSolver();
		
		//TODO: ORDER: scheduling, heur, nogood, open-ended (sum dist, observability heur)
		//heuristic 
//		final MinMaxDistanceValOH minMaxDistanceValueOH = new MinMaxDistanceValOH();
		
		getPanelsFromROSService(panelNames);
		InferenceCallback cb = new InferenceCallback() {
			
			@Override
			public void doInference(long timeNow) {
				metaSolver.clearResolvers();
				metaSolver.backtrack();
				Vector<SymbolicVariableActivity> moveBaseActivities = new Vector<SymbolicVariableActivity>();
				ConstraintNetwork[] cns = metaSolver.getAddedResolvers();
				if (cns != null && cns.length > 0) {
					
					//Find activities to dispatch and current assignment
					for (ConstraintNetwork cn : cns) {
						if (cn.getAnnotation() instanceof SimpleMoveBasePlanner) {
							for (Variable var : cn.getVariables()) {
								SymbolicVariableActivity act = (SymbolicVariableActivity)var;
								if (act.getSymbolicVariable().getSymbols()[0].equals("move_base"))
									moveBaseActivities.add(act);
							}
						}
					
						//Signal current assignment as "nogood" for heuristic
						if (cn.getAnnotation() instanceof AssignmentMetaConstraint) {
//							minMaxDistanceValueOH.setNoGoodSolution(cn);
						}
					}
					//Anchor activities to dispatch to current time
					for (Variable act : moveBaseActivities) {
						AllenIntervalConstraint release = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(timeNow,APSPSolver.INF));
						release.setFrom(act);
						release.setTo(act);
						temporalSolver.addConstraint(release);
					}					
				}
			}
		};
		
		//Last arg: pass true to start paused
		//ConstraintNetworkAnimator animator = new ConstraintNetworkAnimator(activitySolver, 1000, cb, false);
		ConstraintNetworkAnimator animator = new ConstraintNetworkAnimator(temporalSolver, 1000, cb, false) {
			@Override
			protected long getCurrentTimeInMillis() {
				return connectedNode.getCurrentTime().totalNsecs()/1000000;
			}
		};

		//Vars representing robots and what panels (if any) they see
		List<Integer> robotParam = (List<Integer>)params.getList("/" + nodeName + "/used_robots");
		String[] robotTimelines = new String[robotParam.size()];
		Vector<ROSTopicSensor> sensors = new Vector<ROSTopicSensor>();
		for (int i = 0; i < robotParam.size(); i++) {
			robotTimelines[i] = "turtlebot_"+robotParam.get(i);
			ROSTopicSensor sensor = new ROSTopicSensor(robotTimelines[i], animator, metaSolver, connectedNode);
			ROSDispatchingFunction df = new ROSDispatchingFunction(robotTimelines[i], metaSolver, connectedNode, sensor);
			animator.addDispatchingFunctions(temporalSolver, df);
			sensors.add(sensor);
		}
		
//		minMaxDistanceValueOH.setSensors(sensors);

		//TODO: Remove (they know how to add a metacon)
//		AssignmentMetaConstraint mc1 = new AssignmentMetaConstraint(null, minMaxDistanceValueOH);
		AssignmentMetaConstraint mc1 = new AssignmentMetaConstraint(null, null);

		mc1.setPanels(panelNames);
		metaSolver.addMetaConstraint(mc1);

		SimpleMoveBasePlanner mc2 = new SimpleMoveBasePlanner(null, null);
		metaSolver.addMetaConstraint(mc2);

		ObservabilityMetaConstraint mc3 = new ObservabilityMetaConstraint(null, null);
		metaSolver.addMetaConstraint(mc3);

//		//TODO: remove, this is the result of ex ?.
//		SchedulingMetaConstraint mc4 = new SchedulingMetaConstraint(null, null);
//		metaSolver.addMetaConstraint(mc4);
		
		
		//#################################################################################
		//visualize
		//#################################################################################
//		ConstraintNetwork.draw(spatioTemporalSetSolver.getConstraintNetwork(), "SpatioTemporalSet network");
//		ConstraintNetwork.draw(activitySolver.getConstraintNetwork(),"Activity network");
//		ConstraintNetwork.draw(geometricSolver.getConstraintNetwork(),"Polygon network");
//		PolygonFrame pf = new PolygonFrame("Polygon Constraint Network", geometricSolver.getConstraintNetwork()/*,850.0f*/);

		TimelinePublisher tp = new TimelinePublisher((ActivityNetworkSolver)temporalSolver, new Bounds(0,120000), true, robotTimelines);
		TimelineVisualizer tv = new TimelineVisualizer(tp);
		tv.startAutomaticUpdate(1000);
	}

}
