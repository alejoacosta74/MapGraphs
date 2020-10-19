/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.*;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	// Maintain both nodes and edges as you will need to
	// be able to look up nodes by lat/lon or by roads
	// that contain those nodes.
	private HashMap<GeographicPoint,MapNode> pointNodeMap;
	private HashSet<MapEdge> edges;

	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		pointNodeMap = new HashMap<GeographicPoint,MapNode>();
		edges = new HashSet<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return pointNodeMap.values().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return pointNodeMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if (location == null) {
			return false;
		}
		MapNode n = pointNodeMap.get(location);
		if (n == null) {
			n = new MapNode(location);
			pointNodeMap.put(location, n);
			return true;
		}
		else {
			return false;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		MapNode n1 = pointNodeMap.get(from);
		MapNode n2 = pointNodeMap.get(to);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+from+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+to+"is not in graph");

		MapEdge edge = new MapEdge(roadName, roadType, n1, n2, length);
		edges.add(edge);
		n1.addEdge(edge);
		
	}
		
	/** 
	 * Get a set of neighbor nodes from a mapNode
	 * @param node  The node to get the neighbors from
	 * @return A set containing the MapNode objects that are the neighbors 
	 * 	of node
	 */
	private Set<MapNode> getNeighbors(MapNode node) {
		return node.getNeighbors();
	}

	private boolean bfsSearch (HashMap<MapNode,MapNode> parentMap,
							   MapNode startNode,
							   MapNode endNode,
							   Consumer<GeographicPoint> nodeSearched){
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		toExplore.add(startNode);
		MapNode next = null;

		while (!toExplore.isEmpty()) {
			next = toExplore.remove();

			// hook for visualization
			nodeSearched.accept(next.getLocation());

			if (next.equals(endNode)) break;
			Set<MapNode> neighbors = getNeighbors(next);
			for (MapNode neighbor : neighbors) {
				if (!visited.contains(neighbor)) {
					visited.add(neighbor);
					parentMap.put(neighbor, next);
					toExplore.add(neighbor);
				}
			}
		}
		if (!next.equals(endNode)) {
			return false;
		}
		else return true;
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, 
			 					     Consumer<GeographicPoint> nodeSearched)
	{
		/* Note that this method is a little long and we might think
		 * about refactoring it to break it into shorter methods as we 
		 * did in the Maze search code in week 2 */
		
		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		boolean found = bfsSearch(parentMap,startNode, endNode,nodeSearched);
		System.out.println("Resultado dijkstraSearch (boolean): " + found);
		if (found){
			// setup to begin BFS
			// Reconstruct the parent path
			List<GeographicPoint> path =
					reconstructPath(parentMap, startNode, endNode);

			return path;
		}
		else {
			System.out.println("No path found from " +start+ " to " + goal);
			return null;
		}

	}
	


	/** Reconstruct a path from start to goal using the parentMap
	 *
	 * @param parentMap the HashNode map of children and their parents
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	private List<GeographicPoint>
	reconstructPath(HashMap<MapNode,MapNode> parentMap,
					MapNode start, MapNode goal)
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = goal;

		while (!current.equals(start)) {
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}

		// add start
		path.addFirst(start.getLocation());
		return path;
	}

	private boolean dijkstraSearch (HashMap<MapNode,MapNode> parentMap,
							   MapNode startNode,
							   MapNode endNode,
							   Consumer<GeographicPoint> nodeSearched){
		Queue<MapNode> toExplore = new PriorityQueue<>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		resetMapDistances();
		startNode.setDistFromStart(0);
		toExplore.add(startNode);
		MapNode next = null;
		int nodeCount = 0;
		while (!toExplore.isEmpty()) {
			next = toExplore.poll();
			nodeCount++;
			System.out.println("Dijkstra visiting node#: "  + nodeCount + " => "+ next);
			if (!visited.contains(next)){
				visited.add(next);
				// hook for visualization
				nodeSearched.accept(next.getLocation());
				if (next.equals(endNode)) break;
				for (MapEdge edge : next.getEdges()) {
					MapNode neighbor = edge.getEndNode();
					if (!visited.contains(neighbor)){
						double pathThroughCurrent = next.getDistFromStart() + edge.getLength();
						if (pathThroughCurrent < neighbor.getDistFromStart()) {
							neighbor.setDistFromStart(pathThroughCurrent);
							parentMap.put(neighbor, next);
							toExplore.add(neighbor);
						}

					}
				}

			}
		}
		if ( next == null || !next.equals(endNode)) {
			return false;
		}

		else {
			return true;
		}
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		boolean found = dijkstraSearch(parentMap,startNode, endNode,nodeSearched);
		if (found){
			// setup to begin BFS
			// Reconstruct the parent path
			List<GeographicPoint> path =
					reconstructPath(parentMap, startNode, endNode);

			return path;
		}
		else {
			System.out.println("No path found from " +start+ " to " + goal);
			return null;
		}
	}

	private void resetMapDistances(){
			for (MapNode node : this.pointNodeMap.values()){
				node.setDistToGoal(0);
				node.setDistFromStart(Double.POSITIVE_INFINITY);
			}
	}

	private boolean aStartSearchAlgorithm (HashMap<MapNode,MapNode> parentMap,
									MapNode startNode,
									MapNode endNode,
									Consumer<GeographicPoint> nodeSearched){
		Queue<MapNode> toExplore = new PriorityQueue<>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		resetMapDistances();
		double predictedDistToGoal = 0;
		double pathThroughCurrent = 0;
		startNode.setDistFromStart(pathThroughCurrent);
		predictedDistToGoal = startNode.getLocation().distance(endNode.getLocation());
		startNode.setDistToGoal(predictedDistToGoal);
		toExplore.add(startNode);
		MapNode next = null;
		int nodeCount =0;
		while (!toExplore.isEmpty()) {
			next = toExplore.remove();
			nodeCount++;
			System.out.println("A* visiting node#: "  + nodeCount + " => "+ next);
			/*
			System.out.println("Dequeued Node: "  + next);
			System.out.println("Dist from start: " + (int)next.getDistFromStart() + " // predicted dist: " + (int)next.getDistToGoal() + " // totalDistance (stored):" + (int)next.getTotalPredictedDistance());
			if (parentMap.get(next) != null){
				System.out.println("Parent in parentMap: "  + parentMap.get(next));
			}
			 */

			if (!visited.contains(next)){
				visited.add(next);
				// hook for visualization
				nodeSearched.accept(next.getLocation());
				if (next.equals(endNode)) break;
				for (MapEdge edge : next.getEdges()) {
					MapNode neighbor = edge.getEndNode();
					if (!visited.contains(neighbor)){
						pathThroughCurrent = next.getDistFromStart() + edge.getLength();
						if (pathThroughCurrent < neighbor.getDistFromStart()) {
							neighbor.setDistFromStart(pathThroughCurrent);
							predictedDistToGoal = neighbor.getLocation().distance(endNode.getLocation());
							neighbor.setDistToGoal(predictedDistToGoal);
							/**
							 * If neighbor node is already in parentMap
							 *  make sure parent node is the parent with shortest g(n) + h(n)
							 */

							if (parentMap.containsKey(neighbor)){
								MapNode currentParent = parentMap.get(neighbor);
								if (currentParent.getTotalPredictedDistance() > next.getTotalPredictedDistance()){
									parentMap.put(neighbor, next);
								}
							}
							else {parentMap.put(neighbor, next);}
							toExplore.add(neighbor);
							//System.out.println("\t Enqueued Neighbor: "  + neighbor);
							//System.out.println("\t Dist from start: " + (int)neighbor.getDistFromStart() + " // predicted dist: " + (int)neighbor.getDistToGoal() + " // totalDistance (stored):" + (int)neighbor.getTotalPredictedDistance() + "\n");

						}

					}
				}

			}
		}
		if ( next == null || !next.equals(endNode)) {
			return false;
		}

		else {
			return true;
		}
	}


	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		boolean found = aStartSearchAlgorithm(parentMap,startNode, endNode,nodeSearched);
		System.out.println("Resultado aStartSearch (boolean): " + found);
		if (found){
			// Reconstruct the parent path
			List<GeographicPoint> path =
					reconstructPath(parentMap, startNode, endNode);
			return path;
		}
		else {
			System.out.println("No path found from " +start+ " to " + goal);
			return null;
		}
	}

	public void printPath (List<GeographicPoint> route){
		if (route == null){
			System.err.println("Path not found. Nothing to print \n");
			return;
		}
		System.out.println("Printing path");
		int i = 0;
		for (GeographicPoint p : route){
			i++;
			System.out.println("Point: " + i + " (" + p + ")");
		}
	}

	public void printGraph (){
		System.out.println("\n Printing MapGraph: ");
		for (GeographicPoint p: pointNodeMap.keySet()){
			System.out.println("\n Vertex: " + p + " \t Edges: \n");
			for (MapEdge e: pointNodeMap.get(p).getEdges()){
				System.out.println("\t \t " + e );
			}
		}
	}

	
	
	public static void main(String[] args)
	{
		/*
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		theMap.printGraph();
		GeographicPoint start = new GeographicPoint(7, 3);
		GeographicPoint end = new GeographicPoint(4, -1);
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route = theMap.aStarSearch(start,end);
		theMap.printPath(route);

		 */

		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);


		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		//simpleTestMap.printPath(testroute);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		//simpleTestMap.printPath(testroute2);



		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);


		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);

		 */

		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);


	}
	
}
