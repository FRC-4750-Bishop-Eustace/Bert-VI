package org.usfirst.frc.team4750.robot.pathfinding;

import java.awt.Point;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import org.usfirst.frc.team4750.robot.commands.DriveToDistance;
import org.usfirst.frc.team4750.robot.commands.GoToMidPos;
import org.usfirst.frc.team4750.robot.commands.RotateToBottom;
import org.usfirst.frc.team4750.robot.commands.TurnToAngle;

import edu.wpi.first.wpilibj.command.Command;

public class SquareGraph {

	private Node[][] map;
	private Point startPosition;
	private Point targetPosition;
	private Heap<Node> openNodes;
	private Set<Node> closedNodes;
	public ArrayList<Node> originalPath;
	private boolean allowDiagonal = false;

	public SquareGraph(int mapDimension) {
		map = new Node[mapDimension][mapDimension];
		startPosition = new Point();
		targetPosition = new Point();
		openNodes = new Heap<Node>();
		closedNodes = new HashSet<Node>();
	}

	public Node getMapCell(Point coord) {
		return map[(int) coord.getX()][(int) coord.getY()];
	}

	public Node getNode(int x, int y) {
		return map[x][y];
	}

	public void setMapCell(Point coord, Node n) {
		map[(int) coord.getX()][(int) coord.getY()] = n;
	}

	public Point getStartPosition() {
		return startPosition;
	}

	public Point getTargetPosition() {
		return targetPosition;
	}

	public void setStartPosition(Point coord) {
		startPosition.setLocation(coord);
	}

	public void setTargetPosition(Point coord) {
		targetPosition.setLocation(coord);
	}

	public int getDimension() {
		return map.length;
	}

	public void addToOpenNodes(Node n) {
		n.setOpen();
		openNodes.add(n);
	}

	public Node popBestOpenNode() {
		return openNodes.remove();
	}

	public void addToClosedNodes(Node n) {
		n.setClosed();
		closedNodes.add(n);
	}

	public boolean isPassable(int x, int y) {
		return map[x][y].isNormal();
	}

	public boolean isInsideMap(Point p) {
		return ((p.getX() >= 0) && (p.getX() < getDimension()) && (p.getY() >= 0) && (p.getY() < getDimension()));
	}

	public Set<Node> getNeighbours(Node n) {
		Set<Node> neighbours = new HashSet<Node>();
		for (int i = -1; i <= 1; i++) {
			for (int j = -1; j <= 1; j++) {
				if (!allowDiagonal) {
					if (!(Math.abs(i) == Math.abs(j)))
						if (isInsideMap(new Point(n.getX() + i, n.getY() + j))) {
							Node temp = getMapCell(new Point(n.getX() + i, n.getY() + j));
							if (!temp.isObstacle())
								neighbours.add(temp);
						}
				} else {
					if (!(i == 0 && j == 0))
						if (isInsideMap(new Point(n.getX() + i, n.getY() + j))) {
							Node temp = getMapCell(new Point(n.getX() + i, n.getY() + j));
							if (!temp.isObstacle())
								neighbours.add(temp);
						}
				}

			}
		}
		return neighbours;
	}

	double calculateDistance(Point from, Point to) {
		if (allowDiagonal) {
			return Math.max(Math.abs(from.getX() - to.getX()), Math.abs(from.getY() - to.getY()));
		}
		return Math.abs(from.getX() - to.getX()) + Math.abs(from.getY() - from.getY());
	}

	static double calculateDistance(Node from, Node to) {
		return Math.pow(Math.pow(from.getX() - to.getX(), 2) + Math.pow(from.getY() - to.getY(), 2), 0.5);
	}

	public ArrayList<Node> compressPath(ArrayList<Node> path) {
		ArrayList<Node> remove = new ArrayList<Node>();
		for (int i = 1; i <= path.size() - 2; i++) {
			int initialX = (path.get(i).getX() - path.get(i - 1).getX());
			int initialY = (path.get(i).getY() - path.get(i - 1).getY());
			int finalX = (path.get(i + 1).getX() - path.get(i).getX());
			int finalY = (path.get(i + 1).getY() - path.get(i).getY());
			if ((initialX == 0 && finalX == 0) || (initialY == 0 && finalY == 0)) {
				Node toRemove = path.get(i);
				remove.add(toRemove);
			}
		}
		ArrayList<Node> compressedPath = new ArrayList<Node>();
		compressedPath.addAll(path);
		for (int i = 0; i < compressedPath.size(); i++) {
			for (int j = 0; j < remove.size(); j++) {
				if (compressedPath.get(i).equals(remove.get(j))) {
					compressedPath.remove(i);
				}
			}
		}
		path = compressedPath;
		return path;
	}

	public ArrayList<Node> reconstructPath(Node target) {
		ArrayList<Node> path = new ArrayList<Node>();
		Node current = target;

		while (current.getParent() != null) {
			path.add(current.getParent());
			current = current.getParent();
		}
		Collections.reverse(path);
		path.add(target);
		originalPath = path;
		path = compressPath(path);
		return path;
	}

	public ArrayList<Node> reduceTurns(ArrayList<Node> path) {
		int i = 1;
		while (i < path.size() - 1) {
			if (shiftNode(i, path) == null) {
				printPath(path);
				i++;
				System.out.println(i);
				continue;
			}
			System.out.println("Original Node: (" + path.get(i).getX() + ", " + path.get(i).getY() + ")");
			Node shiftedNode = shiftNode(i, path);
			path.set(i, shiftedNode);
			System.out.println("Shifted Node: (" + path.get(i).getX() + ", " + path.get(i).getY() + ")");

			path = compressPath(path);

			printPath(path);
			i = 1;
			System.out.println(i);
		}
		return path;

	}

	public Node shiftNode(int i, ArrayList<Node> path) {
		Node shiftedNode;
		if (path.get(i).getX() == path.get(i - 1).getX() && path.get(i).getY() == path.get(i + 1).getY()) {
			shiftedNode = getNode(path.get(i + 1).getX(), path.get(i - 1).getY());
		} else {
			shiftedNode = getNode(path.get(i - 1).getX(), path.get(i + 1).getY());
		}
		// If the shifted node is an obstacle, it would be invalid
		if (shiftedNode.isObstacle()) {
			return null;
		}
		// This part tests to see if there is an obstacle between the shifted node and
		// the previous node
		// If there is an obstacle, the path would be invalid and the program returns a
		// null value
		if (shiftedNode.getX() == path.get(i - 1).getX()) {
			if (shiftedNode.getY() < path.get(i - 1).getY()) {
				for (int j = shiftedNode.getY(); j <= path.get(i - 1).getY(); j++) {
					Node betweenNode = getNode(shiftedNode.getX(), j);
					if (betweenNode.isObstacle()) {
						return null;
					}
				}
			} else {
				for (int j = path.get(i - 1).getY(); j <= shiftedNode.getY(); j++) {
					Node betweenNode = getNode(shiftedNode.getX(), j);
					if (betweenNode.isObstacle()) {
						return null;
					}
				}
			}
		} else {
			if (shiftedNode.getX() < path.get(i - 1).getX()) {
				for (int j = shiftedNode.getX(); j <= path.get(i - 1).getX(); j++) {
					Node betweenNode = getNode(j, shiftedNode.getY());
					if (betweenNode.isObstacle()) {
						return null;
					}
				}
			} else {
				for (int j = path.get(i - 1).getX(); j <= shiftedNode.getX(); j++) {
					Node betweenNode = getNode(j, shiftedNode.getY());
					if (betweenNode.isObstacle()) {
						return null;
					}
				}
			}
		}
		// This part tests to see if there is an obstacle between the shifted node and
		// the next node
		// If there is an obstacle, the path would be invalid and the program returns a
		// null value
		if (shiftedNode.getX() == path.get(i + 1).getX()) {
			if (shiftedNode.getY() < path.get(i + 1).getY()) {
				for (int j = shiftedNode.getY(); j <= path.get(i + 1).getY(); j++) {
					Node betweenNode = getNode(shiftedNode.getX(), j);
					if (betweenNode.isObstacle()) {
						return null;
					}
				}
			} else {
				for (int j = path.get(i + 1).getY(); j <= shiftedNode.getY(); j++) {
					Node betweenNode = getNode(shiftedNode.getX(), j);
					if (betweenNode.isObstacle()) {
						return null;
					}
				}
			}
		} else {
			if (shiftedNode.getX() < path.get(i + 1).getX()) {
				for (int j = shiftedNode.getX(); j <= path.get(i + 1).getX(); j++) {
					Node betweenNode = getNode(j, shiftedNode.getY());
					if (betweenNode.isObstacle()) {
						return null;
					}
				}
			} else {
				for (int j = path.get(i + 1).getX(); j <= shiftedNode.getX(); j++) {
					Node betweenNode = getNode(j, shiftedNode.getY());
					if (betweenNode.isObstacle()) {
						return null;
					}
				}
			}
		}
		// If the shifted node is not an obstacle and has an open path to the previous
		// and next node, it is valid
		return shiftedNode;
	}

	public void printPath(ArrayList<Node> path) {
		for (int i = 0; i < path.size(); i++) {
			Node node = path.get(i);
			System.out.println("node : (" + node.getX() + "," + node.getY() + ")");
		}
	}

	public void printDrivePath(ArrayList<String> drivePath) {
		for (int i = 0; i < drivePath.size(); i++) {
			String step = drivePath.get(i);
			System.out.println(step);
		}
	}

	public ArrayList<Node> executeAStar() {
		Node start = getMapCell(getStartPosition());
		Node target = getMapCell(getTargetPosition());
		addToOpenNodes(start);

		start.setCostFromStart(0);
		start.setTotalCost(start.getCostFromStart() + calculateDistance(start.getPosition(), target.getPosition()));
		while (!openNodes.isEmpty()) {
			Node current = popBestOpenNode();
			if (current.equals(target)) {
				return reconstructPath(target);
			}
			addToClosedNodes(current);
			Set<Node> neighbours = getNeighbours(current);
			for (Node neighbour : neighbours) {
				if (!neighbour.isClosed()) {
					double tentativeCost = current.getCostFromStart()
							+ calculateDistance(current.getPosition(), neighbour.getPosition());
					if ((!neighbour.isOpen()) || (tentativeCost < neighbour.getCostFromStart())) {
						neighbour.setParent(current);
						neighbour.setCostFromStart(tentativeCost);
						neighbour.setTotalCost(neighbour.getCostFromStart()
								+ calculateDistance(neighbour.getPosition(), start.getPosition()));
						if (!neighbour.isOpen())
							addToOpenNodes(neighbour);
					}
				}
			}
		}
		return null;
	}

	public void translatePath(ArrayList<Node> path) {
		for (int i = 0; i < path.size() - 1; i++) {
			if (i < path.size() - 2) {
				Node first1 = path.get(i);
				Node second1 = path.get(i + 1);
				System.out.println("Drive " + calculateDistance(first1, second1) + " feet");
				if (turnRight(path, i)) {
					System.out.println("Turn right");
				} else {
					System.out.println("Turn left");
				}
			} else {
				Node first2 = path.get(i);
				Node second2 = path.get(i + 1);
				System.out.println("Drive " + calculateDistance(first2, second2) + " feet");
			}
		}
	}

	public ArrayList<Command> makeDrivePath(ArrayList<Node> path) {
		ArrayList<Command> drivePath = new ArrayList<Command>();
		drivePath.add(new RotateToBottom());
		drivePath.add(new GoToMidPos());
		for (int i = 0; i < path.size() - 1; i++) {
			if (i < path.size() - 2) {
				Node first1 = path.get(i);
				Node second1 = path.get(i + 1);
				drivePath.add(new DriveToDistance((int) calculateDistance(first1, second1), true));
				if (turnRight(path, i)) {
					drivePath.add(new TurnToAngle(90));
				} else {
					drivePath.add(new TurnToAngle(-90));
				}
			} else {
				Node first2 = path.get(i);
				Node second2 = path.get(i + 1);
				drivePath.add(new DriveToDistance((int) calculateDistance(first2, second2), true));
			}
		}
		return drivePath;
	}

	public void printMap(char[][] map1, ArrayList<Node> path) {
		for (int i = 0; i < map1.length; i++) {
			for (int j = 0; j < map1.length; j++) {
				if (j == startPosition.getX() && i == startPosition.getY()) {
					System.out.print("A ");
				} else if (j == targetPosition.getX() && i == targetPosition.getY()) {
					System.out.print("B ");
				} else if (getNode(j, i).isNormal() && path.contains(getNode(j, i))) {
					System.out.print("O ");
				} else {
					System.out.print(map1[i][j] + " ");
				}
			}
			System.out.println("");
		}
	}

	public int getPathOrientation(ArrayList<Node> path, int nodeNumber) {
		Node node1 = path.get(nodeNumber);
		Node node2 = path.get(nodeNumber + 1);
		int pathOrientation;
		if (node1.getX() == node2.getX()) {
			if (node1.getY() < node2.getY()) {
				pathOrientation = 90;
			} else {
				pathOrientation = 270;
			}
		} else {
			if (node1.getX() < node2.getX()) {
				pathOrientation = 0;
			} else {
				pathOrientation = 180;
			}
		}
		return pathOrientation;
	}

	public boolean turnRight(ArrayList<Node> path, int nodeNumber) {
		int orientation1 = getPathOrientation(path, nodeNumber);
		int orientation2 = getPathOrientation(path, nodeNumber + 1);
		if (90 == Math.abs(orientation1 - orientation2)) {
			if (orientation2 > orientation1) {
				return false;
			}
			return true;
		}
		if (orientation2 > orientation1) {
			return true;
		}
		return false;
	}

}