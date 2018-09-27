package org.usfirst.frc.team4750.robot.pathfinding;

import java.awt.Point;

public class InputHandler {

	public SquareGraph readMap(Point startPoint, Point targetPoint)
			throws InvalidLetterException {
		
		Map mapClass = new Map();

		int mapDimension = mapClass.getMap().length;
		char[][] map = mapClass.getMap();
		SquareGraph graph = new SquareGraph(mapDimension);

		Node start = new Node((int) startPoint.getX(), (int) startPoint.getY(), "NORMAL");
		graph.setMapCell(new Point((int) startPoint.getX(), (int) startPoint.getY()), start);
		graph.setStartPosition(new Point((int) startPoint.getX(), (int) startPoint.getY()));

		Node target = new Node((int) targetPoint.getX(), (int) targetPoint.getY(), "NORMAL");
		graph.setMapCell(new Point((int) targetPoint.getX(), (int) targetPoint.getY()), target);
		graph.setTargetPosition(new Point((int) targetPoint.getX(), (int) targetPoint.getY()));

		for (int i = 0; i < mapDimension; i++) {
			for (int j = 0; j < mapDimension; j++) {
				char typeSymbol = map[j][i];
				if (typeSymbol == ' ') {
					Node normal = new Node(i, j, "NORMAL");
					graph.setMapCell(new Point(i, j), normal);
				} else if (typeSymbol == 'X') {
					Node obstacle = new Node(i, j, "OBSTACLE");
					graph.setMapCell(new Point(i, j), obstacle);
				} else {
					throw new InvalidLetterException(
							"There was a wrong character in the array.The character must be 'X' or ' '.");
				}
			}
		}
		return graph;
	}
}
