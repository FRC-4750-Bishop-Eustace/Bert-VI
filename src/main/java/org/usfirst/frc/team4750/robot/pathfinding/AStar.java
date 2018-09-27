package org.usfirst.frc.team4750.robot.pathfinding;

import java.awt.Point;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Command;

public class AStar {
	
	public AStar() {
		
	}
	
	public ArrayList<Command> getPath(Point startPoint, Point targetPoint)
			throws InvalidLetterException {

		InputHandler handler = new InputHandler();
		SquareGraph graph = handler.readMap(startPoint, targetPoint);
		Map map = new Map();		
		
		ArrayList<Node> path = graph.executeAStar();
		path = graph.reduceTurns(path);
		ArrayList<Command> drivePath = graph.makeDrivePath(path);

		char[][] mapArray = map.getMap();
		
		if (path == null) {
			System.out.println("There is no path to target");
		} else {
			System.out.println("--- Path to target ---");
			graph.printPath(path);
			System.out.println("");
			System.out.println("--- Translated path ---");
			graph.translatePath(path);
			System.out.println("");
			System.out.println("");
			System.out.println("--- Map ---");
			graph.printMap(mapArray, path);
		}
		return drivePath;
	}

}
