package org.usfirst.frc.team4750.robot.pathfinding;

public class Map {

	public char[][] map =
		{
			{'X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X'},
			{'X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X'},
			{'X','X','X','X',' ','X','X','X','X','X','X','X','X','X',' ','X','X','X','X','X','X','X','X','X',' ','X','X','X','X'},
			{'X','X','X',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ',' ',' ',' ',' ','X','X','X','X','X','X','X','X','X',' ',' ',' ',' ',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ',' ',' ',' ',' ','X','X','X','X','X','X','X','X','X',' ',' ',' ',' ',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ',' ',' ',' ',' ','X','X','X','X','X','X','X','X','X',' ',' ',' ',' ',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ',' ',' ',' ',' ','X','X','X','X','X','X','X','X','X',' ',' ',' ',' ',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ','X','X',' ','X','X','X','X','X','X','X','X','X','X','X',' ','X','X',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ','X','X',' ','X','X','X','X','X','X','X','X','X','X','X',' ','X','X',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ',' ',' ','X','X','X','X','X','X','X','X','X','X','X','X','X',' ',' ',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ','X','X',' ','X','X','X','X','X','X','X','X','X','X','X',' ','X','X',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X',' ',' ',' ','X','X','X'},
			{'X','X','X',' ',' ',' ','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X',' ',' ',' ','X','X','X'},
			{'X','X','X',' ','X',' ','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X',' ','X',' ','X','X','X'},
			{'X','X','X',' ','X',' ','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X',' ','X',' ','X','X','X'},
			{'X','X','X',' ','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X',' ','X','X','X'},
			{'X','X','X',' ',' ',' ','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X',' ',' ',' ','X','X','X'},
			{'X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X','X'},
	};
	
	public char[][] getMap() {
		return map;
	}
}
