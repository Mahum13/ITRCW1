/**
 * pf0.pde (sklar/12-nov-2018)
 * potential field for mobile robot: basic code
 */

int MAX_X = 20;
int MAX_Y = 20;
int DELTA = 25; // = 500 / MAX_X
int MIN_FORCE = 1; // minimum force for potential field
int MAX_FORCE = ( MAX_X * MAX_Y ); // maximum force for potential field
int FONT_SIZE = 10;

int NUM_OBSTACLES = int(( MAX_X * MAX_Y ) * 0.25 ); // number of obstacles in robot's world
int CELL_EMPTY = 0; 
int CELL_OBSTACLE = -1;
int CELL_TARGET = MAX_FORCE;
int CELL_TARGET_FOUND = MAX_FORCE + 1;

PVector T = new PVector( 0, 0 ); // coordinates of target point
PVector R = new PVector( 0, 0 ); // coordinates of robot's current location
int sensors[] = new int[8];  // values returned by each of robot's range sensors
int SENSORS_X_DIR[] = { -1, 0, +1, -1, +1, -1, 0, +1 };
int SENSORS_Y_DIR[] = { -1, -1, -1, 0, 0, +1, +1, +1 };
int world[][] = new int[MAX_Y][MAX_X]; // the robot's world, which will be represented as a potential field
boolean searching;


/**
 * setup()
 */
void setup() {
  // set size of robot's world in pixels, which translates to MAX_X x MAX_Y in cells
  // 500 x 500 => (MAX_X * DELTA) x (MAX_Y * DELTA)
  size( 500, 500 );
  // set background for drawing robot's world
  background( #ffffff );
  // set font size for labelling cells
  textSize( FONT_SIZE );
  // initialise locations and robot's world
  reset();
  sense();
  // initialise flag indicating that robot is still searching
  searching = true;
} // end of setup()


/**
 * pickRandomCell()
 * This function picks a random cell within the robot's world and returns it.
 */
PVector pickRandomCell() {
  PVector C = new PVector();
  C.x = int( random( width/DELTA ));
  C.y = int( random( height/DELTA ));
  return( C );
} // end of pickRandomCell()


/**
 * initWorld()
 * This function initialises the robot's world.
 */
void initWorld() {
  // initialise the world to all empty cells
  for ( int y=0; y<MAX_Y; y++ ) {
    for ( int x=0; x<MAX_X; x++ ) {
      world[y][x] = CELL_EMPTY;
    }
  }
  // set cell value for target location
  world[int(T.y)][int(T.x)] = CELL_TARGET;
  // put obstacles in cells
  int num = 0;
  PVector C;
  while ( num < NUM_OBSTACLES ) {
    C = pickRandomCell();
    if ( world[int(C.y)][int(C.x)] == CELL_EMPTY ) {
      world[int(C.y)][int(C.x)] = CELL_OBSTACLE;
      num++;
    }
  }
} // end of initWorld()


/**
 * reset()
 * This function (re-)sets the starting and ending (target) locations for a new round.
 */
void reset() {
  // define target location
  T = pickRandomCell();
  // define starting location for robot, making sure it's not the same as target location
  R = pickRandomCell();
  while (( R.x == T.x ) && ( R.y == T.y )) {
    R = pickRandomCell();
  } // end while
  // re-initialise the robot's world
  initWorld();
  // activate the robot's sonar sensors
  sense();
  // reset "searching" flag
  searching = true;
} // end of reset()


/**
 * collide()
 * This function returns "true" if the proposed new location for the robot, 
 * indicated by the argument (try_x,try_y), will result in a collision 
 * (with an obstacle or the "edge" of the robot's world).
 */
boolean collide( int try_x, int try_y ) {
  if ( validCell( try_x, try_y ) && ( world[try_y][try_x] != CELL_OBSTACLE )) {
    return( false ); // no collision
  } else {
    return( true ); // collision is imminent
  }
} // end of collide()


/**
 * keyPressed()
 * This function is called when any key is pressed.
 */
void keyPressed() {
  int c, try_x = int(R.x), try_y = int(R.y);
  if (( key == 'q' ) || ( key == 'Q' )) {
    exit();
  } else if ( key == ' ' ) { // space
    reset();
  } else if ( searching ) {
    if (( key == CODED ) && ( keyCode == UP )) {
      try_y = int(R.y) - 1;
    } else if (( key == CODED ) && ( keyCode == DOWN )) {
      try_y = int(R.y) + 1;
    } else if (( key == CODED ) && ( keyCode == LEFT )) {
      try_x = int(R.x) - 1;
    } else if (( key == CODED ) && ( keyCode == RIGHT )) {
      try_x = int(R.x) + 1;
    }
    if ( collide( try_x, try_y )) {
      println( "oops! an obstacle!" );
    } else {
      R.x = try_x;
      R.y = try_y;
      if ( world[int(R.y)][int(R.x)] == CELL_TARGET ) {
        println( "target reached!" );
        world[int(R.y)][int(R.x)] = CELL_TARGET_FOUND;
        searching = false;
      } else {
        sense();
        println( "moved robot!" );
      }
    }
  }
} // end of keyPressed()


/**
 * validCell()
 * This function returns true if the argument coordinates are valid within the robot's grid world.
 */
boolean validCell( int x, int y ) {
  if (( x >= 0 ) && ( x < MAX_X ) && ( y >= 0 ) && ( y < MAX_Y )) {
    return( true );
  }
  else {
    return( false );
  }
} // end of validCell()


/**
 * sense()
 * Each sensor is set equal to the distance (in cells) to the closest obstacle, 
 * within one cell (the sensor range), for each of the eight 8 directions.
 * 0 1 2
 * 3 R 4
 * 5 6 7
 */
void sense() {
  int cell_x, cell_y;
  for ( int i=0; i<8; i++ ) {
    cell_x = int(R.x) + SENSORS_X_DIR[i];
    cell_y = int(R.y) + SENSORS_Y_DIR[i];
    if ( validCell( cell_x, cell_y )) {
      sensors[i] = rangeOfFive(cell_x, cell_y, i);
    }
  }
} // end of sense()

int rangeOfFive(int cell_x, int cell_y, int i) {
  int counter = 1;
  for (int j = 1; j < 6; j++) {
    if ( validCell( cell_x, cell_y )) {
      if (world[cell_y][cell_x] == CELL_OBSTACLE) {
        return counter;
      } else {
        cell_x = cell_x + SENSORS_X_DIR[i];
        cell_y = cell_y + SENSORS_Y_DIR[i];
        counter++;
      }
    }
  }
  return counter;

}
/**
 * drawCell()
 */
void drawCell( int x, int y, int colour ) {
  fill( colour );
  rect( x*DELTA, y*DELTA, DELTA, DELTA );
} // end of drawCell()


/**
 * drawSensors()
 */
void drawSensors() {
  String label;
  float labelWidth, labelHeight;
  int cell_x, cell_y;
  for ( int i=0; i<8; i++ ) {
    cell_x = int(R.x) + SENSORS_X_DIR[i];
    cell_y = int(R.y) + SENSORS_Y_DIR[i];
    if ( validCell( cell_x, cell_y )) {
      if ( world[cell_y][cell_x] == CELL_TARGET ) {
        drawCell( cell_x, cell_y, #990000 ); // target location
      } else if ( world[cell_y][cell_x] == CELL_TARGET_FOUND ) {
        drawCell( cell_x, cell_y, #ff00ff ); // target location (after target is found)
      } else if ( world[cell_y][cell_x] == CELL_OBSTACLE ) {
        drawCell( cell_x, cell_y, #666666 ); // obstacle
      }
      else {
        drawCell( cell_x, cell_y, #cccccc ); // sensor
      }
      fill( #000099 );
      label = str( sensors[i] );
      labelWidth = textWidth( label );
      labelHeight = FONT_SIZE/2;
      text( label, cell_x*DELTA+(DELTA-labelWidth)/2, cell_y*DELTA+(DELTA+labelHeight)/2 );
    }
  }
} // end of drawSensors()


/**
 * draw()
 */
void draw() {
  String label;
  float labelWidth, labelHeight;
  background( #ffffff );
  // draw grid world
  stroke( #999999 );  
  for ( int i=0; i<width/DELTA; i++ ) {
    line( i*DELTA, 0, i*DELTA, height );
  }
  for ( int i=0; i<height/DELTA; i++ ) {
    line( 0, i*DELTA, width, i*DELTA );
  }
  // draw cells in world
  for ( int y=0; y<MAX_Y; y++ ) {
    for ( int x=0; x<MAX_X; x++ ) {
      if ( world[y][x] == CELL_TARGET ) {
        drawCell( x, y, #990000 ); // target location
      } else if ( world[y][x] == CELL_TARGET_FOUND ) {
        drawCell( x, y, #ff00ff ); // target location (after target is found)
      } else if ( world[y][x] == CELL_OBSTACLE ) {
        drawCell( x, y, #666666 ); // obstacle
      }
      fill( #000000 );
      label = str( world[y][x] );
      labelWidth = textWidth( label );
      labelHeight = FONT_SIZE/2;
      text( label, x*DELTA+(DELTA-labelWidth)/2, y*DELTA+(DELTA+labelHeight)/2 );
    }
  }
  if ( searching ) {
    // draw robot's position and sensor range
    drawCell( int(R.x), int(R.y), #000099 ); // robot's current position
    drawSensors();
  }
} // end of draw()
