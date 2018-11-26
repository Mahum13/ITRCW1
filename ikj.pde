/** 
  * ikj.pde
  * This program animated a 3-segment arm over a series of possible positions
  * This code provides a Jacobian solution to Inverse Kinematics
**/

//define offset values so that (0,0) is in the middle of the display window
float X_OFFSET, Y_OFFSET;

//define global variabls (length of each arm link)
float L1 = 100;
float L2 = 100;
float L3 = 100;

//time counter
int TC = 0;

//define arrays of data over each 200 arm positions
float theta_total[] = new float[200];                 // total angle
float px[]  = new float[200], py[]  = new float[200]; // position of end effector, benchmark data
float l1x[] = new float[200], l1y[] = new float[200]; // computed position of end of link 1 (P1)
float l2x[] = new float[200], l2y[] = new float[200]; // computed position of end of link 2 (P2)
float l3x[] = new float[200], l3y[] = new float[200]; // computed position of end of link 3 (P3)
float pxj[] = new float[200], pyj[] = new float[200]; // computed position of end effector, jacobian approach
float l0x = 0, l0y = 0; // position of base of arm (P0)
float theta1[] = new float[200]; // computed angle of link 1
float theta2[] = new float[200]; // computed angle of link 2
float theta3[] = new float[200]; // computed angle of link 3


/**
 * setup()
 * This function is called once, when the program starts up. 
 */
void setup() {
  // set up drawing window
  size( 500, 500 );
  // save centre of drawing window, in pixels
  X_OFFSET = 250;
  Y_OFFSET = 250;
  // set up mode for drawing circles
  ellipseMode( CENTER );
  // slow down drawing rate so we can see what is happening
  frameRate( 10 );
  // initialise time counter for looping through arm positions
  TC = 0;
  // generate benchmark data (200 arm positions)
  generateBenchmarkData();
  // generate data using jacobian approach to inverse kinematics (for 200 arm positions)
  generateJacobianData();
  // save the jacboian data in a file (will be saved in the sketch folder)
  PrintWriter outfile = createWriter( "ikj-data.txt" );
  for ( int t=0; t<200; t++ ) {
    outfile.println( t + ", " + l1x[t] + ", " + l1y[t] + ", " + l2x[t] + ", " + l2y[t] + ", " + l3x[t] + ", " + l3y[t] );
  }
  outfile.close();
} // end of setup()

void generateBenchmarkData() {
  float i = 0.0;
  for ( int t=0; t<200; t++ ) {
    // set angles                                                                                                   
    theta1[t] = i;
    theta2[t] = i;
    theta3[t] = PI - ( theta1[t] + theta2[t] ); // because theta1+theta2+theta3=pi                                                                                      
    theta_total[t] = ( theta1[t] + theta2[t] + theta3[t] );
    // set end point                                                                                                
    px[t] = ( L1*cos(theta1[t]) + L2*cos(theta1[t]+theta2[t]) + L3*cos(theta1[t]+theta2[t]+theta3[t]) );
    py[t] = ( L1*sin(theta1[t]) + L2*sin(theta1[t]+theta2[t]) + L3*sin(theta1[t]+theta2[t]+theta3[t]) );
    i += 0.0157;
  }
} // end of generateBenchmarkData()

/**
  * IK_jacobian()
  * This function computes and returns three joint angles, given the endpoint (px, py),
  * lengths of 3 joints (L1, L2, L3) and the total angle (the_total).
  * this function is returning qdot, + change in angle 
  //pwdot = end effector angle currently at time  
  //pw = end effector angle at time t+1
  //
*/
float[] IK_jacobian( float px, float py, float pw, float w1, float w2, float w3, float pxdot, float pydot, float pwdot ) {
  float new_angles[] = { 0.0, 0.0, 0.0 };
  float ve[] = { px - pxdot, py - pydot, pw - pwdot }; // velocity of pe
  float J[][] = new float[3][3]; //Jacobian matrix
  
  J[0][0] = -L1*sin(w1) - L2*sin(w1+w1) - L3*sin(w1+w2+w3); //Pey
  J[0][1] = -L2*sin(w1+w2) - L3*sin(w1+w2+w3);
  J[0][2] = -L3*sin(w1+w2+w3);
  J[1][0] = L1*cos(w1) + L2*cos(w1+w2) + L3*cos(w1+w2+w3); //Pex
  J[1][1] = L2*cos(w1+w2) + L3*cos(w1+w2+w3); // 
  J[1][2] = L3*cos(w1+w2+w3);
  J[2][0] = 1;
  J[2][1] = 1;
  J[2][2] = 1;
  
  float JInv[][] = inv3( J ); // compute J inverse
  float qdot[] = multiply3x1( JInv, ve ); //angle changes
  
  new_angles[0] = w1 + qdot[0];
  new_angles[1] = w2 + qdot[1];
  new_angles[2] = w3 + qdot[2];
  return ( new_angles );
}

/** 
 * generateJacobianData()
 * This function generates data using the Jacobian solution to the inverse kinematics to 
 * compute the location of the links for display. 
*/
void generateJacobianData() {
  float angles[];
   for ( int t =0; t < 200; t++ ) {
    float pxdot = L1*cos(theta1[t]) + L2*cos(theta1[t]+theta2[t]) + L3*cos(theta1[t]+theta2[t]+theta3[t]);
    float pydot = L1*sin(theta1[t]) + L2*sin(theta1[t]+theta2[t]) + L3*sin(theta1[t]+theta2[t]+theta3[t]);
    angles = IK_jacobian(px[t], py[t], theta_total[t], theta1[t], theta2[t], theta3[t], pxdot, pydot,  (theta1[t]+theta2[t]+theta3[t]));
  
    // set end point
    pxj[t] = ( L1*cos(angles[0]) + L2*cos(angles[0]+angles[1]) + L3*cos(angles[0]+angles[1]+angles[2]) );
    pyj[t] = ( L1*sin(angles[0]) + L2*sin(angles[0]+angles[1]) + L3*sin(angles[0]+angles[1]+angles[2]) );
    // set first point                                                                                              
    l1x[t] = ( L1*cos(angles[0]) );
    l1y[t] = ( L1*sin(angles[0]) );
    // set second point                                                                                             
    l2x[t] = ( L1*cos(angles[0]) + L2*cos(angles[0]+angles[1]) );
    l2y[t] = ( L1*sin(angles[0]) + L2*sin(angles[0]+angles[1]) );
    // set third point                                                                                              
    l3x[t] = ( L1*cos(angles[0]) + L2*cos(angles[0]+angles[1]) + L3*cos(angles[0]+angles[1]+angles[2]) );
    l3y[t] = ( L1*sin(angles[0]) + L2*sin(angles[0]+angles[1]) + L3*sin(angles[0]+angles[1]+angles[2]) );
  
  }
}

// returns determinant of 2x2 matrix M
float det2( float M[][] ) {
  return( M[0][0] * M[1][1] - M[0][1] * M[1][0] );
} // end of det2()


// returns determinant of 3x3 matrix M
float det3( float M[][] ) {
  float tmpM1[][] = {{ M[1][1], M[1][2] }, { M[2][1], M[2][2] }};
  float tmpM2[][] = {{ M[1][0], M[1][2] }, { M[2][0], M[2][2] }};
  float tmpM3[][] = {{ M[1][0], M[1][1] }, { M[2][0], M[2][1] }};
  float d1 = det2( tmpM1 );
  float d2 = det2( tmpM2 );
  float d3 = det2( tmpM3 );
  return( M[0][0] * d1 - M[0][1] * d2 + M[0][2] * d3 );
} // end of det3()


// returns inverse of argument 3x3 matrix M                                                                                  
float[][] inv3( float M[][] ) {
  float d = det3( M );
  float IM[][] = new float[3][3];
  float M00[][] = {{ M[1][1], M[1][2] }, { M[2][1], M[2][2] }};
  float M01[][] = {{ M[0][2], M[0][1] }, { M[2][2], M[2][1] }};
  float M02[][] = {{ M[0][1], M[0][2] }, { M[1][1], M[1][2] }};
  float M10[][] = {{ M[1][2], M[1][0] }, { M[2][2], M[2][0] }};
  float M11[][] = {{ M[0][0], M[0][2] }, { M[2][0], M[2][2] }};
  float M12[][] = {{ M[0][2], M[0][0] }, { M[1][2], M[1][0] }};
  float M20[][] = {{ M[1][0], M[1][1] }, { M[2][0], M[2][1] }};
  float M21[][] = {{ M[0][1], M[0][0] }, { M[2][1], M[2][0] }};
  float M22[][] = {{ M[0][0], M[0][1] }, { M[1][0], M[1][1] }};
  IM[0][0] = ( 1 / d ) * det2( M00 );
  IM[0][1] = ( 1 / d ) * det2( M01 );
  IM[0][2] = ( 1 / d ) * det2( M02 );
  IM[1][0] = ( 1 / d ) * det2( M10 );
  IM[1][1] = ( 1 / d ) * det2( M11 );
  IM[1][2] = ( 1 / d ) * det2( M12 );
  IM[2][0] = ( 1 / d ) * det2( M20 );
  IM[2][1] = ( 1 / d ) * det2( M21 );
  IM[2][2] = ( 1 / d ) * det2( M22 );
  return( IM );
} // end of inv3()


// computes global variable Delta as M * V where argument M is a 3x3 matrix and argument V is a 3x1 matrix                                                     
float[] multiply3x1( float M[][], float V[] ) {
  float[] MM = new float[3];
  MM[0] = M[0][0] * V[0] + M[0][1] * V[1] + M[0][2] * V[2];
  MM[1] = M[1][0] * V[0] + M[1][1] * V[1] + M[1][2] * V[2];
  MM[2] = M[2][0] * V[0] + M[2][1] * V[1] + M[2][2] * V[2];
  return( MM );
} // end of multiply3x1()






/**
 * draw()
 * This function is called every time the Processing drawing loop iterates.
 */
void draw() {
  background( #ffffff );
  // draw original endpoint in a black circle
  strokeWeight( 1 );
  stroke( #000000 );
  ellipse( px[TC]+X_OFFSET, Y_OFFSET-py[TC], 5, 5 );
  // draw analytical endpoint in red
  strokeWeight( 3 );
  stroke( #ff0000 );
  point( pxj[TC]+X_OFFSET, Y_OFFSET-pyj[TC] );
  // draw arm
  strokeWeight( 2 );
  stroke( #0000ff );
  line( l0x+X_OFFSET, Y_OFFSET-l0y, l1x[TC]+X_OFFSET, Y_OFFSET-l1y[TC] );
  stroke( #ff00ff );
  line( l1x[TC]+X_OFFSET, Y_OFFSET-l1y[TC], l2x[TC]+X_OFFSET, Y_OFFSET-l2y[TC] );
  stroke( #00ff00 );
  line( l2x[TC]+X_OFFSET, Y_OFFSET-l2y[TC], l3x[TC]+X_OFFSET, Y_OFFSET-l3y[TC] );
  // increment time counter (TC)
  if ( TC < 199 ) {
    TC += 1;
  }
  // for debugging, print the data we are plotting in this drawing iteration
  print( "TC=", TC );
  print( " theta1=", degrees( theta1[TC] ));
  print( " theta2=", degrees( theta2[TC] ));
  print( " theta3=", degrees( theta3[TC] ));
  print( " total=", degrees( theta_total[TC] ));
  println();
} // end of draw()
