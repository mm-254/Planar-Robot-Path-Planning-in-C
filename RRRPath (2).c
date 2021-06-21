//Robot path planning for constant velocity straight line trajectory
//Mandira Marambe 06/19/2021

/* 
This programs takes initial and final end effector configurations, total time, and number of desired steps to compute the robot trajectory at constant velocity along a straight line
*/

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

/*-------------------------VARIABLES-------------------------*/

//Parameters to be input
double initial[3]; //initial configuration (x,y,phi)
double final[3]; //final configuration (x,y,phi)
double t = 2; //time in seconds desired
double N = 10; //No. of steps desired
double L1 = 5; double L2=4; double L3=3;

//Initializing vectors and variables 
double v[3];  //speed vector in x, y, phi
double dp[3]; //displacement vector per time step
double dphi;  //angular displacement of the end effector
bool singularity;

/*-------------------------SUPPORTING FUNCTIONS-------------------------*/

// Direct inverse of a 3x3 matrix m
void mat3x3_inv(double minv[3][3], const double m[3][3])
{
   double det;
   double invdet;

   det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
         m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
         m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

   
   invdet = 1.0 / det;

   minv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet;
   minv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
   minv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
   minv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
   minv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
   minv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet;
   minv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet;
   minv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet;
   minv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet;
}


/* Explicit 3x3 matrix * vector.  b = A x  where A is 3x3 */
void mat3x3_vec_mult(double b[3], const double A[3][3], const double x[3])
{
   const int N = 3;
   int      idx, jdx;

   for( idx = 0; idx < N; idx++ )
   {
      b[idx] = 0.0;

      for( jdx = 0; jdx < N; jdx++)
      {
         b[idx] += A[idx][jdx] * x[jdx] ;
      }
   }
}


/*-------------------------MAIN LOOP-------------------------*/
int main() {
    //------------------Inputs and Preliminary Computations------------
    
    //User inputs
    printf("Enter initial point coordinates\n");
    for(int i = 0;i < 3;i++) scanf("%lf",&initial[i]);
    
    printf("Enter final point coordinates\n");
    for(int i = 0;i < 3;i++) scanf("%lf",&final[i]);
    
    printf("Enter total time\n");
    scanf("%lf",&t);
    
    printf("Enter number of points in trajectory\n"); 
    scanf("%lf",&N);
    
    //Calculate length of a single time step
    double time_step = t/N;
    
    //Compute displacement and velocity vectors
    //dphi = (final[2] - initial[2])/N;  //Angular displacement
    
    for(int i = 0;i < 3;i++){
        v[i]=(final[i]-initial[i])/t;  //x, y, phi velocity (const)
        dp[i]=(final[i]-initial[i])/N; //x, y, phi displacement (const)
    } 
    
    //Print inputs and preliminary computations
    printf ("Lengths: %2f %2f %2f\n", L1,L2,L3);
    printf ("Initial: %2f %2f %2f\n", initial[0],initial[1],initial[2]);
    printf ("Final: %2f %2f %2f\n", final[0],final[1],final[2]);
    printf ("Time step: %2f\n", time_step);
    printf ("Velocity: %2fi + %2fj\n", v[0],v[1]);
    printf ("Displacement per time step (i,j,k): %2fi + %2fj\n", dp[0],dp[1]);
    printf ("Rotation per time step: %2f\n", dphi);
    printf ("\n");
    
    
    //------------------------Path generator Loop--------------------------
    
    //Initializing vectors and variables to be computed in iterative loop
    double x=initial[0];  //x position at step k
    double y = initial[1]; //y position at step k
    double phi=initial[2];  // phi (rotation) at step k
    
    double theta[3]; //join angles at step k
    double theta_dot[3]; //joint rates at step k
    double J[3][3]={{0,0,0},{0,0,0},{1,1,1}}; //Jacobian at step k
    double Jinv[3][3]; //Inverse of Jacobian at step k
    
    for(int k = 0;k < N;k++){ //Iterate through each step in N steps
    
        //Update to new position  (Note: since the required output is N rows, the computations for the starting point are not included)
        x = x+dp[0]; y = y+dp[1];
        phi = phi+dp[2];
        
        //----------------Inverse Kinematics to find theta----------------
        //Theta 1 (See document for derivation)
        double xb=x-L3*cos(phi); double yb=y-L3*sin(phi);
        
        theta[0]= atan2(-yb/(sqrt(xb*xb + yb*yb)), -xb/(sqrt(xb*xb + yb*yb)) )+acos(-(xb*xb + yb*yb + L1*L1 - L2*L2)/(2*L1*sqrt(xb*xb + yb*yb)));
        //Note: there are two values that can be generated for theta1 with atan(Q)+/-acos(R), so two possible path configurations. Here we take the +acos path.
        
        //Theta 2
        theta[1]=atan2((yb-(L1*sin(theta[0])))/L2, (xb-(L1*cos(theta[0])))/L2)-theta[0];
        
        //Theta 3
        theta[2]=phi-theta[1]-theta[0];
        
        //----------------Joint Velocity Computations----------------
        //Jacobian computation (derived in part (1))
        J[0][0]=-(L1*sin(theta[0]))-(L2*sin(theta[0]+theta[1]))-(L3*sin(theta[0]+theta[1]+theta[2]));
        J[0][1]=-(L2*sin(theta[0]+theta[1]))-(L3*sin(theta[0]+theta[1]+theta[2]));       
        J[0][2]=-(L3*sin(theta[0]+theta[1]+theta[2]));
        J[1][0]=(L1*cos(theta[0]))+(L2*cos(theta[0]+theta[1]))+(L3*cos(theta[0]+theta[1]+theta[2]));    
        J[1][1]=(L2*cos(theta[0]+theta[1]))+(L3*cos(theta[0]+theta[1]+theta[2]));      
        J[1][2]=(L3*cos(theta[0]+theta[1]+theta[2]));   
        
        //Inverse of Jacobian
        mat3x3_inv(Jinv, J);
        
        //Joint rates (theta_dot = Jinv*v)
        mat3x3_vec_mult(theta_dot, Jinv, v);
        
        
        //----------------Print Outputs----------------
        
        printf ("Position: %2f (x) %2f (y) %2f (phi) \n", x,y,phi);
        
        //Print if point is unreachable (Joint rates could not be computed)
        if(isnan(theta_dot[0])|| isnan(theta_dot[1]) || isnan(theta_dot[2])){
            printf("Pose Unreachable: ");
        }
        
        printf ("Theta1: %2f (rad) Theta2: %2f (rad) Theta3: %2f (rad)", theta[0],theta[1],theta[2]);
        printf ("Theta1_dot: %2f (rad/s) Theta1_dot: %2f (rad/s) Theta3_dot: %2f (rad/s) \n", theta_dot[0],theta_dot[1],theta_dot[2]);
        //printf("J: %2f %2f %2f\n",J[0][0],J[0][1],J[0][2]);        
        //printf("Jinv: %2f %2f %2f\n",Jinv[0][0],Jinv[0][1],Jinv[0][2]);
        
         
    } 
    
    return 0;
}