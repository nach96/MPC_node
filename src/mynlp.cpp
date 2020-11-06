#include "mynlp.h"

#include <vector>

using CppAD::AD;

//Global variable
size_t N = 10; //Horizon
AD<double> dt = 0.1;

//X vector managing variables
size_t x_start = 0;
size_t y_start = x_start + N;
size_t tita_start = y_start + N;
size_t V_start = tita_start + N;
size_t W_start = V_start + N-1; //-1 as last actions are not applied

   
FG_eval::FG_eval(){};

FG_eval::FG_eval(double _xp, double _yp, double _ap, double _dist, double _ang, double _yaw, double _K1, double _K2, double _K3, double _K4, double _K5, double _preV, double _preW){
  xp = _xp;
  yp = _yp;
  ap = _ap;
  dist = _dist;
  ang = _ang;
  yaw = _yaw;
  K1 = _K1;
  K2 = _K2;
  K3 = _K3;
  K4 = _K4;
  K5 = _K5;
  preV = _preV;
  preW = _preW;

}

void FG_eval::operator()(ADvector& fg, const ADvector& x)
{


    assert( fg.size() == 3*(N-1) + 1 );
    assert( x.size()  == 3*N + 2*(N-1) );


     // f(x)
     fg[0]=0;

     for(int i=0; i < N; i++){
           fg[0] += K1*CppAD::pow( CppAD::pow( CppAD::pow(xp-x[x_start+i],2) + CppAD::pow(yp-x[y_start + i],2), 0.5) - dist , 2);
           fg[0] += K2*CppAD::pow((ap - x[tita_start+i]) - ang,2);
           fg[0] += K3*CppAD::pow(CppAD::atan2(yp-x[y_start+i], xp-x[x_start+i])- yaw -x[tita_start+i],2);
      }

      //Limit first acceleration, relative to previous speed 
      fg[0] += K4*CppAD::pow( x[V_start]-preV,2 ) + K5*CppAD::pow(x[W_start]-preW,2);  
      //Limit acceleration
      for(int i=0;i<N-2; i++){
        fg[0] += K4*CppAD::pow( (x[V_start+i+1]-x[V_start+i]),2);
        fg[0] += K5*CppAD::pow( (x[W_start+i+1]-x[W_start+i]),2);

      }
      
      //g(x)  Constraint equations
    for(int i=1,j=1; i < N; i++,j+=3){
        AD<double> x0 = x[x_start + i -1];
        AD<double> y0 = x[y_start + i -1];
        AD<double> tita0 = x[tita_start + i -1];
        AD<double> V0 = x[V_start + i -1];
        AD<double> W0 = x[W_start + i -1];

        AD<double> x1 = x[x_start + i];
        AD<double> y1 = x[y_start + i];
        AD<double> tita1 = x[tita_start + i];

        fg[j] = x1 - x0 - dt*CppAD::cos(tita0)*V0;
        fg[j+1] = y1 - y0 - dt*CppAD::sin(tita0)*V0;
        fg[j+2] = tita1 - tita0 - dt*W0;


    }

     return;
}


myNLP::myNLP(double _vmax, double _wmax){
  vmax = _vmax;
  wmax = _wmax;
}  

void myNLP::my_solve(double xp, double yp, double ap, double dist, double ang, double yaw, double K1, double K2, double K3, double K4, double K5, double preV, double preW){

    // number of independent variables (domain dimension for f and g)
    size_t nx;
    nx = 3*N + 2*(N-1);


    //size_t nx = 3*N + 2*(N-1);
    // number of constraints (range dimension for g)
    size_t ng = 3*(N-1);

    Dvector x_i(nx);

    /*
    //Initial values: Straight line from robot to pre-computed "goal"
    AD<double> x_goal_ad = xp + dist*cos(yaw);
    AD<double> y_goal_ad = yp + dist*sin(yaw);
    double x_goal = CppAD::Value(x_goal_ad);
    double y_goal = CppAD::Value(y_goal_ad);

    for(int i = 0; i<nx; i+=5){
        x_i[i]= x_goal/(nx-i);
        x_i[i+1] = y_goal/(nx-i);
        x_i[i+2] = CppAD::atan2(y_goal,x_goal);
        x_i[i+3]=0;
        x_i[i+4]=0;
    }
    */
    for(int i=0; i<nx; i++){
        x_i[i]=0.0;
    }

    //Lower and upper limits for x
    Dvector x_l(nx), x_u(nx);
    // lower and upper limits for g
    Dvector g_l(ng), g_u(ng);


    x_l[x_start]=x_u[x_start]=0.0; //Xr0
    x_l[y_start]=x_u[y_start]=0.0; //Yr0
    x_l[tita_start]=x_u[tita_start]=0.0; //Titar0


    for(int i= 1; i<N; i++){
        x_l[x_start + i] = -1.0e19;//Xr
        x_u[x_start + i] = +1.0e19;

        x_l[y_start + i] = -1.0e19;//Yr
        x_u[y_start + i] = +1.0e19;

        x_l[tita_start + i] = -6.283;//tita_r
        x_u[tita_start + i] = +6.283;
    }
    for(int i=0; i<N-1; i++){
        x_l[V_start + i] = -vmax;//V_r
        x_u[V_start + i] = +vmax;

        x_l[W_start + i] = -wmax;//w_r
        x_u[W_start + i] = +wmax;
    }



    // Equality constraints. (Dynamic equations) We set the bounds on this constraint
    // to be equal (and zero).
    for (int i=0; i<ng; i++){
        g_l[i] = g_u[i] = 0.0;

    }

    // object that computes objective and constraints
    FG_eval fg_eval(xp,yp,ap, dist, ang, yaw, K1, K2, K3, K4, K5, preV, preW);

    //std::string options = set_options();
    // options
    std::string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    //options += "String  sb           yes\n";
    // maximum number of iterations
    options += "Integer max_iter     500\n";
    options += "Numeric tol          1e-8\n";
    //options += "String  derivative_test            second-order\n";
    //options += "Numeric derivative_test_tol          0.01\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";


    assert(x_l.size()==nx);
    assert(g_l.size()==ng);
    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
                options, x_i, x_l, x_u, g_l, g_u, fg_eval, solution
                );

    save_solution(solution);

}


void myNLP::save_solution(CppAD::ipopt::solve_result<Dvector> solution){

    std::vector<double> Xr2(&solution.x[x_start], &solution.x[y_start]);


    std::vector<double> Yr2(&solution.x[y_start], &solution.x[tita_start]);
    std::vector<double> Titar2(&solution.x[tita_start], &solution.x[V_start]);
    std::vector<double> Vr2(&solution.x[V_start], &solution.x[W_start]);
    std::vector<double> Wr2(&solution.x[W_start], &solution.x[W_start+N-2]);

    Xr = Xr2;
    Yr=Yr2;
    Titar = Titar2;
    Vr=Vr2;
    Wr=Wr2;

/*

      //Print a matrix with the results to plot it in matlab
      std::ofstream out("X.txt");

      //printf("Xr:");
      //Xr; Yr; Titar; Vr; Wr
      out << "X = [";
      for(auto it = Xr.begin() ; it != Xr.end(); it++){
         //std::cout << ' ' << *it;
         //std::cout << '\n';
         out << *it << " ";
      }

      //printf("Titar:");
      out << "; \n";
      for(auto it = Yr.begin() ; it != Yr.end(); it++){
         out << *it << " ";

      }
      //printf("Titar:");
       out << "; \n";
      for(auto it = Titar.begin() ; it != Titar.end(); it++){
         out << *it << " ";

      }
      //printf("Vr:");
       out << "; \n";
      for(auto it = Vr.begin() ; it != Vr.end(); it++){
         out << *it << " ";

      }
      //printf("Wr:");
       out << 0 << "; ";
      for(auto it = Wr.begin() ; it != Wr.end(); it++){
         out << *it << " ";

      }
      out << 0<< " "<< 0 <<"]";

*/


}

