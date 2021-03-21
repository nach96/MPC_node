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

    int nfg = fg.size();
    //nfg += 2*(N-1); //add acceleration constraints.
    //assert( fg.size() == 3*(N-1) + 1 );
    size_t ng = 3*(N-1) + 1;
    assert(nfg == ng + 2*(N-1));
    assert( x.size()  == 3*N + 2*(N-1) );


     //-------------------------------- f(x) Objective Function --------------------------------------------------------------
     fg[0]=0;

     for(int i=0; i < N; i++){
           fg[0] += K1*CppAD::pow( CppAD::pow( CppAD::pow(xp-x[x_start+i],2) + CppAD::pow(yp-x[y_start + i],2), 0.5) - dist , 2);
           fg[0] += K2*CppAD::pow((ap - x[tita_start+i]) - ang,2);
           fg[0] += K3*CppAD::pow(CppAD::atan2(yp-x[y_start+i], xp-x[x_start+i])- yaw -x[tita_start+i],2);
      }

      
    //--------------------- Additional terms of f(x) to tune behaviour ---------------------------------------------------
      

      /* It doesn work at all
      //Give aditional importance to the end-point. Relevant for convergence at the end of the trajectory.
      fg[0] += K4*0*CppAD::pow( CppAD::pow( CppAD::pow(xp-x[x_start+N-1],2) + CppAD::pow(yp-x[y_start + N-1],2), 0.5) - dist , 2);
      fg[0] += K4*1*CppAD::pow((ap - x[tita_start+N-1]) - ang,2);
      fg[0] += K4*0.1*CppAD::pow(CppAD::atan2(yp-x[y_start+N-1], xp-x[x_start+N-1])- yaw -x[tita_start+N-1],2);
      */

       //Now it is limited with constraint eq.
      //Penalize first acceleration, relative to previous speed 
      fg[0] += K4*CppAD::pow( x[V_start]-preV,2 ) + K5*CppAD::pow(x[W_start]-preW,2);  
      //Limit acceleration
      for(int i=0;i<N-2; i++){
        fg[0] += K4*CppAD::pow( (x[V_start+i+1]-x[V_start+i]),2);
        fg[0] += K5*CppAD::pow( (x[W_start+i+1]-x[W_start+i]),2);

      }
      
      
      /*
      //Penalize actions    
      for(int i=0; i<N-1; i++){
           //Limit actions
           fg[0] += K4*CppAD::pow(x[V_start+i],2);
           fg[0] += K5*CppAD::pow(x[W_start+i],2);
      }
      */
      
      
      
      // --------------------------------------g(x)  Constraint equations---------------------------------------------------------
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
    //Add constraints to acceleration

    fg[ng] = x[V_start]-preV;
    fg[ng+1] = x[W_start]-preW;

    for(int i=1,j=ng+2; i< (N-1); i++, j+=2){
      fg[j] = x[V_start+i]-x[V_start+i-1];
      fg[j+1] = x[W_start+i]-x[W_start+i-1];
    }

     return;
}


myNLP::myNLP(double _vmax, double _wmax){
  vmax = _vmax;
  wmax = _wmax;
}  

void myNLP::my_solve(double xp, double yp, double ap, double dist, double ang, double yaw, double K1, double K2, double K3, double K4, double K5){

    // number of independent variables (domain dimension for f and g)
    size_t nx;
    nx = 3*N + 2*(N-1);


    //size_t nx = 3*N + 2*(N-1);
    // number of constraints (range dimension for g)
    size_t ng1 = 3*(N-1);
    size_t ng = ng1 + 2*(N-1); //add acceleration constraints.

    Dvector x_i(nx);

    

    if(solution.x.size()>0){
      preV = solution.x[V_start];
      preW = solution.x[W_start];

      //Give previous solution as start. In order to be a warm-start I should also give initial lambda values.
      //Shift it one step. For the last value, repete the previous last
      //This doesn't reduce at all the number of iterations needed.
      
      /* //Now this is done in cppad/solve_callback.hpp (Modified file)
      for(int i=0;i<N-1;i++){
        x_i[x_start+i]=solution.x[x_start+i+1];
        x_i[y_start+i]=solution.x[y_start+i+1];
        x_i[tita_start+i]=solution.x[tita_start+i+1];
      }
      for(int i=0; i<N-3;i++){
        x_i[V_start+i]=solution.x[V_start+i+1];
        x_i[W_start+i]=solution.x[W_start+i+1];
      }
      x_i[x_start+N-1]=solution.x[x_start+N-1];
      x_i[y_start+N-1]=solution.x[y_start+N-1];
      x_i[tita_start+N-1]=solution.x[tita_start+N-1];
      x_i[V_start+N-2]=solution.x[V_start+N-2];
      x_i[W_start+N-2]=solution.x[W_start+N-2];
      */
            
      for(int i=0; i<nx; i++){
        x_i[i]=0.0;
      }
      

      
    }else{
      preV=0;
      preW=0;
      for(int i=0; i<nx; i++){
        x_i[i]=0.0;
      }
      
    
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
    for (int i=0; i<ng1; i++){
        g_l[i] = g_u[i] = 0.0;

    }
    for(int i=ng1; i<ng; i+=2){
      g_l[i] = -av;
      g_l[i+1] = -aw;
      g_u[i] = av;
      g_u[i+1] = aw;
    }


    // object that computes objective and constraints
    FG_eval fg_eval(xp,yp,ap, dist, ang, yaw, K1, K2, K3, K4, K5, preV, preW);

    //std::string options = set_options();
    // options
    std::string options;
    // turn off any printing
    options += "Integer print_level  5\n";
    options += "String  sb           yes\n";
    // maximum number of iterations
    options += "Integer max_iter     500\n";
    options += "Numeric tol          1e-8\n";
    //options += "String  derivative_test            second-order\n";
    //options += "Numeric derivative_test_tol          0.01\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";

    
    //File urs/include/cppad/ipopt/solve_callback.hpp modified to warm-start also if( solution.x.size()>0 and init_z == true).
    //So if this options are set, warm-start will automatically begin from second step.
    //But must say it doesn't improves as much as it should

    if(solution.x.size()>0){
      options += "String nlp_scaling_method none\n"; 
      options += "String warm_start_init_point yes\n";

      options += "Numeric warm_start_bound_frac 1e-16\n";
      options += "Numeric warm_start_bound_frac 1e-16\n";
      options += "Numeric warm_start_bound_push 1e-16\n";
      options += "Numeric warm_start_mult_bound_push 1e-16\n";
      options += "Numeric warm_start_slack_bound_frac 1e-16\n";
      options += "Numeric warm_start_slack_bound_push 1e-16\n";     
    }


    assert(x_l.size()==nx);
    assert(g_l.size()==ng);
    

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

