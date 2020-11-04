#include "mynlp.h"

#include <vector>

using CppAD::AD;

//Global variable
size_t N = 40; //Horizon
AD<double> dt = 0.1;
//AD<double> xp = 4.0E3; //[mm]
//AD<double> yp = 0.0;
//AD<double> ap = 0.0;
//double xp = 4.0e3;
//double yp = 0.0;
//double ap = 0.0;
AD<double> K1 = 1.0;
AD<double> K2 = 1.0E6;
AD<double> K3 = 1.0E5;
AD<double> dist = 3.0E3; //[mm]
AD<double> ang = 0.0; //[rad]
AD<double> yaw = +1.57; //[rad]
double vmax = 1.5E3; //[mm/s]
double wmax = 0.78; //[rad/s]

//X vector managing variables
size_t x_start = 0;
size_t y_start = x_start + N;
size_t tita_start = y_start + N;
size_t V_start = tita_start + N;
size_t W_start = V_start + N-1; //-1 as last actions are not applied

size_t Vy_start = W_start + N-1;//For holonomic
bool holonomic = false;






     class FG_eval {
     public:
          typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;
         FG_eval(double _xp, double _yp, double _ap){
             xp = _xp;
             yp = _yp;
             ap = _ap;
         }

          void operator()(ADvector& fg, const ADvector& x)
          {
             /*
              if(holonomic){
                  holonomic_fg(fg,x);

              }else{
                  non_holonomic_fg(fg,x);
              }
              */

              assert( fg.size() == 3*(N-1) + 1 );
              assert( x.size()  == 3*N + 2*(N-1) );

               // Fortran style indexing?

               // f(x)
               fg[0]=0;

               for(int i=0; i < N; i++){
                     fg[0] += K1*CppAD::pow( CppAD::pow( CppAD::pow(xp-x[x_start+i],2) + CppAD::pow(yp-x[y_start + i],2), 0.5) - dist , 2);
                     fg[0] += K2*CppAD::pow((ap - x[tita_start+i]) - ang,2);
                     fg[0] += K3*CppAD::pow(CppAD::atan2(yp-x[y_start+i], xp-x[x_start+i])- yaw -x[tita_start+i],2);
                }
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

          void non_holonomic_fg(ADvector& fg, const ADvector& x){
              assert( fg.size() == 3*(N-1) + 1 );
              assert( x.size()  == 3*N + 2*(N-1) );

               // Fortran style indexing?

               // f(x)
               fg[0]=0;

               for(int i=0; i < N; i++){
                     fg[0] += K1*CppAD::pow( CppAD::pow( CppAD::pow(xp-x[x_start+i],2) + CppAD::pow(yp-x[y_start + i],2), 0.5) - dist , 2);
                     fg[0] += K2*CppAD::pow((ap - x[tita_start+i]) - ang,2);
                     fg[0] += K3*CppAD::pow(CppAD::atan2(yp-x[y_start+i], xp-x[x_start+i])- yaw -x[tita_start+i],2);
                }
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
         }

          void holonomic_fg(ADvector& fg, const ADvector& x){
              assert( fg.size() == 3*(N-1) + 1 );
              assert( x.size()  == 3*N + 3*(N-1) );
              // f(x)
              fg[0]=0;

              for(int i=0; i < N; i++){
                    fg[0] += K1*CppAD::pow( CppAD::pow( CppAD::pow(xp-x[x_start+i],2) + CppAD::pow(yp-x[y_start + i],2), 0.5) - dist , 2);
                    fg[0] += K2*CppAD::pow((ap - x[tita_start+i]) - ang,2);
                    fg[0] += K3*CppAD::pow(CppAD::atan2(yp-x[y_start+i], xp-x[x_start+i])- yaw -x[tita_start+i],2);
               }
             for(int i=1,j=1; i < N; i++,j+=3){
                 AD<double> x0 = x[x_start + i -1];
                 AD<double> y0 = x[y_start + i -1];
                 AD<double> tita0 = x[tita_start + i -1];
                 AD<double> V0 = x[V_start + i -1];
                 AD<double> W0 = x[W_start + i -1];
                 AD<double> Vy0 = x[Vy_start + i -1];

                 AD<double> x1 = x[x_start + i];
                 AD<double> y1 = x[y_start + i];
                 AD<double> tita1 = x[tita_start + i];



                 fg[j] = x1 - x0 - dt*V0;
                 fg[j+1] = y1 - y0 - dt*Vy0;
                 fg[j+2] = tita1 - tita0 - dt*W0;


             }

          }
     private:
          double xp;
          double yp;
          double ap;

     };


myNLP::myNLP(){}

void myNLP::my_solve(){

    // number of independent variables (domain dimension for f and g)
    size_t nx;
    if(holonomic){
        nx = 3*N + 3*(N-1);//Holonomic
    }else{
        nx = 3*N + 2*(N-1);
    }

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

        if(holonomic){
            x_l[Vy_start + i] = -vmax;//V_r
            x_u[Vy_start + i] = +vmax;
        }


    }



    // Equality constraints. (Dynamic equations) We set the bounds on this constraint
    // to be equal (and zero).
    for (int i=0; i<ng; i++){
        g_l[i] = g_u[i] = 0.0;

    }

    // object that computes objective and constraints
    FG_eval fg_eval(4.0e3,0.0,0.0);

    //std::string options = set_options();
    // options
    std::string options;
    // turn off any printing
    options += "Integer print_level  4\n";
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
    std::vector<double> Wr2(&solution.x[W_start], &solution.x[Vy_start-1]);

    Xr = Xr2;
    Yr=Yr2;
    Titar = Titar2;
    Vr=Vr2;
    Wr=Wr2;




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




}

