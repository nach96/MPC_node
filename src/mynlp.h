#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

typedef CPPAD_TESTVECTOR( double ) Dvector;



#ifndef FG_EVAL_H
#define FG_EVAL_H

class FG_eval 
{
public:
    typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;

    FG_eval();
    FG_eval(double _xp,  double _yp, double _ap, double _dist, double _ang, double _yaw, double _K1, double _K2, double _K3, double _K4, double _K5, double _preV, double _preW);
    void operator()(ADvector& fg, const ADvector& x);

private:
  double xp;
  double yp;
  double ap;
  AD<double> K1;// = 1.0; //Distance error
  AD<double> K2;// = 1.0; //Tita error
  AD<double> K3;// = 1.0; // Yaw error
  AD<double> K4; // = 0.5; // acceleration limit
  AD<double> K5; // = 0.5; //Angular acceleration limit
  AD<double> dist;// = 3.0;  //[m]
  AD<double> ang;// = 0.0; //[rad]
  AD<double> yaw;// = -1.57; //[rad]
  AD<double> preV;
  AD<double> preW;
  

};


#endif





#ifndef MYNLP_H
#define MYNLP_H

class myNLP
{
public:
    myNLP(double _vmax, double _wmax);
    void my_solve(double xp, double yp, double ap,double dist, double ang, double yaw, double K1, double K2, double K3, double K4, double K5);
    void save_solution(CppAD::ipopt::solve_result<Dvector> solution);
    //std::string set_options(void);
    //void set_limits(Dvector x_l, Dvector x_u, Dvector g_l, Dvector g_u);

    std::vector<double> Xr; //(&solution.x[x_start], &solution.x[y_start]);
    std::vector<double> Yr;//(&solution.x[y_start], &solution.x[tita_start]);
    std::vector<double> Titar;//(&solution.x[tita_start], &solution.x[V_start]);
    std::vector<double> Vr;//(&solution.x[V_start], &solution.x[W_start]);
    std::vector<double> Wr;//(&solution.x[W_start], &solution.x[Vy_start-1]);

private:
    double vmax; // = 1.5;//E3; //[mm/s]
    double wmax; // = 0.78; //[rad/s]
    FG_eval fgeval;
    double preV;
    double preW;
    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    double av = 1*0.1;
    double aw = 1*0.1;

   


};

#endif // MYNLP_H
