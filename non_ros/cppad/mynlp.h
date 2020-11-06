#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

typedef CPPAD_TESTVECTOR( double ) Dvector;

#ifndef MYNLP_H
#define MYNLP_H




class myNLP
{
public:
    myNLP();
    void my_solve();
    void save_solution(CppAD::ipopt::solve_result<Dvector> solution);
    //std::string set_options(void);
    //void set_limits(Dvector x_l, Dvector x_u, Dvector g_l, Dvector g_u);

    std::vector<double> Xr; //(&solution.x[x_start], &solution.x[y_start]);
    std::vector<double> Yr;//(&solution.x[y_start], &solution.x[tita_start]);
    std::vector<double> Titar;//(&solution.x[tita_start], &solution.x[V_start]);
    std::vector<double> Vr;//(&solution.x[V_start], &solution.x[W_start]);
    std::vector<double> Wr;//(&solution.x[W_start], &solution.x[Vy_start-1]);






};

#endif // MYNLP_H
