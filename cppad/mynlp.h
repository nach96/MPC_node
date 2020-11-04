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






};

#endif // MYNLP_H
