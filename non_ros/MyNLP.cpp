// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#include "MyNLP.hpp"

#include <cassert>
#include <math.h>
#include <iostream>
#include <fstream>

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

using namespace Ipopt;

/* Constructor. */
MyNLP::MyNLP()
{ }

MyNLP::~MyNLP()
{ }

bool MyNLP::get_nlp_info(
   Index&          n,
   Index&          m,
   Index&          nnz_jac_g,
   Index&          nnz_h_lag,
   IndexStyleEnum& index_style
)
{
   // Decision Variables: 3 states and 2 inputs each control step.
   n = 5*(N+1); //P.ej, 2 intervalos, 3 nodos con variables.

   // 3 equality constraints each control step. (Discretized system equations)
   m = 3*N;

   // Number of nonzeros in the jacobian (one for x1, and one for x2),
   nnz_jac_g = 11*N;

   // Number of nonzeros in the hessian of the lagrangian
   nnz_h_lag = 7*N;

   // We use the standard fortran index style for row/col entries
   index_style = C_STYLE;

   return true;
}

bool MyNLP::get_bounds_info(
   Index   n,
   Number* x_l,
   Number* x_u,
   Index   m,
   Number* g_l,
   Number* g_u
)
{
   // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
   // If desired, we could assert to make sure they are what we think they are.
   assert(n == 5*(N+1));
   assert(m == 3*N);

   //If No bounds = -/+1e19

   //Start position is fixed. I set initial position as reference frame. zero
   x_l[0]=x_u[0]=0.0; //Xr0
   x_l[1]=x_u[1]=0.0; //Yr0
   x_l[2]=x_u[2]=0.0; //Titar0
   x_l[3] = -vmax;//V_r
   x_u[3] = +vmax;
   x_l[4] = -wmax;//w_r
   x_u[4] = +wmax;

   
   for(int i=5; i<5*(N+1); i+=5){
      /*
      x_l[i] = -1.0e19;//Xr
      x_u[i] = +1.0e19;
   
      x_l[i+1] = -1.0e19;//Yr
      x_u[i+1] = +1.0e19;

      x_l[i+2] = -1.0e19;//tita_r
      x_u[i+2] = +1.0e19;
      */
      x_l[i] = -40.0;//Xr
      x_u[i] = +40.0;
   
      x_l[i+1] = -40.0;//Yr
      x_u[i+1] = +40.0;

      x_l[i+2] = -40.0;//tita_r
      x_u[i+2] = +40.0;

      x_l[i+3] = -vmax;//V_r
      x_u[i+3] = +vmax;

      x_l[i+4] = -wmax;//w_r
      x_u[i+4] = +wmax;

   }


   // Equality constraints. (Dynamic equations) We set the bounds on this constraint
   // to be equal (and zero).
   for (int i=0;i<3*N;i++){
      g_l[i] = g_u[i] = 0.0;

   }
   

   return true;
}

bool MyNLP::get_starting_point(
   Index   n,
   bool    init_x,
   Number* x,
   bool    init_z,
   Number* z_L,
   Number* z_U,
   Index   m,
   bool    init_lambda,
   Number* lambda
)
{
   // I should try to give starting values at least for x.
   assert(init_x == true);
   assert(init_z == false);
   assert(init_lambda == false);

   x[0]=0.0;
   x[1]=0.0;
   x[2]=0.0;


   /*
   x[5*N-3]=0.0;
   x[5*N-4]=0.0;
   x[5*N-5]=2.0;
   
   //Start all variables as positives
   for(int i=0;i<5*N;i++){
      x[i]=1;
   }
   */

   // we initialize x in bounds, in the upper right quadrant
   

   return true;
}

bool MyNLP::eval_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number&       obj_value
)
{
   // return the value of the objective function

   for(int i=0; i<3*(N+1); i+=3){
      Number xpr2 = (x[i]-xp)*(x[i]-xp);
      Number ypr2 = (x[i+1]-yp)*(x[i+1]-yp);
      obj_value += K1*(xpr2 + ypr2 - dist2)*(xpr2 + ypr2 - dist2) + K2*(x[i+2]-ang)*(x[i+2]-ang);
   }
   

   return true;
}

bool MyNLP::eval_grad_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number*       grad_f
)
{
   // return the gradient of the objective function grad_{x} f(x)


   for(int i=0; i<5*(N+1); i+=5){
      Number xpr2 = (x[i]-xp)*(x[i]-xp);
      Number ypr2 = (x[i+1]-yp)*(x[i+1]-yp);

      grad_f[i] = 4.0*K1*(x[i]-xp)*(xpr2 + ypr2 - dist2);
      grad_f[i+1] = 4.0*K1*(x[i+1]-yp)*(xpr2 + ypr2 - dist2);
      grad_f[i+2] = 2.0*K2*x[i+2];
      //grad_f[i+2] = 0;
      grad_f[i+3] = 0;
      grad_f[i+4] = 0;

   }


   return true;
}

bool MyNLP::eval_g(
   Index         n,
   const Number* x,
   bool          new_x,
   Index         m,
   Number*       g
)
{
   // return the value of the constraints: g(x)

   //Dynamic equations: (= 0)
   // xr1 - xr0 - dt*cos(tita0)*Vr0 = 0
   // yr1 - yr0 - dt*sin(tita0)*Vr0 = 0
   // titar1 - titar0 - dt*Wr0 = 0

   for(int i=0, j=0; i<5*N; i+=5, j+=3){
      //i loops over x (step 5)
      //j loops over g (step 3)
      g[j] = x[i+5] - x[i] -dt*cos(x[i+2])*x[i+3];
      g[j+1] = x[i+6] - x[i+1] -dt*sin(x[i+2])*x[i+3];
      g[j+2] = x[i+7] - x[i+2] - dt*x[i+4];

   }

   return true;
}

bool MyNLP::eval_jac_g(
   Index         n,
   const Number* x,
   bool          new_x,
   Index         m,
   Index         nele_jac,
   Index*        iRow,
   Index*        jCol,
   Number*       values
)
{

   assert(nele_jac = 11*N);

   if( values == NULL )
   {
      // return the structure of the jacobian of the constraints

      int k = 0;
      //Loop around each interval. 
      for(int i=0, j=0; i<3*N; i+=3, j+=5){         
            
            iRow[k] = i;
            jCol[k] = j;
            k++;
            iRow[k] = i;
            jCol[k] = j+2;
            k++;
            iRow[k] = i;
            jCol[k] = j+3;
            k++;
            iRow[k] = i;
            jCol[k] = j+5;
            k++;

            iRow[k] = i+1;
            jCol[k] = j+1;
            k++;
            iRow[k] = i+1;
            jCol[k] = j+2;
            k++;
            iRow[k] = i+1;
            jCol[k] = j+3;
            k++;
            iRow[k] = i+1;
            jCol[k] = j+6;
            k++;

            iRow[k] = i+2;
            jCol[k] = j+2;

            k++;
            iRow[k] = i+2;
            jCol[k] = j+4;
            k++;
            iRow[k] = i+2;
            jCol[k] = j+7;
            k++;
      }
 

      }

   else
   {

      //Loop around each interval. 
      int k = 0;
      for(int i=0, j=0; i<3*N; i+=3, j+=5){         
            
            values[k] = -1.0;
            k++;
            values[k] = dt*x[j+3]*sin(x[j+2]);
            k++;
            values[k] = -dt*cos(x[j+2]);
            k++;
            values[k] = 1.0;
            k++;

            values[k] = -1.0;
            k++;
            values[k] = -dt*x[j+3]*cos(x[j+2]);
            k++;
            values[k] = -dt*sin(x[j+2]);
            k++;
            values[k] = 1.0;
            k++;

            values[k] = -1.0;
            k++;
            values[k] = -dt;
            k++;
            values[k] = 1.0;
            k++;
      }
      

   }

   return true;
}

bool MyNLP::eval_h(
   Index         n,
   const Number* x,
   bool          new_x,
   Number        obj_factor,
   Index         m,
   const Number* lambda,
   bool          new_lambda,
   Index         nele_hess,
   Index*        iRow,
   Index*        jCol,
   Number*       values
)
{
   
   if( values == NULL )
   {
      // return the structure. 
      int k = 0;
      //Loop around each interval. 
      for(int i=0; i<5*N; i+=5){
         iRow[k]=i;
         jCol[k]=i;
         k++;
         iRow[k]=i;
         jCol[k]=i+1;
         k++;
         iRow[k]=i+1;
         jCol[k]=i;
         k++;
         iRow[k]=i+1;
         jCol[k]=i+1;
         k++;
         iRow[k]=i+2;
         jCol[k]=i+2;
         k++;
         iRow[k]=i+2;
         jCol[k]=i+3;
         k++;
         iRow[k]=i+3;
         jCol[k]=i+2;
         k++;
      }

   }
   else
   {
      // return the values
      int k = 0;
      //Loop around each interval. 
      for(int i=0,j=0; i<5*N; i+=5,j+=3){
         Number xpr2 = (x[i]-xp)*(x[i]-xp);
         Number ypr2 = (x[i+1]-yp)*(x[i+1]-yp);

         values[k]=8.0*K1*xpr2 + 4.0*K1*(xpr2+ypr2-dist2); //term from grad2f
         k++;
         values[k]=8.0*K1*(x[i]-xp)*(x[i+1]-yp);//term from grad2f
         k++;
         values[k]=8.0*K1*ypr2 + 4.0*K1*(xpr2+ypr2-dist2);//term from grad2f
         k++;
         values[k]=8.0*K1*(x[i]-xp)*(x[i+1]-yp);//term from grad2f
         k++;
         values[k] = lambda[j]*dt*x[i+3]*cos(x[i+2]);//term from lambda1,grad2g
         values[k]+= lambda[j+1]*dt*x[i+3]*sin(x[i+2]);//term from lambda2,grad2g
         k++;
         values[k] = 2.0*K2; //term from grad2f
         values[k]+= lambda[j]*dt*sin(x[i+2]);
         values[k]+= -lambda[j+1]*dt*cos(x[i+2]);
         k++;
         values[k] = lambda[j]*dt*sin(x[i+2]);
         values[k]+= -lambda[j+1]*dt*cos(x[i+2]);
         k++;


      }


      // Note: off-diagonal elements are zero for this problem
   }
   //return true;
   
   return false; //Dummy function because Quasi-Newton Approximation is being used
   
}

void MyNLP::finalize_solution(
   SolverReturn               status,
   Index                      n,
   const Number*              x,
   const Number*              z_L,
   const Number*              z_U,
   Index                      m,
   const Number*              g,
   const Number*              lambda,
   Number                     obj_value,
   const IpoptData*           ip_data,
   IpoptCalculatedQuantities* ip_cq
)
{
   for(int i=0; i<5*N;i+=5){
      Xr.push_back(x[i]);
      Yr.push_back(x[i+1]);
      Titar.push_back(x[i+2]);
      Vr.push_back(x[i+3]);
      Wr.push_back(x[i+4]);
   }

   //Print a matrix with the results to plot it in matlab
   std::ofstream out("X.txt");

   //printf("Xr:");
   //Xr; Yr; Titar; Vr; Wr
   out << "X = [";
   for(auto it = Xr.begin() ; it != Xr.end(); it++){
      std::cout << ' ' << *it;
      std::cout << '\n';
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
    out << "; ";
   for(auto it = Wr.begin() ; it != Wr.end(); it++){
      out << *it << " ";

   }
   out << "]";
}
