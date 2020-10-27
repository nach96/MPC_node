#include <iostream>
#include <vector>
#include <IpTNLP.hpp>

int main(){

      int N = 2;



      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      ///////////////////          JACOBIAN             ////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////

      int iRow[11*N];
      int jCol[11*N];

      int k = 0;
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

      for(int i=0;i<11*N;i++){
            printf("%d",i);
            printf(" Row %d",iRow[i]);
            printf(" Col %d",jCol[i]);
            printf("\n");
      }
      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      ///////////////////          GRAD_F               ////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<double> v;
    Ipopt::Number* values;
    

    v.push_back(1);
    v.push_back(2); // 0,1
    v.push_back(3); // 0,2
    v.push_back(4); // 0,3
    
    v.push_back(5); // 1,0
    v.push_back(6); // 1,1
    v.push_back(7); // 1,2
    v.push_back(8); // 1,3

    values = &v[0];

    for(int i=0; i<v.size(); i++){
      printf("v (%d, %.f)",i,values[i]);
      printf("\n");

    }

      return 1;

}
