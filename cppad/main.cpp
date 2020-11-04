#include <iostream>
#include "mynlp.cpp"

using namespace std;

int main()
{
    //cout << "Hello World!" << endl;
    myNLP mynlp;

    mynlp.my_solve();

   // std::vector<double> Xprint = mynlp.Xr;

     for(auto it = mynlp.Xr.begin() ; it != mynlp.Xr.end(); it++){
        //std::cout << ' ' << *it;
        //std::cout << '\n';
        std::cout << *it << " ";
     }



    return 0;
}
