#include <iostream>
#include "modact_problems.h"

using namespace modact;
using namespace std;

int main(int argc, char** argv)
{
    string selected_prob;

    if (argc > 1) {
        selected_prob = argv[1];
        cout << "Selected: " << selected_prob << endl;
        try {
            auto prob = make_unique<problem>(get_problem(selected_prob));
            cout << "Problem exists!" << endl;
            cout << "ndims: " << prob->get_ndim() << endl;
            cout << "nconsts: " << prob->get_nconst() << endl;
            cout << "nobjs: " << prob->get_nobj() << endl;
            auto ret = prob->operator()(prob->get_bounds().first);
            py::print(ret);
        } catch (exception &e) {
            cerr << e.what() << endl;
        }
    }
    return 0;
}
