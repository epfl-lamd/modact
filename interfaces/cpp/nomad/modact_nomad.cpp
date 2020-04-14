#include <stdexcept>
#include <iostream>
#include "modact_problems.h"
#include "nomad.hpp"

using namespace modact;
using namespace std;

class ModactEvaluator : public NOMAD::Multi_Obj_Evaluator {
public:
  shared_ptr<modact::problem> prob;

  ModactEvaluator  ( const NOMAD::Parameters & param, shared_ptr<modact::problem> p ) :
    NOMAD::Multi_Obj_Evaluator ( param ), prob(p) {}

  ~ModactEvaluator ( void ) {}

  bool eval_x ( NOMAD::Eval_Point &x, const NOMAD::Double &h_max, bool &count_eval ) const {

    vector<double> x_v(x.size());
    for (int i=0; i < x.size(); i++) {
        x_v[i] = x[i].value();
    }
    auto out = prob->operator()(x_v, true);
    int j = 0;
    for (auto f : out.first) {
        x.set_bb_output(j, f);
        j++;
    }
    for (auto c : out.second) {
        x.set_bb_output(j, c);
        j++;
    }

    count_eval = true; // count a black-box evaluation

    return true;       // the evaluation succeeded
  }
};

void StoreParetoFront(ostream &out, NOMAD::Pareto_Front *front) {
    auto cur_point = front->begin();
    while(cur_point != NULL) {
        auto bbo = cur_point->get_bb_outputs();
        bbo.Point::display (out , " " , 13 , -1 );
        out << endl;
        cur_point = front->next();
    }
}

int main( int argc, char* argv[] )
{
    string selected_prob;
    int nfe(0), nx0(0);
    srand (static_cast<int>(time(0)));

    NOMAD::Display out ( cout );
    out.precision ( NOMAD::DISPLAY_PRECISION_STD );

    // NOMAD initializations:
    NOMAD::begin(argc , argv);

    if (argc < 4) {
        cerr << "Not enough arguments usage: problem_name NFE nX0" << endl;
        return 1;
    }

    selected_prob = argv[1];
    cout << "Selected problem: " << selected_prob << endl;
    nfe = stoi(argv[2]);
    nx0 = stoi(argv[3]);

    try {
        auto prob = make_shared<problem>(get_problem(selected_prob));
        NOMAD::Parameters p(out);
        p.set_DIMENSION(prob->get_ndim());  // number of variables

        vector<NOMAD::bb_output_type> bbot(prob->get_nconst() + prob->get_nobj()); // definition of
        for (int i=0; i < prob->get_nobj(); i++) {
            bbot[i] = NOMAD::OBJ;
        }
        for (int i=0; i < prob->get_nconst(); i++) {
            bbot[prob->get_nobj() + i] = NOMAD::EB;
        }
        p.set_BB_OUTPUT_TYPE ( bbot );

        auto bounds = prob->get_bounds();
        auto x0 = NOMAD::Point(prob->get_ndim());
        auto lb = NOMAD::Point(prob->get_ndim());
        auto ub = NOMAD::Point(prob->get_ndim());

        for (int i = 0; i < prob->get_ndim(); ++i) {
            lb[i] = bounds.first[i];
            ub[i] = bounds.second[i];
        }

        p.set_LOWER_BOUND(lb);
        p.set_UPPER_BOUND(ub);

        p.set_X0(lb);
        p.set_X0(ub);

        for (int j=0; j < nx0; j++) {
            for (int i=0; i < prob->get_ndim(); i++) {
                x0[i] = lb[i] + static_cast<NOMAD::Double>(rand())/RAND_MAX * (ub[i] - lb[i]);
            }
            p.set_X0(x0);
        }

        p.set_MULTI_OVERALL_BB_EVAL(nfe);
        p.check();
        ModactEvaluator ev(p, prob);
        // algorithm creation and execution:
        NOMAD::Mads mads ( p , &ev );
        mads.multi_run();
        if (NOMAD::Slave::is_master())
        {
            auto pareto = mads.get_pareto_front();
            ofstream fout ( "best_x.txt" );
            StoreParetoFront(fout, pareto);
            fout.close();
        }
    }
    catch( exception& e ) { cerr << e.what() << endl; }

    NOMAD::Slave::stop_slaves(out);
    NOMAD::end();
    return 0;
}
