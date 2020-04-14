#ifndef MODACT
#define MODACT

#include <memory>
#include <string>
#include <vector>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace std;

namespace modact {

struct SharedPython
{
    py::scoped_interpreter guard{};
    py::module pb = py::module::import("modact.problems");
} sp;

py::object get_problem(const string name)
{
    return sp.pb.attr("get_problem")(name);
}

class problem
{
protected:
    string name;
    py::object pyprob;
    pair<vector<double>, vector<double> > bounds;
    py::tuple weights;
    py::tuple c_weights;
public:
    problem(py::object prob);
    ~problem() {};
    string get_name() { return name; };
    pair<vector<double>, vector<double> > get_bounds() { return bounds; };
    vector<double>::size_type get_ndim() { return bounds.first.size(); };
    vector<double>::size_type get_nconst() { return c_weights.size(); };
    vector<double>::size_type get_nobj() { return weights.size(); };

    pair<vector<double>, vector<double> > operator()(const vector<double>& x, bool minimize = 0, bool bigger_0 = 0);
};

problem::problem(py::object prob) : name(prob.attr("name").cast<string>()), pyprob(prob)
{
    py::tuple b = pyprob.attr("bounds")();
    auto lb = b[0].cast<py::array_t<double> >();
    auto ub = b[1].cast<py::array_t<double> >();
    bounds.first = vector<double>(lb.data(), lb.data() + lb.size());
    bounds.second = vector<double>(ub.data(), ub.data() + ub.size());
    weights = pyprob.attr("weights");
    c_weights = pyprob.attr("c_weights");
}

pair<vector<double>, vector<double> > problem::operator()(const vector<double>& x, bool minimize, bool bigger_0)
{
    double fit_multiplier = minimize ? -1 : 1;
    double const_multiplier = bigger_0 ? -1 : 1;
    py::tuple result = pyprob(x);
    auto fitness = result[0].cast<py::tuple>();
    auto constraints = result[1].cast<py::tuple>();
    auto out_f = vector<double>(fitness.size());
    auto out_g = vector<double>(constraints.size());
    for (auto i = 0; i < weights.size(); ++i)
    {
        out_f[i] = fitness[i].cast<double>()*weights[i].cast<double>()*fit_multiplier;
    }
    for (auto j = 0; j < c_weights.size(); ++j)
    {
        out_g[j] = constraints[j].cast<double>()*c_weights[j].cast<double>()*const_multiplier;
    }
    return make_pair(out_f, out_g);
}

} // namespace modact

#endif
