# MODAct

Python package for the Multi-Objective Design of electro-mechanical Actuators
that is used to derive 20 benchmark problems for constrained multi-objective
optimization.

There is a related publication currently under review.

## Installation

`modact` has a few requirements listed in `requirements.txt`. In particular,
[python-fcl](https://github.com/BerkeleyAutomation/python-fcl) needs to be
installed along with the required `fcl` shared library.

The easiest way to get started is to build a Docker image.

```bash
docker build -t modact .
```

Otherwise, users can install fcl through their package manager (apt, brew, vcpkg)
and then run:

```bash
pip install -r requirements.txt
python setup.py install
```

## Usage

Each benchmark problem is in a self-contained object:

```python
import modact.problems as pb

# Create problem
cs1 = pb.get_problem('cs1')

# Get search bounds
xl, xu = cs1.bounds()

# Objective weights: -1 --> minimization / 1 --> maximization
cs1.weights  # (-1, 1)
# Constraints weights: -1 --> g(x) >= 0 / 1 --> g(x) <= 0
cs1.c_weights  # (-1, -1, -1, -1, -1, -1, -1)

# To evaluate a vector
f, g = cs1(xl)
```

Note that the output of the function call is not per se automatically converted
to a minimization problem. The `weights` and `c_weights` tuples need to be used.
An example of how this is done is given in the adapter for pymoo:
`modact.interfaces.pymoo`.

Usage examples are shown in the `scripts` folder. In particular, optimization
example using [pymoo](https://github.com/msu-coinlab/pymoo) are given.

Interfaces form different languages (C++ and MATLAB) to python are provided in
the `interfaces` folder.

The best-known Pareto fronts approximations of the 20 problems can be downloaded
here: [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3647916.svg)](https://doi.org/10.5281/zenodo.3647916)
