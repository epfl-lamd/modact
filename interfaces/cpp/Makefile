PYBIND11_INCLUDE = `python3 -m pybind11 --includes`
PYTHON_LIB_DIR = `python -c "import sys; print(sys.exec_prefix + '/lib')"`

modact: test_modact.cpp modact_problems.h
	c++ -O3 -Wall -std=c++14 -undefined dynamic_lookup $(PYBIND11_INCLUDE) test_modact.cpp -o modact -l python3.7m -L$(PYTHON_LIB_DIR)
	install_name_tool -add_rpath $(PYTHON_LIB_DIR) modact

test:
	./modact cs2
