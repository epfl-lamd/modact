#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>
#include <time.h>
#include <math.h>
#include <mpi.h>
#include <vector>
#include <memory>
#include <iostream>
#include "borgms.h"

#include "modact_problems.h"

using namespace std;
using namespace modact;

shared_ptr<modact::problem> prob;

void obj_func(double* vars, double* objs, double* consts) {
	vector<double> x_v(prob->get_ndim());
	for (int i=0; i < x_v.size(); i++) {
		x_v[i] = vars[i];
	}
	auto out = prob->operator()(x_v, true);
	copy(out.first.begin(), out.first.end(), objs);
	int j = 0;
	for (auto c : out.second) {
		if (c < 0) {
			c = 0;  // Borg treats every none-0 value as violation
		}
		consts[j] = c;
		j++;
	}
}

int main(int argc, char* argv[]) {
	string selected_prob;
	int nfe(100);
	int rank;
	char runtime[256];
	char result_file[256];

	BORG_Problem problem;

	BORG_Algorithm_ms_startup(&argc, &argv);
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);

	if (argc < 3) {
		cerr << "Not enough arguments usage: problem_name NFE" << endl;
		return 1;
	}

	selected_prob = argv[1];
	nfe = stoi(argv[2]);

	try {
		if (rank == 0) cout << "Selected problem: " << selected_prob << endl;
		prob = make_shared<modact::problem>(get_problem(selected_prob));
		
		if (rank == 0) cout << "Max evaluations: " << nfe << endl;
		if (rank == 0) cout << "Optimizing: " << selected_prob << endl;

		BORG_Algorithm_ms_max_evaluations(nfe);

		auto bounds = prob->get_bounds();
		auto nvars = prob->get_ndim();
		// Define the problem.
		problem = BORG_Problem_create(nvars, prob->get_nobj(), prob->get_nconst(), obj_func);

		for (auto j(0); j<nvars; j++) {
			BORG_Problem_set_bounds(problem, j, bounds.first[j], bounds.second[j]);
		}

		for (auto j(0); j<prob->get_nobj(); j++) {
			BORG_Problem_set_epsilon(problem, j, 0.005);
		}

		// Seed the random number generator.
		BORG_Random_seed(37*(rank+1));

		// Run the MPI Borg MOEA on the problem.
		BORG_Archive result = BORG_Algorithm_ms_run(problem);

		// Print the Pareto optimal solutions to the screen and save to file
		if (result != NULL) {
			BORG_Archive_print(result, stdout);
			sprintf(result_file, "%s.txt", selected_prob.c_str());
			FILE* fp = fopen(result_file, "w");
			BORG_Archive_print(result, fp);
			fclose(fp);
			BORG_Archive_destroy(result);
		}
		BORG_Problem_destroy(problem);
	}
	catch( exception& e ) { cerr << e.what() << endl; }

	// Shutdown the parallel processes and exit.
	BORG_Algorithm_ms_shutdown();
	return EXIT_SUCCESS;
}
