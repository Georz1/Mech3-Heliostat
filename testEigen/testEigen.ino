#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>

#include <HybridNonLinearSolver.h>


class Line {
	double a;	// slope
	double b;	// y-intercept

public:
	Line(double slope = 1, double yintercept = 1):
		a(slope),b(yintercept){} 
	double operator()(double x){
		return a*x + b;
	}
};

Line fa;			// y = 1*x + 1
Line fb(5.0,10.0);

const int n=9;
int info;
Eigen::VectorXd x_init = Eigen::VectorXd::Constant(n, -1.);

Eigen::HybridNonLinearSolver<Line> angleSolver(fa);
angleSolver.solve(initVec);
