#include "unsupported/Eigen"
//#include "unsupported/Eigen/"

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

FVectorType initVec(1);
initVec(0)=1;
Eigen::HybridNonLinearSolver<Line> angleSolver(fa);
angleSolver.solve(initVec);

