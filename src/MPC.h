#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

namespace vector_index{
    enum StateIdx{
        siX,
        siY,
        siPsi,
        siV,
        siCTE,
        siEPsi,
        siCount
    };
}

class MPC {
 public:
  MPC();

  virtual ~MPC();
  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
