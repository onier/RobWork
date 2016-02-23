/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include <rwlibs/algorithms/optimization/QPProblem.hpp>

using namespace rwlibs::algorithms;

QPProblem::QPProblem(const Eigen::MatrixXd& Q, const Eigen::VectorXd& c, const Eigen::MatrixXd& A, const Eigen::VectorXd& b):
	_f(Q,c), _A(A), _b(b)
{
}

QPProblem::QPProblem(const QuadraticFunction& f, const Eigen::MatrixXd& A, const Eigen::VectorXd& b):
	_f(f), _A(A), _b(b)
{
}

QPProblem::~QPProblem() {
}
