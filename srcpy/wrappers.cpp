#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "ball_models/ball_trajectory.hpp"
#include "ball_models/racket_contact_model.hpp"
#include "ball_models/table_contact_model.hpp"

namespace py = pybind11;
using namespace ball_models;

PYBIND11_MODULE(ball_models_wrp, m)
{
    py::class_<BallTrajectory>(m, "BallTrajectory")
        .def(py::init<double, double, double, double, double, double, double>())
        .def("step", &BallTrajectory::step)
        .def("calculate_derivative", &BallTrajectory::compute_derivative)
        .def("get_lin_jacobian", &BallTrajectory::compute_jacobian)
        .def("simulate", (void (BallTrajectory::*)(double, double)) &BallTrajectory::simulate)
        .def("update_state", (void (BallTrajectory::*)(Eigen::VectorXd)) &BallTrajectory::update_state);
}