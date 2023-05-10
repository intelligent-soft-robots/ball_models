#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "ball_models/ball_trajectory.hpp"

namespace py = pybind11;
using namespace ball_models;

PYBIND11_MODULE(ball_models_wrp, m)
{
    m.doc() = "Wrapper class for ball models.";

    py::class_<ball_models::BallTrajectory>(m, "BallTrajectory")
        //.def("update_state", &BallTrajectory::update_state)
        //.def("step", &BallTrajectory::step)
        //.def("compute_derivative", &BallTrajectory::compute_derivative)
        .def(py::init<double, double, double, double, double, double, double>())
        .def(py::init<std::string>())
        .def("integrate", &ball_models::BallTrajectory::integrate)
        .def("integrate_with_contacts", &ball_models::BallTrajectory::integrate_with_contacts)
        .def("simulate", &ball_models::BallTrajectory::simulate)
        .def("compute_jacobian", &ball_models::BallTrajectory::compute_jacobian);
}