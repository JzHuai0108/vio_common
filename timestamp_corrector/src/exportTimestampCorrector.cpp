#include <pybind11/pybind11.h>
#include <TimestampCorrector.hpp>

namespace py = pybind11;
 

PYBIND11_MODULE(TimestampCorrector, m)
{
    py::class_< TimestampCorrector<double> >( m, "TimestampCorrector")
        .def(py::init<>())
        .def("correctTimestamp", &TimestampCorrector<double>::correctTimestamp, "correctedEventLocalTime = correctTimestamp(eventRemoteTime, eventLocalTimes).\nNote: This function must be called with monotonically increasing remote timestamps.")
        .def("getLocalTime", &TimestampCorrector<double>::getLocalTime, "eventLocalTime = getLocalTime(eventRemoteTime)")
        .def("convexHullSize", &TimestampCorrector<double>::convexHullSize)
        .def("printHullPoints", &TimestampCorrector<double>::printHullPoints)
        .def("getSlope", &TimestampCorrector<double>::getSlope)
        .def("getOffset", &TimestampCorrector<double>::getOffset)
    ;
}
