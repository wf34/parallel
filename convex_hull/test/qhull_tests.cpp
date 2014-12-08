
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/RboxPoints.h>

#include "gtest/gtest.h"

namespace {
TEST (ConvexHullTests, Basic) {
    orgQhull::RboxPoints generator;
    generator.appendPoints("10 D2");
    std::cout << generator.rboxMessage ();
    orgQhull::Coordinates c = generator.getCoordinates ();
    std::cout << "Dim " << generator.dimension () << std::endl;

    orgQhull::Qhull hullComputer (generator, "p Fx");

    orgQhull::QhullPoints points = hullComputer.points ();
    std::cout << points.count () << ".\n";
    std::cout << hullComputer.facetList ();

    ASSERT_TRUE (1 == 0);
}

}
