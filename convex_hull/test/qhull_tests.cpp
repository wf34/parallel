
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/RboxPoints.h>

#include "gtest/gtest.h"

namespace {
TEST (ConvexHullTests, Basic) {
    orgQhull::RboxPoints generator;
    generator.appendPoints("100");

    orgQhull::Qhull hullComputer;
    hullComputer.runQhull (generator, "");
    std::cout << hullComputer.facetList ();

    ASSERT_TRUE (1 == 0);
}

}
