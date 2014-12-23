
#include <algorithm>
#include <iostream>
#include <stack>
#include <string>
#include <vector>

using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::vector;


struct Point2D {
    Point2D (double x, double y);
    Point2D& operator = (Point2D&& other);
    double x;
    double y;
};

vector<Point2D> read_points (std::istream& stream = cin);
void write_points (const vector<Point2D>& points, std::ostream& stream = cout);
vector<Point2D> graham_scan (const vector<Point2D>& points);

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation (const Point2D& p, const Point2D& q, const Point2D& r);

// A utility function to return square of distance between p1 and p2
double dist (const Point2D& first, const Point2D& second);

// A utility function to find next to top in a stack
Point2D next_to_top (std::stack<Point2D>& S);

struct compare_points_closer_to_leftmost {
    bool operator () (const Point2D& first, const Point2D& second) {
        if (first.y - second.y < -1e-5) {
            return true;
        } else if (std::abs (first.y - second.y) < 1e-5) {
            return first.x - second.x < -1e-5;
        } else {
            return false;
        }
    }
};

// A function used by library function qsort() to sort an array of
// points with respect to the first point
struct compare_points_polar_angle_bigger {
    explicit compare_points_polar_angle_bigger (const Point2D& seed)
        : seed_point (seed) {
    }

    bool operator () (const Point2D& first, const Point2D& second) {

        // Find orientation
        int o = orientation (seed_point, first, second);
        if (o == 0) {
            return !(dist (seed_point, second) >= dist (seed_point, first));
        }

        return !(o == 2);
    }

    Point2D seed_point;
};

/////////////////////////////////////////////////////////////////////////////

Point2D::Point2D (double x, double y)
    : x (x)
    , y (y) {
}



Point2D& Point2D::operator = (Point2D&& other) {
    x = std::move (other.x);
    y = std::move (other.y);
}



vector<Point2D> read_points (std::istream& stream) {
    string status_line;
    std::getline (cin, status_line);
    int pointsAmount;
    stream >> pointsAmount;

    vector<Point2D> points;
    points.reserve (pointsAmount);
    for (int i = 0; i < pointsAmount; ++i) {
        double x, y;
        stream >> x;
        stream >> y;
        points.emplace_back (x,y);
    }

    return points;
}



void write_points (const vector<Point2D>& points, std::ostream& stream) {
    const int dimensionality = 2;
    stream << dimensionality << endl;
    stream << points.size () << endl;
    for (const Point2D& point : points) {
        stream << point.x << " " << point.y << endl;
    }
}



// Prints convex hull of a set of n points.
vector<Point2D> graham_scan (const vector<Point2D>& points) {
    // Find the bottommost point
    auto bottommost_element = std::min_element (points.cbegin(), points.cend (),
                                                                     compare_points_closer_to_leftmost ());

    // Place the bottom-most point at first position
    std::iter_swap (points.begin (), bottommost_element);

    // Sort n-1 points with respect to the first point.  A point p1 comes
    // before p2 in sorted ouput if p2 has larger polar angle (in
    // counterclockwise direction) than p1
    std::sort (points.begin (), points.end (),
               compare_points_polar_angle_bigger (*points.begin ()));

    // Create an empty stack and push first three points to it.
    std::stack<Point2D> S;
    S.emplace (points[0]);
    S.emplace (points[1]);
    S.emplace (points[2]);

    // Process remaining n-3 points
    for (int i = 3; i < points.size (); i++)
    {
        // Keep removing top while the angle formed by points next-to-top,
        // top, and points[i] makes a non-left turn
        while (orientation (next_to_top (S), S.top (), points[i]) != 2)
            S.pop ();
        S.push (points[i]);
    }

    // Now stack has the output points, print contents of stack
    while (!S.empty ())
    {
        Point2D p = S.top ();
        cout << "(" << p.x << ", " << p.y << ")" << endl;
        S.pop ();
    }
    return vector<Point2D> ();
}


int orientation (const Point2D& p, const Point2D& q, const Point2D& r) {
    int val = (q.y - p.y) * (r.x - q.x) -
        (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0;  // colinear
    return (val > 0) ? 1 : 2; // clock or counterclock wise
}


double dist (const Point2D& first, const Point2D& second) {
    return (first.x - second.x)*(first.x - second.x) +
           (first.y - second.y)*(first.y - second.y);
}



Point2D next_to_top (std::stack<Point2D>& S) {
    Point2D p = S.top ();
    S.pop ();
    Point2D res = S.top ();
    S.push (p);
    return res;
}



int main () {
    cout.setf (std::ios_base::fixed, std::ios_base::floatfield);
    cout.precision (17);

    vector<Point2D> points = read_points ();
    vector<Point2D> convex_hull = graham_scan (points);
    write_points (convex_hull);

    return 0;
}