
#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <iterator>
#include <stack>
#include <string>
#include <vector>

extern "C" {
#include "FreeImage.h"
}

using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::vector;


struct Point2D {
    Point2D ();
    Point2D (double x, double y);
    bool operator < (const Point2D& b);

    double x;
    double y;
};
std::ostream& operator<< (std::ostream& stream, const Point2D& point);

vector<Point2D> read_points (std::istream& stream = cin);
void write_points (const vector<Point2D>& points, std::ostream& stream = cout);
vector<Point2D> graham_scan (vector<Point2D> points);

void draw_hull (const vector<Point2D>& points, const vector<Point2D>& hull);
void draw_line (FIBITMAP* bitmap, const Point2D& src, const Point2D& dst);

enum orientation_status {
    UNDEF = 0,
    COLLINEAR,
    CLOCKWISE,
    COUNTER_CLOCKWISE
};

// To find orientation of ordered triplet (p, q, r).
orientation_status orientation (const Point2D& p, const Point2D& q, const Point2D& r);

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

    bool operator () (const Point2D& first, const Point2D& second) const {

        // Find orientation
        orientation_status status = orientation (seed_point, first, second);
        if (status == COLLINEAR) {
            return dist (seed_point, first) >= dist (seed_point, second);
        }

        return status != CLOCKWISE;
    }

    Point2D seed_point;
};

/////////////////////////////////////////////////////////////////////////////

Point2D::Point2D ()
    : x (0.0)
    , y (0.0) {
}



Point2D::Point2D (double x, double y)
    : x (x)
    , y (y) {
}



bool Point2D::operator < (const Point2D& b) {
    if (y != b.y)
        return y < b.y;
    return x < b.x;
}



std::ostream& operator<< (std::ostream& stream, const Point2D& point) {
    stream << "[" << point.x << ", " << point.y << "]";
    return stream;
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



vector<Point2D> graham_scan (vector<Point2D> points) {
    auto bottommost_element = std::min_element (points.cbegin(), points.cend (),
                                                compare_points_closer_to_leftmost ());
    vector<Point2D>::iterator bottommost = points.begin ();
    std::advance (bottommost, std::distance<vector<Point2D>::const_iterator> (bottommost, bottommost_element));

    std::iter_swap (points.begin (), bottommost);

    std::sort (points.begin () + 1, points.end (),
               compare_points_polar_angle_bigger (*points.begin ()));

    vector <Point2D> hull = { points.at (0), points.at (1) };
    int i = 2;
    while (i < points.size ())
    {   while (COUNTER_CLOCKWISE != orientation (*(hull.end () - 2),
                                                 *(hull.end () - 1),
                                                 points.at (i)))
        {   if (hull.size () > 2)
            {   hull.pop_back ();
            } else if (i == points.size () - 1)
            {   break;
            } else
            {   ++i;
            }
        }

        hull.emplace_back (points.at (i));
        ++i;
    }


    return hull;
}


orientation_status orientation (const Point2D& p, const Point2D& q, const Point2D& r) {
    double val = (q.x - p.x) * (r.y - p.y) -
        (r.x - p.x) * (q.y - p.y);

    if (0.0 == val) {
        return COLLINEAR;
    }

    return (val > 0.0) ? COUNTER_CLOCKWISE : CLOCKWISE;
}


double dist (const Point2D& first, const Point2D& second) {
    return std::pow (first.x - second.x, 2) +
           std::pow (first.y - second.y, 2);
}



Point2D next_to_top (std::stack<Point2D>& S) {
    assert (S.size () >= 2);

    Point2D p = S.top ();
    S.pop ();
    Point2D res = S.top ();
    S.push (p);
    return res;
}


void draw_line (FIBITMAP* bitmap, const Point2D& src, const Point2D& dst)
{   Point2D left (src);
    Point2D right (dst);
    if (left.x > right.x)
    {   std::swap (left, right);
    }

    RGBQUAD red_colour;
    red_colour.rgbRed = 255;
    red_colour.rgbGreen = 0;
    red_colour.rgbBlue = 0;

    double x_step = (right.x - left.x) / 100.0;
    double y_step = (right.y - left.y) / 100.0;
    for (int i = 0; i < 100; ++i) {
        FreeImage_SetPixelColor (bitmap,
            std::floor (left.x + x_step * static_cast<double> (i)+0.5),
            std::floor (left.y + y_step * static_cast<double> (i)+0.5),
            &red_colour);
    }
}

void draw_hull (const vector<Point2D>& points, const vector<Point2D>& hull) {
    const int width = 640;
    const int height = 480;
    
    std::function <bool (const Point2D&, const Point2D&)> compare_points_farther =
        [](const Point2D& first, const Point2D& second) -> bool
    {   return std::max (std::abs (first.x), std::abs (first.y)) <
               std::max (std::abs (second.x), std::abs (second.y));
    };

    auto most_distant_point = std::max_element (points.cbegin (), points.cend (), compare_points_farther);
    const double scale = 200 / std::max (std::abs (most_distant_point->x), std::abs (most_distant_point->y));
    const double center_x = static_cast<double> (width) / 2.0;
    const double center_y = static_cast<double> (height) / 2.0;
    const int bits_per_pixel = 24;
    FreeImage_Initialise ();
    FIBITMAP* bitmap = FreeImage_Allocate (width, height, bits_per_pixel);
    
    if (nullptr == bitmap)
    {   cout << "bitmap creation failed" << endl;
        return;
    }
    
    std::function <Point2D (const Point2D&)> convert_to_frame_space =
        [scale, center_x, center_y] (const Point2D& point) -> Point2D
    {   Point2D frame_point;
        frame_point.x = point.x * scale + center_x;
        frame_point.y = point.y * scale + center_y;
        return frame_point;
    };

    RGBQUAD green_colour;
    green_colour.rgbRed = 0;
    green_colour.rgbGreen = 255;
    green_colour.rgbBlue = 0;
    RGBQUAD white_colour;
    white_colour.rgbRed = 255;
    white_colour.rgbGreen = 255;
    white_colour.rgbBlue = 255;
    RGBQUAD black_colour;
    black_colour.rgbRed = 0;
    black_colour.rgbGreen = 0;
    black_colour.rgbBlue = 0;

    FreeImage_FillBackground (bitmap, &white_colour);
    draw_line (bitmap, Point2D (center_x, 0), Point2D (center_x, height));
    draw_line (bitmap, Point2D (0, center_y), Point2D (width, center_y));

    for (vector<Point2D>::const_iterator point = hull.cbegin ();
         point != hull.cend (); ++point)
    {   auto prev_point = (point == hull.cbegin ()) ? hull.cend () - 1 : point - 1;
        Point2D prev_frame_point = convert_to_frame_space (*prev_point);
        Point2D frame_point = convert_to_frame_space (*point);
        draw_line (bitmap, prev_frame_point, frame_point);
    }

    for (const Point2D& point : points)
    {   Point2D frame_point = convert_to_frame_space (point);
        RGBQUAD current_color (black_colour);
        if (point.x == points.front ().x &&
            point.y == points.front ().y)
        {   current_color = green_colour;
        }

        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x + 1, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x - 1, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y - 1, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y + 1, &current_color);
    }

    if (0 == FreeImage_Save (FIF_PNG, bitmap, "C:/projects/parallel/convex_hull/build/Release/hull.png"))
    {   cout << "Saving failed" << endl;
    }
    FreeImage_DeInitialise ();
}



int main () {
    cout.setf (std::ios_base::fixed, std::ios_base::floatfield);
    cout.precision (17);

    vector<Point2D> points = read_points ();
    vector<Point2D> convex_hull = graham_scan (points);
    write_points (convex_hull);


    draw_hull (points, convex_hull);
    return 0;
}