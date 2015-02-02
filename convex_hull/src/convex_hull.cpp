
#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <iterator>
#include <queue>
#include <set>
#include <stack>
#include <string>
#include <vector>

extern "C" {
#include "FreeImage.h"

#include "mcbsp.h"
}

#include "logging.h"

using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::vector;


struct Point2D {
    Point2D ();
    Point2D (double x, double y);
    bool operator < (const Point2D& b) const;
    bool operator == (const Point2D& b) const;
    bool operator != (const Point2D& b) const;

    double x;
    double y;
};
std::ostream& operator<< (std::ostream& stream, const Point2D& point);

typedef vector<Point2D>::iterator Point2D_iterator;
typedef vector<Point2D>::const_iterator Point2D_const_iterator;

vector<Point2D> read_points (std::istream& stream = cin);
void write_points (const vector<Point2D>& points, std::ostream& stream = cout);
vector<Point2D> graham_scan (vector<Point2D> points);
// triangles association (by CS341, pp.125)
vector<Point2D> compute_enet (const vector<Point2D>& points);


string get_app_name (const string& full_path);

void draw_hull (const string& path,
                const vector<Point2D>& points,
                const vector<Point2D>& hull,
                const Point2D& apex = {-1, -1});

void draw_subset (const string& path,
                  const vector<Point2D>& points,
                  const vector<Point2D>& subset,
                  const Point2D& apex = {-1, -1},
                  const vector<std::pair<Point2D, Point2D>>& lines =
                  vector<std::pair<Point2D, Point2D>> ());
void draw_line (FIBITMAP* bitmap, const Point2D& src, const Point2D& dst);

enum orientation_status {
    UNDEF = 0,
    COLLINEAR,
    CLOCKWISE,
    COUNTER_CLOCKWISE
};

// To find orientation of ordered triplet (p, q, r).
orientation_status orientation (const Point2D& p, const Point2D& q, const Point2D& r);
// true, if the given point lies in the opposite halfspace from apex
bool is_point_in_halfpace (const Point2D& point,
                           const std::pair<Point2D, Point2D>& halfspace_line,
                           const Point2D& apex);

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

class parallel_2d_hull {
public:
    void lead_init (const vector<Point2D>& points);
    void distribute_input ();
    void compute_hull ();
    void collect_output (vector<Point2D>& destination);
    vector<Point2D> get_whole_set ();

    void print_local_set (const vector<Point2D>& data);

private:
    vector<Point2D> compute_local_samples ();
    void collect_all_samples (const vector<Point2D>& local_data);
    void communicate_interior_point ();
    void compute_enet (const vector<Point2D>& points);

    // distributes local hull points to corresponding buckets;
    // buckets distribution is trivial
    // bucket is a splitter polygon vertex. bucket defined by 2 half-planes
    // i-th processor in [0, p) is in charge 
    // of 2 consecutive buckets from [0, 2p): 2*i and 2*i+1.
    void distribute_over_buckets ();
    bool does_point_fall_into_bucket (const Point2D& point,
                                      vector<Point2D>::const_iterator bucket);
    void accumulate_buckets ();

    int id_;
    int processors_amount_;
    int subset_cardinality_;
    static const int halfplane_coefficient_;
    static const int buckets_per_proc_;

    vector <Point2D> whole_pointset_; // exists only on LEAD
    vector <Point2D> local_subset_;
    vector <Point2D> local_hull_;
    vector <Point2D> samples_set_; // exists only on LEAD
    vector <Point2D> splitters_;

    vector <Point2D> buckets_; // exists only on LEAD

    // required for checking bucket residence
    Point2D interior_point_;

    vector <Point2D> bucket_0;
    vector <Point2D> bucket_1;
};

struct triangle {
    triangle ();
    std::pair<Point2D, Point2D> edge;
    int weight;
};

vector<triangle> merge_light_triangles (const vector<triangle>& triangles,
                                        double heaviness_threshold);
vector<Point2D> extract_hull_points_from_tringles (const vector<triangle>& triangles_n_bins);

std::pair<Point2D, Point2D> generate_line_prev (const vector<Point2D>::const_iterator& bucket,
                                                const vector<Point2D>& splitters);
std::pair<Point2D, Point2D> generate_line_next (const vector<Point2D>::const_iterator& bucket,
                                                const vector<Point2D>& splitters);

// function picks one inside point,
// which is located furtherest from hull points (if prioritization is true)
Point2D_const_iterator find_interior_point (const vector<Point2D>& points,
                                            const vector<Point2D>& hull,
                                            bool prioritized = false);

const int PROCESSORS_AMOUNT_ = 4;
const int LEAD_PROCESSOR_ID_ = 0;

vector<Point2D> points;
vector<Point2D> convex_hull;
string app_name;

void compute_2d_hull_with_bsp ();

/////////////////////////////////////////////////////////////////////////////

const int parallel_2d_hull::halfplane_coefficient_ = 4;
const int parallel_2d_hull::buckets_per_proc_ = 2;

triangle::triangle ()
    : weight (0.0) 
    , edge (std::make_pair<Point2D, Point2D> ({-1,-1}, {-1,-1})) {
}

Point2D::Point2D ()
    : x (0.0)
    , y (0.0) {
}



Point2D::Point2D (double x, double y)
    : x (x)
    , y (y) {
}



bool Point2D::operator < (const Point2D& b) const {
    if (y != b.y)
        return y < b.y;
    return x < b.x;
}



bool Point2D::operator == (const Point2D& b) const {
    return std::fabs (x - b.x) < 1e-9 &&
           std::fabs (y - b.y) < 1e-9;
}



bool Point2D::operator != (const Point2D& b) const {
    return !(*this == b);
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
    cout.setf (std::ios_base::fixed, std::ios_base::floatfield);
    cout.precision (17);
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
    std::advance (bottommost,
                  std::distance<vector<Point2D>::const_iterator> (bottommost,
                                                                  bottommost_element));
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



orientation_status orientation (const Point2D& p,
                                const Point2D& q,
                                const Point2D& r) {
    double val = (q.x - p.x) * (r.y - p.y) -
        (r.x - p.x) * (q.y - p.y);

    if (0.0 == val) {
        return COLLINEAR;
    }

    return (val > 0.0) ? COUNTER_CLOCKWISE : CLOCKWISE;
}



bool is_point_in_halfpace (const Point2D& point,
                           const std::pair<Point2D, Point2D>& halfspace_line,
                           const Point2D& apex)
{
    bool with_apex = CLOCKWISE == orientation (halfspace_line.first,
                                               halfspace_line.second,
                                               apex);
    bool with_point = CLOCKWISE == orientation (halfspace_line.first,
                                                halfspace_line.second,
                                                point);
    return with_apex !=with_point;
}



bool is_point_invalid (const Point2D& p) {
    return p.x == -1.0 && p.y == -1.0;
}



bool is_point_inside_triangle (const Point2D& p,
                               const Point2D& a,
                               const Point2D& b,
                               const Point2D& c) {
    bool with_edge_ab, with_edge_bc, with_edge_ca;
    with_edge_ab = CLOCKWISE == orientation (p, a, b);
    with_edge_bc = CLOCKWISE == orientation (p, b, c);
    with_edge_ca = CLOCKWISE == orientation (p, c, a);
    return with_edge_ab == with_edge_bc &&
           with_edge_bc == with_edge_ca;
}

double dist (const Point2D& first, const Point2D& second) {
    return std::pow (first.x - second.x, 2.0) +
           std::pow (first.y - second.y, 2.0);
}



Point2D next_to_top (std::stack<Point2D>& S) {
    assert (S.size () >= 2);

    Point2D p = S.top ();
    S.pop ();
    Point2D res = S.top ();
    S.push (p);
    return res;
}



bool is_point_inside_polygon (const vector<Point2D>& polygon,
                              const Point2D& point) {
    int i, j, nvert = polygon.size ();
    bool c = false;
    for (i = 0, j = nvert - 1; i < nvert; j = i++)
    {   if (((polygon[i].y > point.y) != (polygon[j].y > point.y)) &&
            (point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x))
        {   c = !c;
        }
    }
    return c;
}



Point2D_const_iterator find_interior_point (const vector<Point2D>& points,
                                            const vector<Point2D>& hull,
                                            bool prioritized) {
    struct compare_points_less_penalty {
        bool operator () (const std::pair<Point2D, double>& first_point,
                          const std::pair<Point2D, double>& second_point)
        {   return first_point.second > second_point.second;
        }
    };

    auto generatePair = [] (const Point2D& p) -> std::pair<Point2D, double>
    {   return std::make_pair (p, dist (p, {0.0, 0.0}));
    };

    std::priority_queue<std::pair<Point2D, double>,
                        std::vector<std::pair<Point2D, double>>,
                        compare_points_less_penalty> pq;

    for (auto p = points.cbegin ();
         p != points.cend (); ++p)
    {   if (is_point_inside_polygon (hull, *p))
        {   if (prioritized)
            {   pq.emplace (generatePair (*p));
            } else
            {   return p;
            }
        }
    }

    if (pq.empty ())
    {   return points.cend ();
    } else
    {   return std::find (points.cbegin (), points.cend (), pq.top ().first);
    }
}



vector<Point2D> find_interior_points (vector<Point2D> points,
                                      const vector<Point2D>& hull,
                                      const Point2D& origin_point) {
    points.erase (std::remove (points.begin (), points.end (), origin_point));
    for (auto hull_point: hull)
    {   points.erase (std::remove (points.begin (),
                                   points.end (),
                                   hull_point));
    }

    vector<Point2D> interior_points;
    Point2D_const_iterator current_interior_point;
    {   current_interior_point = find_interior_point (points, hull);
        interior_points.emplace_back (*current_interior_point);
        points.erase (points.begin () +
            std::distance<Point2D_const_iterator> (points.cbegin (),
                                                   current_interior_point));
    } while (current_interior_point != points.cend ())
    return interior_points;
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
    for (int i = 0; i < 100; ++i)
    {   FreeImage_SetPixelColor (bitmap,
            std::floor (left.x + x_step * static_cast<double> (i)+0.5),
            std::floor (left.y + y_step * static_cast<double> (i)+0.5),
            &red_colour);
    }
}



string get_app_name (const string& full_path) {
    size_t pos = full_path.rfind("/");
    if (string::npos == pos)
    {   return "";
    } else
    {   return full_path.substr (pos + 1);
    }
}



void draw_hull (const string& path,
                const vector<Point2D>& points,
                const vector<Point2D>& hull,
                const Point2D& apex) {
    string final_path = app_name + "_" + path;
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
    {   auto prev_point = (point == hull.cbegin ())
                          ? std::prev (hull.cend ())
                          : std::prev (point);

        Point2D prev_frame_point = convert_to_frame_space (*prev_point);
        Point2D frame_point = convert_to_frame_space (*point);
        draw_line (bitmap, prev_frame_point, frame_point);
    }

    for (const Point2D& point : points)
    {   Point2D frame_point = convert_to_frame_space (point);
        RGBQUAD current_color (black_colour);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x + 1, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x - 1, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y - 1, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y + 1, &current_color);
    }

    if (!is_point_invalid (apex))
    {
        Point2D frame_point = convert_to_frame_space (apex);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y, &green_colour);
        FreeImage_SetPixelColor (bitmap, frame_point.x + 1, frame_point.y, &green_colour);
        FreeImage_SetPixelColor (bitmap, frame_point.x - 1, frame_point.y, &green_colour);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y - 1, &green_colour);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y + 1, &green_colour);
    }

    if (0 == FreeImage_Save (FIF_PNG, bitmap, final_path.c_str ()))
    {   cout << "Saving failed" << endl;
    }
    FreeImage_DeInitialise ();
}


void draw_subset (const string& path,
                  const vector<Point2D>& points,
                  const vector<Point2D>& subset,
                  const Point2D& apex,
                  const vector<std::pair<Point2D,Point2D>>& lines)
{
    const int width = 640;
    const int height = 480;

    std::function <bool (const Point2D&, const Point2D&)> compare_points_farther =
        [] (const Point2D& first, const Point2D& second) -> bool
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
        [scale, center_x, center_y](const Point2D& point) -> Point2D
    {   Point2D frame_point;
    frame_point.x = point.x * scale + center_x;
    frame_point.y = point.y * scale + center_y;
    return frame_point;
    };

    RGBQUAD green_colour;
    green_colour.rgbRed = 0;
    green_colour.rgbGreen = 255;
    green_colour.rgbBlue = 0;
    RGBQUAD red_colour;
    red_colour.rgbRed = 255;
    red_colour.rgbGreen = 0;
    red_colour.rgbBlue = 0;
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

    for (const Point2D& point : points)
    {   Point2D frame_point = convert_to_frame_space (point);
        RGBQUAD current_color (black_colour);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x + 1, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x - 1, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y - 1, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y + 1, &current_color);
    }

    if (!is_point_invalid (apex))
    {
        Point2D frame_point = convert_to_frame_space (apex);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y, &green_colour);
        FreeImage_SetPixelColor (bitmap, frame_point.x + 1, frame_point.y, &green_colour);
        FreeImage_SetPixelColor (bitmap, frame_point.x - 1, frame_point.y, &green_colour);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y - 1, &green_colour);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y + 1, &green_colour);
    }

    for (const Point2D& point : subset)
    {   Point2D frame_point = convert_to_frame_space (point);
        RGBQUAD current_color (red_colour);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x + 1, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x - 1, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y - 1, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y + 1, &current_color);
    }

    for (auto line : lines)
    {   if (!is_point_invalid (line.first) &&
            !is_point_invalid (line.second))
        {   draw_line (bitmap, 
                       convert_to_frame_space (line.first),
                       convert_to_frame_space (line.second));
        }
    }

    if (0 == FreeImage_Save (FIF_PNG, bitmap, path.c_str ()))
    {   cout << "Saving failed" << endl;
    }
    FreeImage_DeInitialise ();
}



void draw_enet_computation (const string& dst,
                            const vector<std::pair<Point2D,Point2D>>& lines,
                            const vector<Point2D>& points,
                            const Point2D& interior_point)
{
    const int width = 640;
    const int height = 480;

    std::function <bool (const Point2D&, const Point2D&)> compare_points_farther =
        [] (const Point2D& first, const Point2D& second) -> bool
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
    RGBQUAD red_colour;
    red_colour.rgbRed = 255;
    red_colour.rgbGreen = 0;
    red_colour.rgbBlue = 0;
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

    for (const Point2D& point : points)
    {   Point2D frame_point = convert_to_frame_space (point);
        RGBQUAD current_color (black_colour);

        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x + 1, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x - 1, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y - 1, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y + 1, &current_color);
    }

    assert (lines.size () > 0);
    for (const std::pair<Point2D,Point2D>& line : lines)
    {   Point2D src (convert_to_frame_space (line.first));
        Point2D dst (convert_to_frame_space (line.second));
        draw_line (bitmap, src, dst);
    }

    Point2D frame_interior_point = convert_to_frame_space (interior_point);
    FreeImage_SetPixelColor (bitmap, frame_interior_point.x, frame_interior_point.y, &green_colour);
    FreeImage_SetPixelColor (bitmap, frame_interior_point.x + 1, frame_interior_point.y, &green_colour);
    FreeImage_SetPixelColor (bitmap, frame_interior_point.x - 1, frame_interior_point.y, &green_colour);
    FreeImage_SetPixelColor (bitmap, frame_interior_point.x, frame_interior_point.y - 1, &green_colour);
    FreeImage_SetPixelColor (bitmap, frame_interior_point.x, frame_interior_point.y + 1, &green_colour);

    if (0 == FreeImage_Save (FIF_PNG, bitmap, dst.c_str ()))
    {   cout << "Saving failed" << endl;
    }
    FreeImage_DeInitialise ();
}



void parallel_2d_hull::lead_init (const vector<Point2D>& points) {
    if (LEAD_PROCESSOR_ID_ == bsp_pid ())
    {   whole_pointset_ = points;
        // possible to distribute uniformly
        assert (0 == whole_pointset_.size () % PROCESSORS_AMOUNT_);
        // input compliant with algorithm slackness
        assert (whole_pointset_.size () >= std::pow (PROCESSORS_AMOUNT_, 3));
        subset_cardinality_ = whole_pointset_.size () / PROCESSORS_AMOUNT_;
        LOG_LEAD ("subset_cardinality_ appears to be " << subset_cardinality_);
    }

    bsp_push_reg (&subset_cardinality_, sizeof (int));
    bsp_sync ();
    if (LEAD_PROCESSOR_ID_ == bsp_pid ())
    {   for (int i = 1; i < PROCESSORS_AMOUNT_; ++i)
        {   bsp_put (i, &subset_cardinality_, &subset_cardinality_, 0, sizeof (int));
        }
    }
    bsp_sync ();
    bsp_pop_reg (&subset_cardinality_);
    bsp_sync ();
}



void parallel_2d_hull::distribute_input () {
    // basic init
    id_ = bsp_pid ();
    processors_amount_ = bsp_nprocs ();
    LOG_ALL ("subset_cardinality_ is " << subset_cardinality_);
    local_subset_.resize (subset_cardinality_);
    bsp_sync();
    // required for successful read
    if (LEAD_PROCESSOR_ID_ != id_)
    {   whole_pointset_.resize (subset_cardinality_ * processors_amount_);
    }
    bsp_sync();
    bsp_push_reg (whole_pointset_.data (),
                  whole_pointset_.size () * sizeof (Point2D));

    bsp_sync ();
    bsp_get (LEAD_PROCESSOR_ID_,
             whole_pointset_.data (),
             subset_cardinality_ * id_ * sizeof (Point2D),
             local_subset_.data (),
             subset_cardinality_ * sizeof (Point2D));

    bsp_sync ();
    bsp_pop_reg (whole_pointset_.data ());
    bsp_sync ();
}



void parallel_2d_hull::compute_hull () {
    LOG_LEAD ("Computing samples...");
    bsp_sync ();
    vector<Point2D> local_samples = compute_local_samples ();
    bsp_sync ();
    LOG_LEAD ("Samples computed locally");
    bsp_sync ();
    collect_all_samples (local_samples);
    bsp_sync ();
    LOG_LEAD ("Samples collected");
    compute_enet (samples_set_);
    // bsp_sync ();
    // if (LEAD_PROCESSOR_ID_ == id_)
    // {   convex_hull = splitters_;
    // }
    bsp_sync ();
    LOG_LEAD ("bucket distribution started");
    distribute_over_buckets ();
    // simplified routine
    LOG_LEAD ("bucket distribution done");
    accumulate_buckets ();
}



void parallel_2d_hull::collect_output (vector<Point2D>& destination) {
    bsp_push_reg (local_subset_.data (), local_subset_.size () *sizeof (Point2D));
    bsp_sync ();
    if (LEAD_PROCESSOR_ID_ == id_)
    {   destination.resize (whole_pointset_.size ());
        for (int i = 0; i < PROCESSORS_AMOUNT_; ++i)
        {   bsp_get (i,
                     local_subset_.data (),
                     0,
                     destination.data () + local_subset_.size () * i,
                     local_subset_.size () * sizeof (Point2D));
        }
    }

    bsp_sync ();
    bsp_pop_reg (local_subset_.data ());
    bsp_sync ();
}

vector<Point2D> parallel_2d_hull::get_whole_set () {
    if (LEAD_PROCESSOR_ID_ != id_ || whole_pointset_.empty ())
    {   return vector<Point2D> ();
    }
    return whole_pointset_;
}



void parallel_2d_hull::print_local_set (const vector<Point2D>& data)
{   for (int i = 0; i < 5; ++i)
    {   for (int j = 0; j < processors_amount_; ++j)
        {
            if (j == id_)
            {   LOG_ALL (data.at (i));
            }
            bsp_sync ();
        }
    }
}



vector<Point2D> parallel_2d_hull::compute_local_samples () {
    local_hull_ = graham_scan (local_subset_);
    bsp_sync ();
    LOG_LEAD ("Graham scan done");
    bsp_sync ();
    if (local_hull_.size () <= processors_amount_)
    {   assert (local_hull_.size () == processors_amount_);
        // not implemented case when local_hull.size () < p
        return local_hull_;
    } else
    {   // samples from local hull with regular step such that card (samples) = p
        vector<Point2D> samples;
        samples.reserve (local_hull_.size ()); 
        double step = static_cast<double> (local_hull_.size () - 1) /
                      static_cast<double> (processors_amount_ - 1);
        for (int i = 0; i < processors_amount_; ++i)
        {   int index = std::floor (i * step + 0.5);
            assert (index < local_hull_.size ());
            samples.emplace_back (local_hull_.at (index));
        }
        return samples;
    }
}



void parallel_2d_hull::collect_all_samples (const vector<Point2D>& local_samples) {
    assert (local_samples.size () == processors_amount_);
    // allocate and register destination vector for all local samples
    samples_set_.resize (std::pow (processors_amount_, 2));
    bsp_push_reg (samples_set_.data (),
                  samples_set_.size () * sizeof (Point2D));
    bsp_sync ();

    bsp_put (LEAD_PROCESSOR_ID_,
             local_samples.data (),
             samples_set_.data (),
             id_ * local_samples.size () * sizeof (Point2D),
             local_samples.size () * sizeof (Point2D));
    bsp_sync ();

    bsp_pop_reg (samples_set_.data ());
    bsp_sync ();
    if (LEAD_PROCESSOR_ID_ == id_)
    {   LOG_LEAD ("Samples: ");
        for (auto p : samples_set_)
        {   LOG_LEAD (p);  
        }
    }
    bsp_sync ();
}



vector<triangle> merge_light_triangles (const vector<triangle>& triangles,
                                        double heaviness_threshold)
{
// #pragma message ("maybe buggish implementation, investigate later")
    vector<triangle> triangles_n_bins;
    triangle current_bin;
    for (const triangle& tri : triangles)
    {   if (current_bin.weight > heaviness_threshold)
        {   assert (current_bin.edge.first.x != 0.0 && current_bin.edge.first.y != 0.0);
            assert (current_bin.edge.second.x != 0.0 && current_bin.edge.second.y != 0.0);
            triangles_n_bins.push_back (current_bin);
            current_bin = triangle ();
        }
        if (tri.weight > heaviness_threshold)
        {   triangles_n_bins.push_back (tri);
        } else if (0 != current_bin.weight)
        {   current_bin.edge.second = tri.edge.second;
            current_bin.weight += tri.weight;
        } else
        {
            current_bin = tri;
        }
    }
    return triangles_n_bins;
}



vector<Point2D> extract_hull_points_from_tringles (const vector<triangle>& triangles_n_bins)
{   auto non_duplicating_push_back =
        [] (const Point2D& p, vector<Point2D>& points) -> void
    {   if (points.empty () ||
            points.back () != p)
        {   points.push_back (p);
        }
    };

    vector<Point2D> hull_points;
    for (auto tri : triangles_n_bins)
    {   non_duplicating_push_back (tri.edge.first, hull_points);
        non_duplicating_push_back (tri.edge.second, hull_points);
    }
    return hull_points;
}



void parallel_2d_hull::communicate_interior_point () {
    bsp_push_reg (&interior_point_, sizeof (Point2D));
    bsp_sync ();
    if (LEAD_PROCESSOR_ID_ != id_)
    {   bsp_get (LEAD_PROCESSOR_ID_,
                 &interior_point_,
                 0,
                 &interior_point_,
                 sizeof (Point2D));
    }
    bsp_sync ();
    bsp_pop_reg (&interior_point_);
    bsp_sync ();
}



void parallel_2d_hull::compute_enet (const vector<Point2D>& points) {
    bsp_sync ();
    splitters_.resize (2 * processors_amount_);
    // local computation on lead processor,
    // procedure from CS431, pp. 125
    if (LEAD_PROCESSOR_ID_ == id_)
    {   vector<Point2D> hull = graham_scan (points);
        auto interior_point_iter = find_interior_point (whole_pointset_, hull, true);
        if (interior_point_iter == points.cend ())
        {   LOG_LEAD ("No interior point");
            exit (1);
        } else
        {   interior_point_ = *interior_point_iter;
            LOG_LEAD ("Int point " << interior_point_);
        }
        draw_hull ("hull_from_samples.png", whole_pointset_, hull, interior_point_);

        // traverse triangles
        vector<Point2D> points_left = find_interior_points (whole_pointset_,
                                                            hull,
                                                            interior_point_);
        vector<std::pair<Point2D, Point2D>> lines;
        vector<triangle> triangles;
        for (auto hull_edge_start = hull.begin ();
             hull_edge_start != hull.end ();
             ++hull_edge_start)
        {   auto hull_edge_end = hull_edge_start != hull.end () - 1
                                 ? hull_edge_start + 1
                                 : hull.begin ();
            triangle tri;
            tri.weight = 3;
            tri.edge = std::make_pair (*hull_edge_start,
                                       *hull_edge_end);

            lines.push_back (tri.edge);
            lines.emplace_back (std::make_pair (*hull_edge_start,
                                                interior_point_));
            for (auto set_point = points_left.begin ();
                 set_point != points_left.end ();)
            {   if (is_point_inside_triangle (*set_point,
                                              interior_point_,
                                              *hull_edge_start,
                                              *hull_edge_end))
                {   ++tri.weight;
                    set_point = points_left.erase (set_point);
                } else {
                    ++set_point;
                }
            }
            triangles.push_back (tri);
        }
        double heaviness_threshold = static_cast<double> (points.size ()) /
                                     static_cast<double> (processors_amount_);
        vector<triangle> triangles_n_bins = merge_light_triangles (triangles,
                                                                   heaviness_threshold);
        /////
        // vector <Point2D> merged;
        // for (auto tri : triangles_n_bins)
        // {   merged.push_back (tri.edge.first);
        //     merged.push_back (tri.edge.second);
        // }
        // draw_hull ("pre_enet.png", points, hull);
        // LOG_LEAD ("pre enet size " << hull.size ());
        /////

        splitters_ = extract_hull_points_from_tringles (triangles_n_bins);
        draw_hull ("splitters.png", whole_pointset_, splitters_);
        splitters_.resize (2 * processors_amount_, {-1,-1});
        draw_enet_computation (app_name + "_" + "tri.png",
                               lines,
                               whole_pointset_,
                               interior_point_);
        // draw_subset ("enet.png", whole_pointset_, splitters_);
    }
    bsp_sync ();
    communicate_interior_point ();
    // communicate splitters
    bsp_push_reg (splitters_.data (), 2 * processors_amount_ * sizeof (Point2D));
    bsp_sync ();
    if (LEAD_PROCESSOR_ID_ != id_)
    {   bsp_get (LEAD_PROCESSOR_ID_,
                 splitters_.data (),
                 0,
                 splitters_.data (),
                 2 * processors_amount_ * sizeof (Point2D));
    }
    bsp_sync ();
    bsp_pop_reg (splitters_.data ());
    splitters_.erase (std::remove_if (splitters_.begin (),
                                      splitters_.end (),
                                      is_point_invalid),
                      splitters_.end ());
    LOG_ALL ("Splitters instance size is " << splitters_.size ());
    bsp_sync ();
}



void parallel_2d_hull::distribute_over_buckets () {
    int bucket_size = subset_cardinality_ * halfplane_coefficient_;
    bucket_0.resize (bucket_size * processors_amount_, {-1,-1});
    bucket_1.resize (bucket_size * processors_amount_, {-1,-1});

    // iterate over local hull and send every point to the appropriate bucket
    vector<vector <Point2D>> bucket_entries;
    bucket_entries.resize (buckets_per_proc_ *
                           processors_amount_);
    for (const Point2D& local_hull_point : local_hull_)
    {   for (auto bucket_iter = splitters_.cbegin();
             bucket_iter != splitters_.cend ();
             ++bucket_iter)
        {   if (does_point_fall_into_bucket (local_hull_point, bucket_iter))
            {   int bucket_index =
                    std::distance<Point2D_const_iterator> (splitters_.cbegin (),
                                                           bucket_iter);
                bucket_entries.at (bucket_index).emplace_back (local_hull_point);
                break;
            }
        }
    }
    bsp_push_reg (bucket_0.data (), bucket_0.size () * sizeof (Point2D));
    bsp_push_reg (bucket_1.data (), bucket_1.size () * sizeof (Point2D));
    bsp_sync ();
    for (int bucket_index = 0;
         bucket_index < bucket_entries.size ();
         ++bucket_index)
    {   if (bucket_entries.at (bucket_index).empty ())
        {   continue;
        }
        int designated_proc = bucket_index / 2;
        void* destination = (0 == bucket_index % 2)
                            ? bucket_0.data ()
                            : bucket_1.data ();
        bsp_put (designated_proc,
                 bucket_entries.at (bucket_index).data (),
                 destination,
                 bucket_size * id_,
                 bucket_entries.at (bucket_index).size () * sizeof (Point2D));
    }
    bsp_pop_reg (bucket_0.data ());
    bsp_pop_reg (bucket_1.data ());
    bsp_sync ();
    ///////////////
    ///////////////
    // bucket_0.erase (std::remove_if (bucket_0.begin (), bucket_0.end (), is_point_invalid), bucket_0.end ());
    // bucket_1.erase (std::remove_if (bucket_1.begin (), bucket_1.end (), is_point_invalid), bucket_1.end ());
    // bsp_sync ();
    ///////////////
    //bsp_push_reg (whole_pointset_.data (), whole_pointset_.size () * sizeof (Point2D));
    // bsp_sync ();
    // if (1 ==id_)
    // {   bsp_get (LEAD_PROCESSOR_ID_,
    //              whole_pointset_.data (),
    //              0,
    //              whole_pointset_.data (),
    //              whole_pointset_.size () * sizeof (Point2D));
    // }
    // 
    // bsp_sync ();
    // bsp_pop_reg (whole_pointset_.data ());
    // bsp_sync ();
    /////////////
    // for (int i = 0; i < processors_amount_; ++i)
    // {   if (i == id_)
    //     {   if (0 == i)
    //         {   vector<std::pair<Point2D, Point2D>> lines =
    //             {generate_line_prev(splitters_.cbegin (), splitters_),
    //              generate_line_next(splitters_.cbegin (), splitters_)};
    //             draw_subset ("bucket_0.png", whole_pointset_, bucket_0,
    //                          interior_point_, lines);
    //         }
    //         assert (bucket_0.size () <= bucket_size);
    //         assert (bucket_1.size () <= bucket_size);
    //         LOG_ALL ("bucket 0");
    //         for (auto p : bucket_0)
    //         {   if (!is_point_invalid (p)) LOG_ALL (p);
    //         }
    //         LOG_ALL ("bucket 1");
    //         for (auto p : bucket_1)
    //         {   if (!is_point_invalid (p)) LOG_ALL (p);
    //         }
    //     }
    //     bsp_sync ();
    // }
}



std::pair<Point2D, Point2D> generate_line_prev (const vector<Point2D>::const_iterator& bucket,
                                           const vector<Point2D>& splitters)
{   if (bucket == splitters.cbegin ())
    {   return std::make_pair<Point2D, Point2D> (static_cast<Point2D> (splitters.back ()),
                                                 static_cast<Point2D> (*bucket));
    } else
    {   return std::make_pair <Point2D, Point2D> (static_cast<Point2D>(*std::prev (bucket)),
                                                  static_cast<Point2D>(*bucket));
    }
}



std::pair<Point2D, Point2D> generate_line_next (const vector<Point2D>::const_iterator& bucket,
                                                const vector<Point2D>& splitters)
{   if (bucket == splitters.cend () - 1)
    {    return std::make_pair<Point2D, Point2D> (static_cast<Point2D> (*bucket),
                                                  static_cast<Point2D> (splitters.front ()));
    } else
    {   return std::make_pair <Point2D, Point2D> (static_cast<Point2D>(*bucket),
                                                  static_cast<Point2D>(*std::next (bucket)));
    }
}



bool parallel_2d_hull::does_point_fall_into_bucket (const Point2D& point,
                                                    vector<Point2D>::const_iterator bucket) {
    auto first_line = generate_line_prev (bucket, splitters_);
    auto second_line = generate_line_next (bucket, splitters_);

    bool is_in_first_halfspace = is_point_in_halfpace (point, first_line, interior_point_);
    bool is_in_second_halfspace = is_point_in_halfpace (point, second_line, interior_point_);
    return is_in_first_halfspace || is_in_second_halfspace;
}



void parallel_2d_hull::accumulate_buckets () {
    int bucket_size = bucket_0.size ();
    bsp_push_reg (bucket_0.data (), bucket_size * sizeof (Point2D));
    bsp_push_reg (bucket_1.data (), bucket_size * sizeof (Point2D));
    bsp_sync ();
    if (LEAD_PROCESSOR_ID_ == id_)
    {   buckets_.resize (bucket_0.size () *
                         processors_amount_ *
                         buckets_per_proc_ *
                         sizeof (Point2D),
                         {-1, -1});
        for (int i = 0; i < processors_amount_; ++i)
        {   bsp_get (i,
                     bucket_0.data (),
                     0,
                     buckets_.data () + 2 * i * bucket_size * sizeof (Point2D),
                     bucket_size * sizeof (Point2D));
            bsp_get (i,
                bucket_1.data (),
                0,
                buckets_.data () + (2 * i + 1) * bucket_size * sizeof (Point2D),
                bucket_size * sizeof (Point2D));
        }
    }
    bsp_sync ();
    bsp_pop_reg (bucket_0.data ());
    bsp_pop_reg (bucket_1.data ());
    bsp_sync ();
    if (LEAD_PROCESSOR_ID_ == id_)
    {   buckets_.erase (std::remove_if (buckets_.begin (),
                                        buckets_.end (),
                                        is_point_invalid),
                        buckets_.end ());
        std::copy (splitters_.cbegin (),
                   splitters_.cend (),
                   std::back_inserter (buckets_));
        // draw_subset ("buckets.png", whole_pointset_, buckets_);
        LOG_LEAD ("last graham_scan started " << buckets_.size ());
        convex_hull = graham_scan (buckets_);
        LOG_LEAD ("last graham_scan done");
        draw_hull ("final_hull.png", whole_pointset_, convex_hull);
    }
    bsp_sync ();
}



void compute_2d_hull_with_bsp () {
    bsp_begin (PROCESSORS_AMOUNT_);
    LOG_LEAD ("begin");
    double time_start = bsp_time();
    parallel_2d_hull computer;
    computer.lead_init (points);
    computer.distribute_input ();
    LOG_LEAD ("Input was distributed" << endl << "Computing ... ");
    bsp_sync ();
    computer.compute_hull ();
    double time_end = bsp_time();
    if (0 == bsp_pid ())
    {   cout << "Time = " << time_end - time_start;
    }
    bsp_sync ();
    bsp_end ();
}



int main (int argc, char ** argv) {
    std::ios_base::sync_with_stdio (false);
    cin.tie (nullptr);
    app_name = get_app_name (argv[0]);

    points = read_points ();
    // draw_subset ("plain.points.png", points, vector<Point2D> ());
    bsp_init (compute_2d_hull_with_bsp, argc, argv);
    compute_2d_hull_with_bsp ();
    // write_points (convex_hull);
    return 0;
}
