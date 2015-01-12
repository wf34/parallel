
#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <iterator>
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
    Point2D ();Point2D (double x, double y);
    bool operator < (const Point2D& b);

    double x;
    double y;
};
std::ostream& operator<< (std::ostream& stream, const Point2D& point);

vector<Point2D> read_points (std::istream& stream = cin);
void write_points (const vector<Point2D>& points, std::ostream& stream = cout);
vector<Point2D> graham_scan (vector<Point2D> points);
// triangles association (by CS341, pp.125)
vector<Point2D> compute_enet (const vector<Point2D>& points);

void draw_hull (const vector<Point2D>& points, const vector<Point2D>& hull);
void draw_subset (const string& path, const vector<Point2D>& points, const vector<Point2D>& subset);
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
    void compute_enet (const vector<Point2D>& points);
    void collect_all_samples (const vector<Point2D>& local_data);

    int id_;
    int processors_amount_;
    int subset_cardinality_;

    vector <Point2D> whole_pointset_; // exists only on LEAD
    vector <Point2D> local_subset_;
    vector <Point2D> samples_set_; // exists only on LEAD
    vector <Point2D> splitters_; // exists only on LEAD
};

struct triangle {
    triangle ();
    std::pair<Point2D, Point2D> edge;
    int weight;
};

vector<triangle> merge_light_triangles (const vector<triangle>& triangles,
                                        double heaviness_threshold);
vector<Point2D> produce_enet (const vector<triangle>& triangles_n_bins);

const int PROCESSORS_AMOUNT_ = 4;
const int LEAD_PROCESSOR_ID_ = 0;

vector<Point2D> points;
vector<Point2D> convex_hull;

void compute_2d_hull_with_bsp ();

/////////////////////////////////////////////////////////////////////////////

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


bool is_point_inside (const Point2D& p,
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


Point2D find_interior_point (const vector<Point2D>& points,
                             const vector<Point2D>& hull) {
    std::set<Point2D, compare_points_closer_to_leftmost> hull_set;
    std::copy (hull.cbegin (), hull.cend (), std::inserter (hull_set, hull_set.end ()));
    assert (hull_set.size () > 0);
    for (const Point2D& p : points)
    {   if (hull_set.end () == hull_set.find (p))
        {   return p;
        }
    }
    return {-1, -1};
}



vector<Point2D> find_interior_points (const vector<Point2D>& points,
                                      const vector<Point2D>& hull,
                                      const Point2D& origin_point) {
    std::set<Point2D, compare_points_closer_to_leftmost> hull_set;
    std::copy (hull.cbegin (), hull.cend (), std::inserter (hull_set, hull_set.end ()));
    hull_set.emplace (origin_point);
    vector<Point2D> interior_points;
    for (const Point2D& p : points)
    {   if (hull_set.end () == hull_set.find (p))
        {   interior_points.emplace_back (p);
        }
    }
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


void draw_subset (const string& path,
                  const vector<Point2D>& points,
                  const vector<Point2D>& subset)
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

    for (const Point2D& point : subset)
    {   Point2D frame_point = convert_to_frame_space (point);
        RGBQUAD current_color (red_colour);

        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x + 1, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x - 1, frame_point.y, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y - 1, &current_color);
        FreeImage_SetPixelColor (bitmap, frame_point.x, frame_point.y + 1, &current_color);
    }

    if (0 == FreeImage_Save (FIF_PNG, bitmap, path.c_str ()))
    {   cout << "Saving failed" << endl;
    }
    FreeImage_DeInitialise ();
}



void draw_enet_computation (string dst, vector<std::pair<Point2D,Point2D>> lines, vector<Point2D> points)
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
    bsp_sync ();
    if (LEAD_PROCESSOR_ID_ == id_)
    {   convex_hull = splitters_;
    }
    bsp_sync ();
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
    vector<Point2D> local_hull = graham_scan (local_subset_);
    bsp_sync ();
    LOG_LEAD ("Graham scan done");
    bsp_sync ();
    if (local_hull.size () <= processors_amount_)
    {   assert (local_hull.size () == processors_amount_);
        // not implemented case when local_hull.size () < p
        return local_hull;
    } else
    {   // samples from local hull with regular step such that card (samples) = p
        vector<Point2D> samples;
        samples.reserve (local_hull.size ()); 
        double step = static_cast<double> (local_hull.size () - 1) /
                      static_cast<double> (processors_amount_ - 1);
        for (int i = 0; i < processors_amount_; ++i)
        {   int index = std::floor (i * step + 0.5);
            assert (index < local_hull.size ());
            samples.emplace_back (local_hull.at (index));
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
}



vector<triangle> merge_light_triangles (const vector<triangle>& triangles,
                                        double heaviness_threshold)
{   vector<triangle> triangles_n_bins;
    triangle current_bin;
    for (const triangle& tri : triangles)
    {   if (current_bin.weight > heaviness_threshold)
        {   // assert (current_bin.edge.first.x != 0.0 && current_bin.edge.first.y != 0.0);
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



vector<Point2D> produce_enet (const vector<triangle>& triangles_n_bins)
{   std::set<Point2D, compare_points_closer_to_leftmost> enet_set;
    for (const triangle& tri : triangles_n_bins)
    {   enet_set.emplace (tri.edge.first);
        enet_set.emplace (tri.edge.second);
    }
    vector<Point2D> enet;
    std::copy (enet_set.cbegin (), enet_set.cend (), std::back_inserter(enet));
    return enet;
}



void parallel_2d_hull::compute_enet (const vector<Point2D>& points) {
    bsp_sync ();
    if (LEAD_PROCESSOR_ID_ == id_)
    {   vector<Point2D> hull = graham_scan (points);
        Point2D interior_point = find_interior_point (points, hull);
        if (interior_point.x == -1 &&
            interior_point.y == -1)
        {   LOG_LEAD ("No interior point");
            exit (1);
        }
        // traverse triangles
        vector<Point2D> points_left = find_interior_points (points, hull, interior_point);
        vector<std::pair<Point2D, Point2D>> lines;
        vector<triangle> triangles;
        for (auto hull_edge_start = hull.begin ();
             hull_edge_start != hull.end ();
             ++hull_edge_start)
        {   auto hull_edge_end = hull_edge_start != hull.end () - 1
                                 ? hull_edge_start + 1
                                 : hull.begin ();
            triangle tri;
            tri.edge = std::make_pair (*hull_edge_start,
                                       *hull_edge_end);
            tri.weight = 3;

            lines.push_back (tri.edge);
            lines.emplace_back (std::make_pair (*hull_edge_start,
                                                interior_point));
            for (auto set_point = points_left.begin (); set_point != points_left.end ();)
            {   if (is_point_inside (*set_point,
                                     interior_point,
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
        splitters_ = produce_enet (triangles_n_bins);
        // draw_enet_computation ("tri.png", lines, points);
        draw_subset ("enet.png", whole_pointset_, splitters_);
        LOG_LEAD ("enet size " << splitters_.size ());
    }
    bsp_sync ();
}



void compute_2d_hull_with_bsp () {
    bsp_begin (PROCESSORS_AMOUNT_);
    LOG_LEAD ("begin");
    parallel_2d_hull computer;
    computer.lead_init (points);
    computer.distribute_input ();
    LOG_LEAD ("Input was distributed" << endl << "Computing ... ");
    bsp_sync ();
    computer.compute_hull ();
    // debug request computer.print_local_set ();
    // bsp_sync ();
    // LOG_LEAD ("Computing ... done");
    // bsp_sync ();
    // computer.collect_output (convex_hull);
    bsp_sync ();
    bsp_end ();
}

int main (int argc, char ** argv) {
    points = read_points ();
    bsp_init( compute_2d_hull_with_bsp, argc, argv );
    compute_2d_hull_with_bsp ();
    write_points (convex_hull);
    return 0;
}