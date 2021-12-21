#if 0
g++ -std=c++11 -shared concaveman.cpp -o libconcaveman.so
exit 0
#endif

//
// Author: Stanislaw Adaszewski, 2019
//

#include "concaveman.h"

extern "C" {
    void rust_concaveman_2d(double *points_c, size_t num_points,
        int *hull_points_c, size_t num_hull_points,
        double concavity, double lengthThreshold,
        double **concave_points_c, size_t *num_concave_points);

    void free_points(double **p_concave_points_c);
}

void rust_concaveman_2d(double *points_c, size_t num_points,
    int *hull_points_c, size_t num_hull_points,
    double concavity, double lengthThreshold,
    double **p_concave_points_c,
    size_t *p_num_concave_points) {

    typedef double T;
    typedef std::array<T, 2> point_type;

    std::vector<point_type> points(num_points);
    for (size_t i = 0; i < num_points; i++) {
        points[i] = { points_c[i << 1], points_c[(i << 1) + 1] };
    }

    std::vector<int> hull(num_hull_points);
    for (size_t i = 0; i < num_hull_points; i++) {
        hull[i] = hull_points_c[i];
    }

    auto concave_points = concaveman<T, 16>(points, hull, concavity, lengthThreshold);

    double *concave_points_c = *p_concave_points_c = (double*) malloc(sizeof(double) * 2 * concave_points.size());
    for (size_t i = 0; i < concave_points.size(); i++) {
        concave_points_c[i << 1] = concave_points[i][0];
        concave_points_c[(i << 1) + 1] = concave_points[i][1];
    }

    *p_num_concave_points = concave_points.size();
}

void free_points(double **p_concave_points_c) {
    if (p_concave_points_c == NULL)  {
        return;
    }

    free(*p_concave_points_c);

}
