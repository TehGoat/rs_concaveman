use core::slice;

use location_trait::LocationTrait;
use point_in_polygon::point_in_polygon;
use robust_predicates::orient2d;

extern crate libc;

extern "C" {
    fn rust_concaveman_2d(
        points_c: *const libc::c_double,
        num_points: libc::size_t,
        hull_points_c: *const libc::c_int,
        num_hull_points: libc::size_t,
        concavity: libc::c_double,
        lengthThreshold: libc::c_double,
        concave_points_c: *mut *mut libc::c_double,
        num_concave_points: *mut libc::size_t,
    );

    fn free_points(p_concave_points_c: *mut *mut libc::c_double);
}

pub mod location_trait;
mod point_in_polygon;

pub fn concaveman<T>(
    points: &[T],
    concavity: Option<f64>,
    length_threshold: Option<f64>,
) -> Vec<(f64, f64)>
where
    T: LocationTrait,
{
    let concavity = concavity.unwrap_or(2.0).max(0.0);

    let length_threshold = length_threshold.unwrap_or(0.0);

    let hull: Vec<i32> = fast_convex_hull(points)
        .iter()
        .map(|index| *index as i32)
        .collect();

    let converted_points: Vec<f64> = points
        .iter()
        .flat_map(|point| [point.get_x(), point.get_y()])
        .collect();

    let mut result: *mut libc::c_double = std::ptr::null_mut();
    let mut result_size: usize = 0;

    let mut return_value: Vec<(f64, f64)> = Vec::new();

    unsafe {
        rust_concaveman_2d(
            converted_points.as_ptr(),
            points.len(),
            hull.as_ptr(),
            hull.len(),
            concavity,
            length_threshold,
            &mut result,
            &mut result_size,
        );

        let converted_result = slice::from_raw_parts(result, result_size * 2);

        for i in (0..result_size * 2).step_by(2) {
            return_value.push((converted_result[i], converted_result[i + 1]));
        }

        free_points(&mut result);
    }

    return_value
}

pub fn concaveman_convex<T>(points: &[T]) -> Vec<(f64,f64)>
where
    T: LocationTrait
{
    let hull = fast_convex_hull(points);

    hull
    .iter()
    .map(|index| {
        let current_point = &points[*index];

        (current_point.get_x(), current_point.get_y())
    })
    .collect()
}

fn fast_convex_hull<T>(points: &[T]) -> Vec<usize>
where
    T: LocationTrait,
{
    let mut left = (&points[0], 0);
    let mut right = (&points[0], 0);
    let mut top = (&points[0], 0);
    let mut bottom = (&points[0], 0);

    for i in 1..points.len() {
        let current = &points[i];

        if current.get_x() < left.0.get_x() {
            left = (current, i);
        }
        if current.get_x() > right.0.get_x() {
            right = (current, i);
        }
        if current.get_y() < top.0.get_y() {
            top = (current, i);
        }
        if current.get_y() > bottom.0.get_y() {
            bottom = (current, i);
        }
    }

    let cull = vec![left.0, top.0, right.0, bottom.0];
    let mut filtered = vec![left.1, top.1, right.1, bottom.1];

    for i in 0..points.len() {
        if !point_in_polygon(&points[i], &cull) {
            filtered.push(i);
        }
    }

    convex_hull(points, filtered)
}

fn convex_hull<T>(points: &[T], mut filtered: Vec<usize>) -> Vec<usize>
where
    T: LocationTrait,
{
    filtered.sort_by(|first, second| {
        if points[*first].get_x() == points[*second].get_x() {
            points[*first]
                .get_y()
                .partial_cmp(&points[*second].get_y())
                .unwrap()
        } else {
            points[*first]
                .get_x()
                .partial_cmp(&points[*second].get_x())
                .unwrap()
        }
    });

    let mut lower = Vec::new();

    for i in 0..filtered.len() {
        while lower.len() >= 2
            && cross(
                &points[lower[lower.len() - 2]],
                &points[lower[lower.len() - 1]],
                &points[filtered[i]],
            ) <= 0.0
        {
            lower.pop();
        }

        lower.push(filtered[i]);
    }

    let mut upper = Vec::new();

    for i in (0..filtered.len()).rev() {
        while upper.len() >= 2
            && cross(
                &points[upper[upper.len() - 2]],
                &points[upper[upper.len() - 1]],
                &points[filtered[i]],
            ) <= 0.0
        {
            upper.pop();
        }

        upper.push(filtered[i]);
    }

    upper.pop();
    lower.pop();

    lower.append(&mut upper);

    lower
}

fn cross<T>(a: &T, b: &T, c: &T) -> f64
where
    T: LocationTrait,
{
    orient2d(
        &[a.get_x(), a.get_y()],
        &[b.get_x(), b.get_y()],
        &[c.get_x(), c.get_y()],
    ) * -1.0
}

#[cfg(test)]
mod tests {
    use crate::{concaveman, location_trait::LocationTrait};

    impl LocationTrait for [f64; 2] {
        fn get_x(&self) -> f64 {
            self[0]
        }

        fn get_y(&self) -> f64 {
            self[1]
        }
    }

    #[test]
    fn it_works() {
        let raw_points = vec![
            [57.71513055, 12.05632745],
            [57.71737515, 12.05644505],
            [57.7170553, 12.0565587],
            [57.7145244, 12.0569446],
            [57.71422395, 12.0577961],
            [57.7147534, 12.0567111],
            [57.7161877, 12.0589052],
            [57.7164567, 12.0555691],
            [57.7166576, 12.05637745],
            [57.716829, 12.0623367],
            [57.71542125, 12.05983265],
            [57.7139291, 12.0635653],
            [57.71297825, 12.0652677],
            [57.71296595, 12.06127885],
            [57.7166984, 12.0618155],
            [57.71351655, 12.0595376],
            [57.71589575, 12.0614224],
            [57.71421055, 12.0617384],
            [57.7145078, 12.061162],
            [57.7137711, 12.0591028],
            [57.71602575, 12.06129975],
            [57.7132035, 12.0608974],
            [57.71365195, 12.0594795],
            [57.7144841, 12.06088985],
            [57.71591725, 12.0616367],
            [57.7131496, 12.0607028],
            [57.71417225, 12.0618572],
            [57.7160174, 12.0610329],
            [57.71716555, 12.06179935],
            [57.71466495, 12.0603288],
            [57.71470755, 12.0605464],
            [57.71489735, 12.05995995],
            [57.7171412, 12.0633773],
            [57.71531005, 12.06380305],
            [57.7160061, 12.0644319],
            [57.7147262, 12.0640641],
            [57.71425215, 12.06506175],
            [57.7162699, 12.0657061],
            [57.71294775, 12.06515875],
            [57.717121, 12.0629665],
            [57.7172253, 12.0632853],
            [57.71315875, 12.0610388],
            [57.71474755, 12.06452435],
            [57.7135687, 12.0652226],
            [57.7156099, 12.06359915],
            [57.7143989, 12.0644667],
            [57.7155817, 12.0527455],
            [57.7157362, 12.05324595],
            [57.71382225, 12.0653538],
            [57.7147134, 12.06017845],
            [57.71443605, 12.06103885],
            [57.7160007, 12.0602355],
            [57.7164744, 12.06123795],
            [57.71698685, 12.0628726],
            [57.7130834, 12.0583383],
            [57.71438305, 12.0592018],
            [57.71347405, 12.06004185],
            [57.71333055, 12.0601283],
            [57.71335755, 12.0604102],
            [57.7131001, 12.0612242],
            [57.7143805, 12.0612112],
            [57.7134272, 12.06019],
            [57.71596375, 12.0545872],
            [57.7137588, 12.05443335],
            [57.71353745, 12.0514873],
            [57.71452735, 12.054291],
            [57.7148392, 12.054066],
            [57.714971, 12.0537492],
            [57.71296555, 12.0530501],
            [57.7129739, 12.0531103],
            [57.7152057, 12.0521583],
            [57.7150849, 12.0514916],
            [57.7141902, 12.05141695],
            [57.71294775, 12.06515875],
            [57.7160777, 12.051785],
            [57.71294775, 12.06515875],
            [57.71580005, 12.0617041],
            [57.71401085, 12.06417725],
            [57.7156847, 12.06315145],
            [57.71360875, 12.059235],
            [57.715916, 12.05949405],
            [57.7154159, 12.0526459],
            [57.7129146, 12.0525727],
            [57.7179336, 12.0553003],
            [57.7146721, 12.0567938],
            [57.71523305, 12.0552346],
            [57.7171983, 12.0556876],
            [57.71589175, 12.05323985],
            [57.71693255, 12.05623975],
            [57.71677955, 12.05584365],
            [57.71556275, 12.05151365],
        ];

        concaveman(&raw_points, None, None);
    }
}
