use crate::location_trait::LocationTrait;


pub fn point_in_polygon<T>(point: &T, cull: &[&T]) -> bool
where
    T: LocationTrait
{
    let mut inside = false;

    let mut j = cull.len() - 1;

    for i in 0..cull.len() {
        let xi = cull[i].get_x();
        let xj = cull[j].get_x();
        let yi = cull[i].get_y();
        let yj = cull[j].get_y();

        let y = point.get_y();
        let x = point.get_x();

        if (yi > y) != (yj > y) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
            inside = !inside;
        }

        j = i;
    }

    inside
}