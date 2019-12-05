include <cnc_params.scad>
use <gantry_z.scad>

dc_motor_bottom_d = 45.5;
dc_motor_bearing_d = 17.5;
dc_motor_shaft_d = 5;
dc_motor_body_sz_z = 66.4;
dc_motor_bearing_sz_z = 4; //?
dc_motor_shaft_sz_z = 15; //?
dc_motor_bottom_holes_dist = 29;
dc_motor_bottom_hole_d = 3.3;

module dc_motor_mockup() {
	cylinder(d=dc_motor_bottom_d, h=dc_motor_body_sz_z);
	translate([0, 0, -dc_motor_bearing_sz_z]) {
		cylinder(d=dc_motor_bearing_d, h=dc_motor_bearing_sz_z);
		translate([0, 0, -dc_motor_shaft_sz_z]) {
			cylinder(d=dc_motor_shaft_d, h=dc_motor_shaft_sz_z);
		}
	}
}

module dc_motor_holder() {

}

dc_motor_holder();
%dc_motor_mockup();


