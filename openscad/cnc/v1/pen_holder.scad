include <cnc_params.scad>
use <z_sliding_block.scad>

$fn = 50;

holder_sz_x = slider_sz_x-rod_bearing_d-attach_sz_x-2*attach_side_gap;
holder_sz_y = holder_sz_x;
holder_sz_z = 5;

pen_holder_z = 20;

pen_d = 9;
pen_wall = 4;

pen_stopper_wall = 1.2;
pen_holder_stopper_z = 10;

module pen_holder() {
	slider_z_attachment(extra_sz_y=0);
	translate([0, (-(slider_sz_y+attach_sz_y)/2-attach_side_gap)/2-attach_sz_y/2-holder_sz_y/2, -(attach_sz_z-holder_sz_z)/2]) {
		difference() {
			union() {
				cube([holder_sz_x, holder_sz_y/2, holder_sz_z], center=true);
				translate([0, -holder_sz_y/4, 0]) {
					cylinder(d=holder_sz_x, h=holder_sz_z, center=true);
					translate([0, 0, pen_holder_z/2]) {
						cylinder(d=pen_d+2*pen_wall , h=pen_holder_z, center=true);
//						cube([pen_d+2*pen_wall+20, 5, pen_holder_z], center=true);
					}
					translate([0, 0, pen_holder_stopper_z/2]) {
						cylinder(d=pen_d+2*pen_wall+2*pen_stopper_wall , h=pen_holder_stopper_z, center=true);
					}
				}
			}
			translate([0, -holder_sz_y/4, (pen_holder_z)/2]) {
				cylinder(d=pen_d, h=pen_holder_z*2, center=true);
        translate([0, 0, holder_sz_z]) {
          cube([pen_d+2*pen_wall+20, 2, pen_holder_z], center=true);
        }
			}
		}
	}
}

pen_holder();
