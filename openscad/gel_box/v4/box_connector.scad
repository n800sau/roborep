$fn = 20;

overlap = 10;

wall = 2;
gap = 0.5;

meter_sz_x = 63;
meter_sz_y = 54;
meter_sz_z = 58;
meter_d = 54;
meter_hole_dist_x = 55;
meter_hole_dist_z = 55;

meter_box_sz_x = meter_sz_x + 2*wall + gap + overlap;
meter_box_sz_y = meter_sz_y + 2*wall + 2*gap;
meter_box_sz_z = meter_sz_z + 2*wall + 2*gap;

module left_box_fwd() {
	difference() {
		cube([meter_box_sz_x, meter_box_sz_y, meter_box_sz_z], center= true);
		translate([wall, wall, wall]) {
			cube([meter_box_sz_x-wall, meter_box_sz_y-wall, meter_box_sz_z-wall], center= true);
		}
	}
}

module right_box_fwd() {
	difference() {
		union() {
			cube([meter_box_sz_x, meter_box_sz_y, meter_box_sz_z], center= true);
			translate([-overlap/2, wall, wall]) {
				cube([meter_box_sz_x+overlap, meter_box_sz_y-2*wall-2*gap, meter_box_sz_z-wall], center= true);
			}
		}
		translate([-wall+overlap/2, wall, wall]) {
			cube([meter_box_sz_x-wall-overlap/2-gap, meter_box_sz_y-wall, meter_box_sz_z-wall], center= true);
		}
		translate([-wall-overlap/2-gap, wall, wall]) {
			cube([meter_box_sz_x-wall+overlap+gap, meter_box_sz_y-wall, meter_box_sz_z-wall], center= true);
		}
	}
}

translate([meter_box_sz_x/2+10, 0, 0]) {
	color("blue") {
		right_box_fwd();
	}
}
translate([-meter_box_sz_x/2-10, 0, 0]) {
	left_box_fwd();
}

