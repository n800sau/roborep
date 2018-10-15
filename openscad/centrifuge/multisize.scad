base_d = 85;
base_h = 15;

box_len = base_d;
box_width = 20;

rotor_d = 30;
rotor_h = 3;

hole_dist = 21;
hole_d = 2.5;

module centry() {
	difference() {
		// base body
		cylinder(d = base_d, h = base_h, $fn = 280);

		translate([0, 0, rotor_h])
			cylinder(d = rotor_d, h = base_h, $fn = 280);

		for(i=[0: 6]) {
			// Rotor mount holes
			rotate([0, 0, i*360/6]) {
				translate([0, hole_dist/2, -0.1])
					cylinder(d = hole_d, h = 6.1, $fn = 60);
			}
		}
		for(i=[0: 4]) {
			// Rotor mount holes
			rotate([0, 0, i*360/4]) {
				translate([0, base_d/2, 0])
					cube([box_len, box_width, base_h]);
			}
		}
	}
}

centry();
