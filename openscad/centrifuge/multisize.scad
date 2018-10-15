base_d = 85;
base_h = 20;

box_len = base_d;
box_width = 23;

rotor_d = 30;
rotor_h = 5;

hole_dist = 21;
hole_d = 2.5;

insert_width = 3.2;
insert_len = 35;

$fn = 200;

module centry() {
//	union() {
	difference() {
		// base body
		cylinder(d = base_d, h = base_h);


		translate([0, 0, rotor_h]) {
			cylinder(d = rotor_d, h = base_h);
    }

		for(i=[0: 6]) {
			// Rotor mount holes
			rotate([0, 0, i*360/6]) {
				translate([0, hole_dist/2, -0.1])
					cylinder(d = hole_d, h = 6.1);
			}
		}
		for(i=[0: 4]) {
			rotate([0, 0, i*360/4]) {
				translate([-box_width/2, rotor_d/2+3, rotor_h]) {
					cube([box_width, box_len, base_h]);
        }
				translate([-insert_len/2, rotor_d/2+10, rotor_h]) {
					cube([insert_len, insert_width, base_h]);
        }
			}
		}
	}
}

centry();
