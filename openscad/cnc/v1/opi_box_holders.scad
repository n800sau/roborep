$fn = 40;

union() {
	union() {
		cube([12, 3, 120]);
		cube([12, 20+3, 3]);
	}
	for(z=[30, 40]) {
		translate([6-1.8, 0, z]) {
			rotate([-90, 0, 0]) {
				cylinder(d=3.6, h=10);
			}
		}
	}
	translate([6-1.8, 10+3, 0]) {
		cylinder(d=3.6, h=10);
	}
}

