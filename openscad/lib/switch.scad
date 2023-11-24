module switch_holes() {
	cube([9.5, 5, 50], center=true);
	translate([9.5, 0, 0]) {
		cylinder(d=2.5, h = 50, center=true);
	}
	translate([-9.5, 0, 0]) {
		cylinder(d=2.5, h = 50, center=true);
	}
}

