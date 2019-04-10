$fn = 50;
d = 2;
difference() {
	union() {
		for(x=[-10:10]) {
			for(y=[-10:10]) {
				translate([x*d, y*d, 0]) {
					sphere(d=d, center=true);
				}
			}
		}
		translate([0, 0, 0.5]) {
//			cube([100, 100, 1], center=true);
		}
	}
	translate([0, 0, -50]) {
//		cube([100, 100, 100], center=true);
	}
}
