h = 20;
w = 30;
l = 20;
thick = 2;

hole_h = 10;
hole_w = 10;


difference() {
	cube([w, thick, h], center=true);
	cube([hole_w, thick, hole_h], center=true);
	translate([-10, 0, 0]) {
		rotate([90, 0, 0])
			cylinder(d=2.7, h=50, center=true);
	}
}
translate([0, -l/2, -h/2]) {
	difference() {
		cube([w, l, thick], center=true);
		translate([-5, 0, 0]) {
			cylinder(d=3.4, h=50, center=true);
		}
	}
}

