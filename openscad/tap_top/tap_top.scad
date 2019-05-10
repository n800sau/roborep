$fn = 50;

top_d = 30;
top_h = 2;
border_d = 27;
border_w = 2;
border_h = 4;
cut_count = 3;
cut_w = 8;

cylinder(d=top_d, h=top_h, center=true);
translate([0, 0, -(border_h+top_h)/2]) {
	difference() {
		cylinder(d=border_d, h=border_h, center=true);
		cylinder(d=border_d-border_w, h=border_h, center=true);
		for(i=[0:cut_count]) {
			rotate([0, 0, 360/cut_count*i]) {
				cube([cut_w, top_d, border_h], center=true);
			}
		}
	}
}
