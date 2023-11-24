$fn = 50;
wall = 2;

module leg(head_d=8, bolt_d=4, head_space_h=3.5, cone_h=2.8, pad=0.5) {

	ext_d = head_d + wall + pad*2;
	h = head_space_h + wall;

	translate([0, 0, wall]) {
		difference() {
		cylinder(d=ext_d, h=head_space_h);
		cylinder(d=head_d+pad*2, h=head_space_h-cone_h);
		translate([0, 0, head_space_h-cone_h]) {
			cylinder(d1=head_d+pad*2, d2=bolt_d+pad*2, h=cone_h);
		}
		}
	}
	difference() {
		cylinder(d=ext_d, h=wall);
		cylinder(d=bolt_d+pad*2, h=wall);
	}

}

//leg(bolt_d=4, head_d=8, head_space_h=3.5, cone_h=2.8);
leg(bolt_d=3, head_d=5.2, head_space_h=4, cone_h=0, pad=1.2);
