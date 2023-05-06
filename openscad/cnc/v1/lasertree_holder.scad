include <cnc_params.scad>
use <dragonlab_pipette_mockup.scad>
use <z_sliding_block.scad>
use <MCAD/nuts_and_bolts.scad>

$fn = 50;

lasertree_sz_x = 40;
lasertree_sz_y = 40;
lasertree_sz_z = 108;

lasertree_hole_d_through = 3.6;
lasertree_hole_dist = 16;

module lasertree_holder() {
	difference() {
//	union() {
		union() {
			h = lasertree_sz_z;
			cube([holder_sz_x+2*wall+1, wall, h], center=true);
			for(z=[20, 80]) {
				for(x=[-1,1]) {
					translate([x*(holder_sz_x+wall+1)/2, (17-wall)/2, (-h+28)/2+z]) {
						cube([wall, 17, 28], center=true);
					}
				}
			}
		}
		translate([0, 28, -37]) {
			attachment_holes(d=hole_d_through_m4, hole_count=10);
		}
		for(z=[-30:10:30]) {
			translate([0, -20, z]) {
				lasertree_mockup(holes_only=true);
			}
		}
	}
}

module lasertree_mockup(holes_only=false) {
	difference() {
//	union() {
		// laser
		if(!holes_only) {
			cube([lasertree_sz_x, lasertree_sz_y, lasertree_sz_z], center=true);
		}
		for(x=[1, -1]) {
			translate([x*lasertree_hole_dist/2, 20, 0]) {
				rotate([90, 0, 0]) {
					cylinder(d=lasertree_hole_d_through, h=40, center=true);
				}
			}
		}
	}
}


translate([0, -43, 10]) {
  lasertree_holder();
}
//color("blue") {
//slider_z_attachment(extra_sz_z=attach_sz_z+10);
%slider_z_adapter(extra_sz_y=default_attach_extra_sz_y, extra_sz_z=attach_sz_z+10);
//}

translate([0, -60, 7]) {
//	%lasertree_mockup();
}
