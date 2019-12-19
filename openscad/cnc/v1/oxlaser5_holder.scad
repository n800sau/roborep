include <cnc_params.scad>
use <dragonlab_pipette_mockup.scad>
use <z_sliding_block.scad>
use <MCAD/nuts_and_bolts.scad>

$fn = 50;

oxlaser5_sz_x = 33;
oxlaser5_sz_y = 33;
oxlaser5_sz_z = 75;

oxlaser5_lens_d = 17;
oxlaser5_lens_sz_z = 10;

oxlaser5_pcb_sz_x = 41;
oxlaser5_pcb_sz_y = 32;
oxlaser5_pcb_sz_z = 55.5;

oxlaser5_pcb_dist = 5;

oxlaser5_hole_d_through = 3.6;

module oxlaser5_holder() {
	difference() {
		union() {
			h = oxlaser5_sz_z+oxlaser5_pcb_dist+oxlaser5_pcb_sz_z;
			cube([holder_sz_x+2*wall+1, wall, h], center=true);
			for(z=[0, 90]) {
				for(x=[-1,1]) {
					translate([x*(holder_sz_x+wall+1)/2, (17-wall)/2, (-h+28)/2+z]) {
						cube([wall, 17, 28], center=true);
					}
				}
			}
		}
		translate([0, 28, -40]) {
			attachment_holes(d=hole_d_through, hole_count=10);
		}
		translate([0, 0, -30]) {
			oxlaser5_mockup(holes_only=true);
		}
	}
}

module oxlaser5_mockup(holes_only=false) {
	difference() {
//	union() {
		// laser
		if(!holes_only) {
			cube([oxlaser5_sz_x, oxlaser5_sz_y, oxlaser5_sz_z], center=true);
		}
		for(z=[8, 16, 35, 53]) {
			translate([0, oxlaser5_sz_y/2, -oxlaser5_sz_z/2+z]) {
				rotate([90, 0, 0]) {
					cylinder(d=oxlaser5_hole_d_through, h=40, center=true);
				}
			}
		}
	}
	if(!holes_only) {
		translate([0, 0, -(oxlaser5_sz_z+oxlaser5_lens_sz_z)/2]) {
			cylinder(d=oxlaser5_lens_d, h=oxlaser5_lens_sz_z, center=true);
		}
	}
	// pcb holder
	translate([0, 0, (oxlaser5_sz_z+oxlaser5_pcb_sz_z)/2+oxlaser5_pcb_dist]) {
		difference() {
			if(!holes_only) {
				cube([oxlaser5_pcb_sz_x, oxlaser5_pcb_sz_y, oxlaser5_pcb_sz_z], center=true);
			}
			for(z=[14, oxlaser5_pcb_sz_z-14]) {
				translate([0, oxlaser5_sz_y/2, -oxlaser5_pcb_sz_z/2+z]) {
					rotate([90, 0, 0]) {
						cylinder(d=oxlaser5_hole_d_through, h=40, center=true);
					}
				}
			}
		}
	}		
}


translate([0, -43, 40]) {
  oxlaser5_holder();
}
//color("blue") {
//slider_z_attachment(extra_sz_z=attach_sz_z+10);
%slider_z_adapter(extra_sz_y=default_attach_extra_sz_y, extra_sz_z=attach_sz_z+10);
//}

translate([0, -60, 10]) {
	%oxlaser5_mockup();
}
