include <cnc_params.scad>
use <stepper_pusher.scad>
use <z_sliding_block.scad>
use <dragonlab_pipette_holder.scad>

dist_bottom_top = 75;
dist_y = 21;

module pipette_stepper_attachment() {
	difference() {
//	union() {
		translate([0, 13, -50]) {
			for(x=[-1,1]) {
				translate([x*(holder_sz_x+3*wall+1)/2, 0, -40]) {
					cube([wall, 15, 100], center=true);
				}
				hull() {
					translate([x*(holder_sz_x+3*wall+1)/2, 0, 10]) {
						cube([wall, 15, 10], center=true);
					}
					translate([x*(bar_length-10)/2, -(15-8)/2, 19]) {
						cube([10, 8, 5], center=true);
					}
				}
				translate([x*(bar_length-10)/2, -(15-8)/2, 21+(bar_side_length-28)/2]) {
					cube([10, 8, bar_side_length-28], center=true);
// too long
//					cube([12, 8, bar_side_length+10], center=true);
				}
			}
			for(z=[10, -30]) {
				translate([0, -(15-wall)/2, z]) {
					cube([holder_sz_x+3*wall, wall, 10], center=true);
				}
			}
			// bar above
			translate([0, -(15-8)/2, 90]) {
// too long
//				cube([bar_length, 8, 5], center=true);
			}
		}
		side_holes();
		translate([0, 35, -107]) {
			attachment_holes(d=hole_d_through_m4, hole_count=6);
		}
		for(x=[-2:2]) {
			translate([x*10, 10, 40]) {
//				cylinder(d=hole_d_m3, h=50, center=true);
			}
		}
	}
}

rotate([0, 0, 180]) {
	%pusher_frame(); 
	rotate([0, 180, 0]) { 
		%pusher_frame();
	} 
	%pusher();
	rotate([0, 180, 0]) { 
		%pusher(); 
	} 
	
	
	%stepper_frame(); 
	%stepper_mockup();

}

translate([0, 29+dist_y, -122-dist_bottom_top]) {
	%pipette_top_servo_holder();
}
	
pipette_stepper_attachment();
