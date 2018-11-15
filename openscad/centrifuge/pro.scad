use <MCAD/boxes.scad>

a_pro_l = 53;
a_pro_w = 52.5;

power_side_hole_dist = 48.5;
power_side_bottom_hole_off = 2.5;
power_side_left_hole_off = 2.5;
reset_side_hole_dist = 28;
reset_side_bottom_hole_off = 7.5;
bottom_side_hole_dist = 47;

box_l = a_pro_l + 2;
box_w = a_pro_w + 16;

// how Arduino connects to board, choose printed plastic pins or screw holes
mounting_points = 1; // [0:Pins,1:Screw holes]

// in mm
mounting_post_height = 6;

// in mm
mounting_post_diameter = 6.4;

// in mm
screw_hole_diameter = 3.2;

bolt_hole_diameter = 4;


// in mm, 2.9 suggested for MakerBot Replicator 2 at 0.2mm layer height
pin_diameter = 2.9;


$fn = 50;
WALL = 1.5;

module mountingPost(origin = [0,0,0]) {
	translate(origin)
		difference() {
			union() {
				cylinder(h = mounting_post_height+0.5, d = mounting_post_diameter);
				if (mounting_points == 0) {
					translate([0,0,mounting_post_height])
						cylinder(h = 3, d = pin_diameter);
				} // end if mounting_points == 1
			} // end union

			if (mounting_points == 1) {
				cylinder(h = mounting_post_height+1, d = screw_hole_diameter);
			} // end if mounting_points == 1
		} // end difference
} // end module mountingPost

difference() {
  union() {
    difference() {
      translate([-(box_l-a_pro_l)/2, (box_w-a_pro_w)/2-1, 0]) {
        roundedBox([box_l, box_w, WALL], 2, true);
      }
      translate([(-box_l+a_pro_l)/2, (box_w-a_pro_w)/2, 0]) {
        roundedBox([box_l-20, box_w-20, WALL], 1, true);
      }
    }
    // left bottom
    mountingPost([-a_pro_l/2+power_side_left_hole_off,
      -a_pro_w/2+power_side_bottom_hole_off,
      WALL/2]);
    // left top
    mountingPost([-a_pro_l/2+power_side_left_hole_off,
      -a_pro_w/2+power_side_bottom_hole_off+power_side_hole_dist,
      WALL/2]);
    // right bottom
    mountingPost([-a_pro_l/2+power_side_left_hole_off+bottom_side_hole_dist,
      -a_pro_w/2+reset_side_bottom_hole_off,
      WALL/2]);
    // right top
    mountingPost([-a_pro_l/2+power_side_left_hole_off+bottom_side_hole_dist,
      -a_pro_w/2+reset_side_bottom_hole_off+reset_side_hole_dist,
      WALL/2]);
    
    mountingPost([-a_pro_l/2+power_side_left_hole_off+4,
      -a_pro_w/2+power_side_bottom_hole_off+power_side_hole_dist+12,
      WALL/2]);
    mountingPost([-a_pro_l/2+power_side_left_hole_off+45,
      -a_pro_w/2+power_side_bottom_hole_off+power_side_hole_dist+12,
      WALL/2]);

  }

  translate([0, 0, 0]) {
    translate([0, -a_pro_w/2+power_side_bottom_hole_off+power_side_hole_dist+12, 0])
      cylinder(h = 50, d = bolt_hole_diameter, center=true);
    translate([0, -a_pro_w/2+power_side_bottom_hole_off+2, 0])
      cylinder(h = 50, d = bolt_hole_diameter, center=true);
  }

}
