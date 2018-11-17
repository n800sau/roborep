include <bb_box_params.scad>;
use <step_down_1.scad>;


module RoundedBB(extra_width,extra_height,depth,curve=bb_curve_corner, power_side_width=power_side_width) {
	difference() {
    union() {
      minkowski() {
        translate([power_side_width/2-other_side_width/2, 0, 0]) {
          full_width = bb_width-curve+extra_width+power_side_width+other_side_width;
          echo("Full Width:", full_width);
          cube([full_width, bb_height-curve+extra_height,depth],center=true);
        }
        cylinder(r=curve/2, height=depth, $fn=32);
      }
    }
	}
}

module BeagleBox(curve=bb_curve_corner) {
	difference() {
		RoundedBB(wall_thickness, wall_thickness, bb_depth, curve, power_side_width);
		translate([0,0,base_thickness])
			RoundedBB(0, 0, bb_depth, curve-2);
	}
}

module Case() {
  difference() {
    union() {
      difference() {
        BeagleBox();
  
        // holes for lid
        translate([bb_width/2,-bb_height/4,bb_depth-hole4lid_depth])
          rotate([0, 90, 0])
            cylinder(r=2, h=50, center=true);
  
        translate([-bb_width/2,-bb_height/4,bb_depth-hole4lid_depth])
          rotate([0, 90, 0])
            cylinder(r=2, h=30, center=true);
  
        translate([bb_width/2,bb_height/4,bb_depth-hole4lid_depth])
          rotate([0, 90, 0])
            cylinder(r=2, h=50, center=true);
  
        translate([-bb_width/2,bb_height/4,bb_depth-hole4lid_depth])
          rotate([0, 90, 0])
            cylinder(r=2, h=30, center=true);
  
  
        translate([0,0,-bb_depth/2+9]) {
          // -- Bottom End --
  
          // 12v power hole
          translate([32,-bb_height/2, 24])
            rotate([90, 0, 0])
              cylinder(r=3.5, h=5,center=true);
  
  
          // Power connector
          translate([18,-bb_height/2, 1.5+peg_extra_depth])
            cube([10,10,12],center=true);
          // Ethernet port
          translate([-2,-bb_height/2,2.5+peg_extra_depth])
            cube([16.5,10,14],center=true);
  
          // Mini USB (Underneath)
          // translate([-23,-bb_height/2-5,-16])     // protruding
          // translate([-23,-bb_height/2,-9+0.334])  // thin layer underneath
          //	cube([10,12,base_thickness-0.3333]);
          translate([-22,-bb_height/2-5,-12+peg_extra_depth])
            cube([10,15,6]);
  
          // -- Top End --
  
          // SD Card slot
          translate([-6.5,bb_height/2,-6.5+peg_extra_depth])
            cube([14,10,4], center=true);
          // USB port
          translate([10.5,bb_height/2,0.5+peg_extra_depth])
            cube([15,10,8],center=true);
        }
      }
  
      // Pegs
      translate([0,0,-bb_depth/2+9]) {
        // Pegs on "power" end
        translate([-bb_width/2+peg_pow_side,-bb_height/2+peg_pow_front,-4])
          cylinder(r=peg_outer_radius,h=peg_depth,$fn=32,center=true);
        translate([bb_width/2-peg_pow_side,-bb_height/2+peg_pow_front,-4])
          cylinder(r=peg_outer_radius,h=peg_depth,$fn=32,center=true);
  
        // Pegs on "sd card" end
        translate([-bb_width/2+peg_sd_side,bb_height/2-peg_sd_front,-4])
          cylinder(r=peg_outer_radius,h=peg_depth,$fn=32,center=true);
        translate([bb_width/2-peg_sd_side,bb_height/2-peg_sd_front,-4])
          cylinder(r=peg_outer_radius,h=peg_depth,$fn=32,center=true);
  
  
      }
      //slot for step down module
      translate([bb_width/2+power_side_width-1.5, 15, -3])
        rotate([0, 90, 180])
          step_down_1();
    }
  
    // Drill holes through pegs and floor
    translate([0,0,-bb_depth/2+9]) {
      // Pegs on "power" end
      translate([-bb_width/2+peg_pow_side,-bb_height/2+peg_pow_front,-10+peg_extra_depth])
        cylinder(r=peg_inner_radius,h=12,$fn=32,center=true);
      translate([bb_width/2-peg_pow_side,-bb_height/2+peg_pow_front,-10+peg_extra_depth])
        cylinder(r=peg_inner_radius,h=12,$fn=32,center=true);
  
      // Pegs on "sd card" end
      translate([-bb_width/2+peg_sd_side,bb_height/2-peg_sd_front,-10+peg_extra_depth])
        cylinder(r=peg_inner_radius,h=12,$fn=32,center=true);
      translate([bb_width/2-peg_sd_side,bb_height/2-peg_sd_front,-10+peg_extra_depth])
        cylinder(r=peg_inner_radius,h=12,$fn=32,center=true);
  
      // Middle pegs
      translate([10, 25,-5])
        cylinder(r=peg_inner_radius,h=10,$fn=32,center=true);
      translate([10, -20,-5])
        cylinder(r=peg_inner_radius,h=10,$fn=32,center=true);
  
      // USB extra removal
      translate([10.5,bb_height/2+8.5,0.5+peg_extra_depth])
            cube([20,15,10],center=true);
    }
  }
}

Case();
