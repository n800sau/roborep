use <MCAD/2Dshapes.scad>
use <MCAD/nuts_and_bolts.scad>

$fn = 50;

wall = 3;
out_r = 60;
close_hole_dist = 50;
hole_dist = 75;
hole_d = 3.8;
hole_d_through = 4.4;
wall_extra = wall + 1;
wall_extra_out = wall_extra + 2;

metal_block_sz_x = 20 + 0.4;
metal_block_sz_y = 16 + 0.4;

difference() {
	union() {
		cylinder(r=out_r, h=wall, center=true);
    translate([0, 0, (4-wall)/2]) {
      cube([metal_block_sz_x+2*wall, metal_block_sz_y+2*wall, 4], center=true);
    }
    translate([0, 0, wall_extra/2]) {
      linear_extrude(height=wall_extra_out, center=true) {
        for(i=[0:1:4]) {
          donutSlice(46, out_r, 37+i*90, 53+i*90);
        }
      }
    }
    translate([0, 0, wall_extra/2]) {
      linear_extrude(height=wall_extra, center=true) {
        for(i=[0:1:4]) {
          donutSlice(18, 46, 37+i*90, 53+i*90);
        }
      }
    }
	}
  linear_extrude(height=2*wall, center=true) {
    for(i=[0:1:4]) {
      donutSlice(20, out_r-10, -30+i*90, 30+i*90);
    }
  }
  translate([0, 0, (12-wall)/2+wall]) {
    cube([metal_block_sz_x, metal_block_sz_y, 12], center=true);
  }
  translate([-wall+0.4, 0, (12-wall)/2]) {
    cube([metal_block_sz_x-4, metal_block_sz_y-4, 12], center=true);
  }
	for(hole_pos=[[1,-1], [1,1], [-1,1], [-1,-1]]) {
		translate([hole_pos[0]*close_hole_dist/2, hole_pos[1]*close_hole_dist/2, wall]) {
      nutHole(4);
			cylinder(d=hole_d_through, h=50, center=true);
		}
		translate([hole_pos[0]*hole_dist/2, hole_pos[1]*hole_dist/2, -wall]) {
      nutHole(4);
			cylinder(d=hole_d_through, h=50, center=true);
		}
	}
}
