$fn = 50;

pad = 0.5;
wall = 2;

cam_sz_x = 25;
cam_sz_z = 24;

box_sz_x = cam_sz_x + pad*2 + wall*2;
box_off_z = 20;
box_sz_z = cam_sz_z + box_off_z + pad*2 + wall*2;
box_sz_y = 20;

lens_center_from_top = 9;
lens_d = 13.8;

cam_hole_d = 2.2;
cam_stand_h = 5.5;
cam_stand_wall = 0.8;
cam_hole_from_top = 2;
cam_hole_from_side = 2;
cam_hole_dist_x = 21;
cam_hole_dist_z = 13.5;

use <ear.scad>

module holes(stand=false) {
  d=stand ? cam_hole_d+2*cam_stand_wall : cam_hole_d;
  h=stand ? cam_stand_h : 20;
  for(pos=[
      [cam_hole_from_side, cam_hole_from_top, 3],
      [cam_hole_from_side+cam_hole_dist_x, cam_hole_from_top, -3],
      [cam_hole_from_side, cam_hole_from_top+cam_hole_dist_z, 3],
      [cam_hole_from_side+cam_hole_dist_x, cam_hole_from_top+cam_hole_dist_z, -3]
    ]) {
    translate([cam_sz_x/2-pos[0], 0, cam_sz_z/2-pos[1]]) {
      rotate([90, 0, 0]) {
        cylinder(d=d, h=h);
      }
      if(stand) {
        translate([pos[2], -h/2, 0]) {
          cube([6, h, d], center=true);
        }
      }
    }
  }
}

module raspicam() {
  difference() {
    union() {
      cube([cam_sz_x+pad*2, wall, cam_sz_z+pad*2], center=true);
      translate([0, 0, -box_off_z/2]) {
        cube([box_sz_x, wall, box_sz_z], center=true);
        for(sgn=[-1, 1]) {
          translate([sgn*(box_sz_x+wall)/2, (box_sz_y-wall)/2, 0]) {
            cube([wall, box_sz_y, box_sz_z], center=true);
          }
          translate([0, (box_sz_y-wall)/2, sgn*(box_sz_z+wall)/2]) {
            cube([box_sz_x+2*wall, box_sz_y, wall], center=true);
          }
        }
      }
      translate([0, cam_stand_h+wall/2, 0]) {
        holes(stand=true);
      }
    }
    translate([0, 10, cam_sz_z/2-lens_center_from_top]) {
      rotate([90, 0, 0]) {
        cylinder(d=lens_d, h=20);
      }
    }
    translate([0, cam_stand_h+wall/2, 0]) {
      holes();
    }
  }
  for(pos=[
        [-1, -1, 180],
        [1, 1, 0],
        [-1, 1, 0],
        [1, -1, 180],
      ]) {
        translate([pos[0]*(box_sz_x-15)/2, box_sz_y-wall/2, pos[1] * (box_sz_z/2+wall)-box_off_z/2]) {
          rotate([0, pos[2], 0]) {
            rotate([0, 90, 90]) {
              bolt_hole_cone(wall_extra=0, height_extra=5);
            }
          }
        }
  }

}

raspicam();
