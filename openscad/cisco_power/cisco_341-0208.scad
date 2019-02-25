use <../lib/vent.scad>

$fn = 50;

gap = 1;
wall = 2.5;

sz_x = 128;
sz_y = 64;


below_h = 2;
pcb_h = 1.5;
above_h_low=5.4;
above_h_higher = 24.6;

top_part_h = above_h_higher + gap + wall;
bottom_part_h = wall + below_h + pcb_h + above_h_low + gap;

holes_x_dist = 112.5;
holes_y_dist = 50;

module bottom_part() {
  //union() {
  difference() {
    union() {
      translate([0, 0, (bottom_part_h)/2]) {
        difference() {
          cube([sz_x+2*gap+2*wall, sz_y+2*gap+2*wall, bottom_part_h], center=true);
          translate([0, 0, wall]) {
            cube([sz_x+2*gap, sz_y+2*gap, bottom_part_h], center=true);
          }
        }
      }
      for(ypos=[-holes_y_dist/2, holes_y_dist/2]) {
        for(xpos=[-holes_x_dist/2, holes_x_dist/2]) {
          translate([xpos, ypos, wall+below_h/2]) {
            cylinder(d=8, h=below_h, center=true);
          }
        }
      }
    }
    for(ypos=[-holes_y_dist/2, holes_y_dist/2]) {
      for(xpos=[-holes_x_dist/2, holes_x_dist/2]) {
        translate([xpos, ypos, 0]) {
          cylinder(d=3.8, h=20, center=true);
        }
      }
    }
  }
}

module top_part() {
  translate([0, 0, (top_part_h+wall+gap)/2]) {
//    union() {
    difference() {
      cube([sz_x+2*gap+2*wall, sz_y+2*gap+2*wall, top_part_h+wall+gap], center=true);
      translate([0, 0, -wall/2]) {
        cube([sz_x+2*gap, sz_y+2*gap, top_part_h+gap], center=true);
        // 220v hole
        translate([sz_x/2-29/2-15, -sz_y/2, -3.5]) {
          cube([29+2*gap, wall+10, 22+gap], center=true);
        }
      }
      translate([-60/2, -sz_y/2, 0]) {
        cube([60, wall+10, 20], center=true);
      }
      translate([0, sz_y/2, 0]) {
        cube([sz_x-10, wall+10, 20], center=true);
      }
      translate([0, 0, (top_part_h+gap)/2]) {
        cube([sz_x-25, sz_y-10, wall], center=true);
      }
      translate([-sz_x/2, sz_y/2-15, -top_part_h/2-gap]) {
        rotate([0, 90, 0]) {
          cylinder(d=6, h=20, center=true);
        }
      }
    }
    translate([-60/2, -sz_y/2-wall, 0]) {
      vent(x_size=61, z_size=20, y_size=wall, x_count=10, z_count=3);
    }
    translate([0, sz_y/2+wall, 0]) {
      vent(x_size=sz_x-10, z_size=20, y_size=wall, x_count=19, z_count=3);
    }
    translate([0, 0, (top_part_h+gap)/2]) {
      rotate([90, 0, 0]) {
        vent(x_size=sz_x-25, z_size=sz_y-10, y_size=wall, x_count=19, z_count=3);
      }
    }
  }
  for(ypos=[-holes_y_dist/2, holes_y_dist/2]) {
    for(xpos=[-holes_x_dist/2, holes_x_dist/2]) {
      translate([xpos, ypos, (gap+top_part_h+above_h_low)/2-above_h_low]) {
//          union() {
        difference() {
          union() {
            cylinder(d=8, h=gap+top_part_h+above_h_low, center=true);
            translate([0, 0, above_h_low/2]) {
              linear_extrude(height=gap+top_part_h, center = true, convexity = 10) {
                polygon(points=[[0,sign(ypos)*4],
                  [sign(xpos)*4, sign(ypos)*(sz_y/2+wall+gap-abs(ypos))],
                  [sign(xpos)*(sz_x/2+wall+gap-abs(xpos)), sign(ypos)*(sz_y/2+wall+gap-abs(ypos))],
                  [sign(xpos)*(sz_x/2+wall+gap-abs(xpos)), sign(ypos)*4],
                  [sign(xpos)*4, 0]]);
              }
            }
          }
          translate([0, 0, -wall-gap]) {
            cylinder(d=3.2, h=top_part_h, center=true);
          }
        }
      }
    }
  }
}

translate([0, 0, bottom_part_h]) {
//  top_part();
}
bottom_part();

//vent(center=false);

