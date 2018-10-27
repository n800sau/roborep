$fn = 50;

pcb_l = 80;
pcb_w = 36;
pcb_h = 1.5;

interhole_l = 75;
interhole_w = 31;

wall = 1.5;

inbox_l = pcb_l + 1;
inbox_w = pcb_w + 1;
inbox_h = 10;

box_l = inbox_l + 2*wall;
box_w = inbox_w + 2*wall;
box_h = inbox_h + wall;

hole_d = 2.6;
pass_hole_d = 4;

hole_wall_d = hole_d + wall*2;
pass_hole_wall_d = pass_hole_d + wall*2;

attach_w = 30+box_h;

lid_box_h = 9;
lid_inbox_h = lid_box_h - wall;

lcd_l = 71;
lcd_w = 24;

module lid() {
  color("green") {
//import("/home/n800s/work/printer3d/openscad/good2print/bio/centrifuge/2.stl");
    translate([0, 0, 10]) {
      difference() {
        union() {
          difference() {
            translate([0, 0, wall/2]) {
              cube([box_l, box_w, lid_box_h], center=true);
            }
            cube([inbox_l, inbox_w, lid_inbox_h], center=true);
            cube([lcd_l, lcd_w, 50], center=true);
          }
          // hole walls
          translate([interhole_l/2, interhole_w/2, pcb_h/4]) {
            cylinder(d=hole_wall_d, h=lid_inbox_h-pcb_h/2, center=true);
          }
          translate([-interhole_l/2, interhole_w/2, pcb_h/4]) {
            cylinder(d=hole_wall_d, h=lid_inbox_h-pcb_h/2, center=true);
          }
          translate([-interhole_l/2, -interhole_w/2, pcb_h/4]) {
            cylinder(d=hole_wall_d, h=lid_inbox_h-pcb_h/2, center=true);
          }
          translate([interhole_l/2, -interhole_w/2, pcb_h/4]) {
            cylinder(d=hole_wall_d, h=lid_inbox_h-pcb_h/2, center=true);
          }
          // attachment
          difference() {
            translate([0, -box_w/2-wall/2-0.5, (-attach_w+lid_box_h+wall)/2]) {
              cube([box_l, wall+1, attach_w], center=true);
            }
            translate([0, -box_w/2, -attach_w/2-2]) {
              translate([inbox_l/2-10, , 0]) {
                rotate([90, 0, 0]) {
                  cylinder(d=pass_hole_d, h=20, center=true);
                }
              }
              translate([-inbox_l/2+10, 0, 0]) {
                rotate([90, 0, 0]) {
                  cylinder(d=pass_hole_d, h=20, center=true);
                }
              }
            }
          }
        }
        // holes
        translate([0, 0, -25]) {
          translate([interhole_l/2, interhole_w/2, 25]) {
            cylinder(d=pass_hole_d, h=50, center=true);
          }
          translate([-interhole_l/2, interhole_w/2, 25]) {
            cylinder(d=pass_hole_d, h=50, center=true);
          }
          translate([-interhole_l/2, -interhole_w/2, 25]) {
            cylinder(d=pass_hole_d, h=50, center=true);
          }
          translate([interhole_l/2, -interhole_w/2, 25]) {
            cylinder(d=pass_hole_d, h=50, center=true);
          }
        }
      }
    }
  }
}

module box() {
  color("blue") {
    translate([0, 0, -6.5]) {
      difference() {
        union() {
          difference() {
            translate([0, 0, -wall/2]) {
              cube([box_l, box_w, box_h], center=true);
            }
            cube([inbox_l, inbox_w, inbox_h], center=true);
          }
          // hole walls
          translate([interhole_l/2, interhole_w/2, -pcb_h/4]) {
            cylinder(d=hole_wall_d, h=inbox_h-pcb_h/2, center=true);
          }
          translate([-interhole_l/2, interhole_w/2, -pcb_h/4]) {
            cylinder(d=hole_wall_d, h=inbox_h-pcb_h/2, center=true);
          }
          translate([-interhole_l/2, -interhole_w/2, -pcb_h/4]) {
            cylinder(d=hole_wall_d, h=inbox_h-pcb_h/2, center=true);
          }
          translate([interhole_l/2, -interhole_w/2, -pcb_h/4]) {
            cylinder(d=hole_wall_d, h=inbox_h-pcb_h/2, center=true);
          }
        }
        // holes
        translate([0, 0, -25]) {
          translate([interhole_l/2, interhole_w/2, 25]) {
            cylinder(d=hole_d, h=50, center=true);
          }
          translate([-interhole_l/2, interhole_w/2, 25]) {
            cylinder(d=hole_d, h=50, center=true);
          }
          translate([-interhole_l/2, -interhole_w/2, 25]) {
            cylinder(d=hole_d, h=50, center=true);
          }
          translate([interhole_l/2, -interhole_w/2, 25]) {
            cylinder(d=hole_d, h=50, center=true);
          }
        }
        translate([inbox_l/2, -inbox_w/2+13, 0]) {
          cube([20, 13,  inbox_h], center=true);
        }
      }
    }
  }
}

lid();
box();
