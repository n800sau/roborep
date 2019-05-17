use <../lib/lcd16x2.scad>

$fn = 50;

pcb_l = 80;
pcb_w = 36;
pcb_h = 2;

interhole_l = 75.5;
interhole_w = 31.5;

wall = 1.5;

inbox_l = pcb_l + 1;
inbox_w = pcb_w + 1;
inbox_h = 10;

box_l = inbox_l + 2*wall;
box_w = inbox_w + 2*wall;
box_h = inbox_h + wall;

hole_d = 3.2;
pass_hole_d = 4;

hole_wall_d = hole_d + wall*2;
pass_hole_wall_d = pass_hole_d + wall*2;

attach_w = 21+box_h;

lid_box_h = 9;
lid_inbox_h = lid_box_h - wall;

module lid() {
//  color("green") {
//import("/home/n800s/work/printer3d/openscad/good2print/bio/centrifuge/2.stl");
    translate([0, 0, lid_box_h]) {
      difference() {
        union() {
          difference() {
            translate([0, 0, wall/2]) {
              cube([box_l, box_w, lid_box_h], center=true);
            }
            cube([inbox_l, inbox_w, lid_inbox_h], center=true);
            lcd_box_hole(h=50);
          }
          translate([0, 0, pcb_h/4]) {
            lcd_holes(h=lid_inbox_h-pcb_h/2, box_h=0, d=hole_wall_d);
          }
          // attachment
          difference() {
            translate([0, -box_w/2-wall/2-0.5, (-attach_w+lid_box_h+wall)/2]) {
              cube([box_l, wall+1, attach_w], center=true);
            }
            translate([0, -box_w/2, -attach_w/2-2]) {
              translate([inbox_l/2-10, 0, -5]) {
                rotate([90, 0, 0]) {
                  cylinder(d=pass_hole_d, h=20, center=true);
                }
              }
              translate([-inbox_l/2+10, 0, -5]) {
                rotate([90, 0, 0]) {
                  cylinder(d=pass_hole_d, h=20, center=true);
                }
              }
            }
          }
        }
        lcd_holes(h=50);
      }
    }
//  }
}

module box() {
//  color("blue") {
    difference() {
      union() {
        difference() {
          translate([0, 0, -wall/2]) {
            cube([box_l, box_w, box_h], center=true);
          }
          cube([inbox_l, inbox_w, inbox_h], center=true);
        }
        translate([0, 0, -pcb_h/4]) {
          lcd_holes(h=inbox_h-pcb_h/2, d=hole_wall_d);
        }
      }
      lcd_holes(h=50, d=hole_d);
      // wire hole
      wh_w = 13;
      translate([inbox_l/2, -inbox_w/2+wh_w/2+5, 0]) {
        cube([20, wh_w,  inbox_h], center=true);
      }
    }
//  }
}

//lid();
box();

