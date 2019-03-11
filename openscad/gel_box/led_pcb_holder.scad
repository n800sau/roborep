use <MCAD/boxes.scad>


$fn = 50;

wall = 2;
int_d = 21;
int_h = 6;
ext_d = int_d + wall * 2;
ext_h = int_h + wall;

hole_d = 4;
ext_size = hole_d+wall+1;

module led_pcb_holder() {

  difference() {
    union() {
      difference() {
        cylinder(d=ext_d, h=ext_h, center = true);
        translate([0, 0, (ext_h-int_h)/2]) {
          cylinder(d=int_d, h=int_h, center = true);
        }
      }
      for(p=[0:60:360]) {
        rotate([0, 0, p+30]) {
          translate([int_d/2, 0, -int_h/2+wall+3]) {
            hull() {
              cube([3, 2, 2], center=true);
              translate([0, 0, -2]) {
                cube([0.1, 2, 2], center=true);
              }
            }
          }
        }
      }
    }
    translate([0, 0, (ext_h-int_h)/2]) {
      for(p=[0:60:360]) {
        rotate([0, 0, p]) {
          translate([int_d/2, 0, 0]) {
            cube([wall*2, 3, int_h], center=true);
          }
        }
      }
    }
  }
  
  
  rotate([0, 0, 30]) {
    translate([0, 0, (-ext_h+wall)/2]) {
      difference() {
        roundedBox([ext_d+ext_size*2, hole_d + wall, wall], wall, true);
        translate([(ext_d+ext_size)/2, 0, 0]) {
          cylinder(d=hole_d, h=50, center=true);
        }
        translate([-(ext_d+ext_size)/2, 0, 0]) {
          cylinder(d=hole_d, h=50, center=true);
        }
      }
    }
  }
}

led_pcb_holder();
