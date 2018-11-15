$fn = 50;

h = 17;
w = 30;
l = 20;
thick = 2;

hole_h = 6;
hole_w = 11;


module top() {
  // top
  translate([w/8-2, -thick/2, -thick/2]) {
    difference() {
    //  translate([-10, 0, 0])
      cube([w/4*3, thick, h], center=true);
      translate([2.5, 0, 3]) {
        cube([hole_w, thick, hole_h], center=true);
      }
      translate([-2.5-5, 0, 4.5]) {
        rotate([90, 0, 0])
          cylinder(d=4, h=50, center=true);
      }
    }
  }
}

// bottom
module bottom() {
  translate([0, -l/2+3, -h/2]) {
    difference() {
      union() {
        translate([5, 0, 0]) {
          difference() {
            cube([w-l/2-4, l-7, thick], center=true);
            translate([10, -6, 0]) {
              rotate(35) {
                cube([30, 20, thick], center=true);
              }
            }
          }
        }
        // with hole to attach to HDD
        translate([-5, 0, 1]) {
          cylinder(r=l/2-3, h=thick+2, center=true);
        }
      }
      // for bolt
      translate([-5, 0, 0]) {
        cylinder(d=4, h=50, center=true);
      }
      // bigger hole for nut
      translate([-5, 0, -1]) {
        cylinder(r=5, h=2, center=true);
      }
    }
  }
}

module wire_holder() {
  translate([0, -l/2+2, -h/2]) {
    difference() {
      union() {
        // with hole to attach to HDD
        translate([-5, 0, thick]) {
          cylinder(r=4, h=thick, center=true);
          translate([10, 0, 0]) {
            cube([15, 5, thick], center=true);
            translate([6, 0, -thick-0.5]) {
              cube([thick+1, 5, thick+2], center=true);
            }
          }
        }
      }
      // for bolt
      translate([-5, 0, 0]) {
        cylinder(d=4, h=50, center=true);
      }
    }
  }
}


//top();
//bottom();
wire_holder();