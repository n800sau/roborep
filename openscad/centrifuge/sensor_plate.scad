$fn = 50;

h = 20;
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
      translate([2.5, 0, 5]) {
        cube([hole_w, thick, hole_h], center=true);
      }
      translate([-2.5-5, 0, 6.5]) {
        rotate([90, 0, 0])
          cylinder(d=3.2, h=50, center=true);
      }
    }
  }
}

// bottom
module bottom() {
  translate([0, -l/2+2, -h/2]) {
    difference() {
      union() {
        translate([5, 0, 0]) {
          difference() {
            cube([w-l/2-2, l-6, thick], center=true);
            translate([10, -5, 0]) {
              rotate(45) {
                cube([30, 20, thick], center=true);
              }
            }
          }
        }
        // with hole to attach to HDD
        translate([-5, 0, 0]) {
          cylinder(r=l/2-2, h=thick, center=true);
        }
      }
      // for bolt
      translate([-5, 0, 0]) {
        cylinder(d=3.4, h=50, center=true);
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
          cylinder(r=5, h=thick*2, center=true);
          translate([12, 0, thick/2]) {
            cube([17, 5, thick], center=true);
            translate([7, 0, -thick]) {
              cube([thick+1, 5, thick], center=true);
            }
          }
        }
      }
      // for bolt
      translate([-5, 0, 0]) {
        cylinder(d=3.6, h=50, center=true);
      }
    }
  }
}


//top();
//bottom();
wire_holder();