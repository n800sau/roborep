$fn = 40;

module connector(wall=2) {
  translate([7, 5.5, 7.5]) {
    cube([19, 15, wall]);
  }
  translate([26.5, 5.4, 7.4]) {
    rotate([0, 90, 90]) {
      for(y=[0, 6.4]) {
        translate([-10.2, 6.9+y, 7.6]) {
          rotate([90, 0, 0]) {
            difference() {
              union() {
                cylinder(d=14.7, h=2.6, center=true);
                translate([5, 0, 0]) {
                  cube([10, 14.7, 2.6], center=true);
                }
              }
              translate([-1.4, 0.1, 0]) {
                cylinder(d=6, h=3, center=true);
              }
            }
          }
        }
      }
    }
  }
}

connector();