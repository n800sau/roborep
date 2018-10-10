$fn = 30;


module fillet(r, h) {
  translate([r / 2, r / 2, 0])
    difference() {
      cube([r + 0.01, r + 0.01, h], center = true);
      translate([r/2, r/2, 0])
        cylinder(r = r, h = h + 1, center = true);
  }
}


module lg_cam_attach() {
  
  difference() {
    cube([9,20,27]);
    translate([0, 10, 27])
      rotate([90, 90, 0])
        fillet(r=3, h=22);
    translate([9, 10, 27])
      rotate([90, 90, 180])
        fillet(r=3, h=22);
    translate([0, 21, 19]) {
      rotate([90, 0, 0]) {
        cylinder(r=4, h=22);
      }
    }
    translate([4.5, 15, 0]) {
      cylinder(d=2.8, h=17);
      translate([0, -10, 0]) {
        cylinder(d=2.8, h=17);
      }
    }
  }

}

lg_cam_attach();

