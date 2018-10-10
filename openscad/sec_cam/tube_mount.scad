$fn=50;

difference() {
  union() {
    rotate([90, 0, 0]) {
      import("tripod_adapter.stl");
    }
    translate([0, -35, 0]) {
      cube([30, 40, 3]);
      translate([15, 0, 0]) {
        linear_extrude(height=3) {
          circle(r=15);
        }
      }
      translate([15, 40, 0]) {
        linear_extrude(height=3) {
          circle(r=15);
        }
      }
    }
  }
  translate([15, 3, 0]) {
    hole();
    translate([0, -38, 0]) {
      hole();
    }
  }
}

module hole() {
  translate([0, 0, -50]) {
    linear_extrude(height=100) {
      circle(r=1.4);
    }
  }
}
