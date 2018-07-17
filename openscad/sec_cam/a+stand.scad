stlfile = "Raspberry_Pi_A+_holder.stl";

$fn = 50;
cover_thickness = 3;
tube_thickness = 2;
pcb_disk_r = 53.5/2;
cover_disk_r = 54.5/2;
cover_r = pcb_disk_r+tube_thickness;

module stick_with_holes() {
  clen = 5;
  cwidth = 8;
  translate([-cwidth/2, -pcb_disk_r-clen, 0]) {
   difference() {
      union() {
        cube([cwidth, (pcb_disk_r + clen) * 2, cover_thickness]);
        translate([cwidth/2, 0,  0]) {
          cylinder(r=cwidth/2, h=cover_thickness);
        }
        translate([cwidth/2, (pcb_disk_r+clen)*2,  0]) {
          cylinder(r=cwidth/2, h=cover_thickness);
        }
      }
      translate([cwidth/2, 0,  0]) {
        cylinder(r=1.8, h=cover_thickness);
      }
      translate([cwidth/2, (pcb_disk_r+clen)*2,  0]) {
        cylinder(r=1.8, h=cover_thickness);
      }
    }
  }
}


module top_cover() {
  translate([0, 0, 0]) {
    rotate(45) {
      difference() {
        union() {
          cylinder(h=cover_thickness, r=cover_r);
          stick_with_holes();
          rotate(90) {
            stick_with_holes();
          }
        }
        cylinder(h=cover_thickness, r=pcb_disk_r-1);
      }
    }
  }
}

translate([0, 65, 0]) {
  rotate([90, 90, 0]) {
    top_cover();
  }
}

translate([5, 0, 0]) {
  difference() {
    union() {
      translate([16, -35, -1.5]) {
        cube([10, 100, 3]);
      }
      translate([-37, -35, -1.5]) {
        cube([10, 100, 3]);
      }
    }
    translate([20, 14, -20]) {
      cylinder(r=1.8, h=50);
    }
  }
  difference() {
    translate([50, -55, 0]) {
      import(stlfile, convexity=10);
    }
    translate([-42.1, 0, 0]) {
      cube([10, 100, 50], center=true);
    }
    translate([32, 0, 0]) {
      cube([10, 100, 50], center=true);
    }
  }
}
