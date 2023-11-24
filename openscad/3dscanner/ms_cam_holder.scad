ins_h = 30;
ins_w = 4.5;
ins_l = 40;
ins_wall = 2;

wall = 4;
attach_w = 20 + wall*2;

// bolted one
module frame_wall(has_hole=true) {
  difference() {
    union() {
      translate([-attach_w/2, 0, 0]) {
        cube([attach_w, wall, ins_h]);
        translate([attach_w/2-5.5/2, wall, 0]) {
          cube([5.5, 1, ins_h]);
        }
      }
    }
    if(has_hole) {
      translate([0, 8, ins_h/2-5]) {
        rotate([90, 0, 0])
          cylinder(d=4, h=10);
      }
    }
  }
}
module container() {
  // container
  difference() {
    cube([ins_l+ins_wall*2, ins_w+ins_wall*2, ins_h]);
    translate([ins_wall, ins_wall, ins_wall]) {
      cube([ins_l, ins_w, ins_h]);
    }
    translate([ins_wall, 0, ins_wall]) {
      cube([ins_l, ins_w, 8]);
      translate([0, 0, 15]) {
        cube([ins_l, ins_w, 8]);
      }
    }
  }
}

module frame_holder() {
  translate([ins_l/2+wall, wall+ins_w, 0]) {
    frame_wall(has_hole=false);
  }
  translate([ins_l/2-10.5,  attach_w/2+wall+ins_w, ins_h]) {
    rotate([180, 0, 90]) {
      frame_wall(has_hole=true);
    }
  }
  translate([ins_l/2+wall*2+10.5, attach_w/2+wall+ins_w, ins_h]) {
    rotate([180, 0, -90]) {
      frame_wall(has_hole=true);
    }
  }
}

translate([0, -ins_wall, 0])
  frame_holder();
container();

translate([-9, wall+ins_w, 0]) {
  difference() {
    cube([20, 20+wall, 4]);
    translate([9, 12.5, -2.5])
      cylinder(d=4, h=10);
  }
}

translate([-9+ins_l+wall*2-2, wall+ins_w, 0]) {
  difference() {
    cube([20, 20+wall, 4]);
    translate([12, 12.5, -2.5])
      cylinder(d=4, h=10);
  }
}
