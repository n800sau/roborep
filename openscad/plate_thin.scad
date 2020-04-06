$fn = 50;
BASE_W = 22;
BASE_D = 17;
BASE_H = 1;
PIN_W = 3;
PIN_D = 1;
PIN_H = 7;
LOCK_R = 1.5;
LOCK_Z = PIN_H - LOCK_R;
STAND_W = 3;
STAND_D = 3;
STAND_H = 1;

module pin_lock() {
  difference() {
    union() {
      translate([0, 0, PIN_H/2]) {
        cube([PIN_W, PIN_D, PIN_H], center=true);
      }
      translate([0, 0, LOCK_Z]) {
        rotate([0, 90, 0]) {
          cylinder(r=LOCK_R, h=PIN_W, center=true);
        }
      }
    }
    translate([0, PIN_D, PIN_H/2]) {
      cube([PIN_W, PIN_D, PIN_H], center=true);
    }
  }
}

module hole3m() {
  cylinder(r=1.8, h=10, center=true);
}

module stand() {
  cube([STAND_W, STAND_D, STAND_H], center=false);
}

difference() {
  translate([0, 0, 0.5]) {
    union() {
      cube([BASE_W, BASE_D, BASE_H], center=true);
      translate([BASE_W/2-STAND_W, BASE_D/2-STAND_D, 0.5]) {
        stand();
      }
      translate([-BASE_W/2, BASE_D/2-STAND_D, 0.5]) {
        stand();
      }
      translate([-BASE_W/2, -BASE_D/2, 0.5]) {
        stand();
      }
      translate([BASE_W/2-STAND_W, -BASE_D/2, 0.5]) {
        stand();
      }
    }
  }
  translate([BASE_W/4, BASE_D/4, 0]) {
    hole3m();
  }
  translate([BASE_W/4, -BASE_D/4, 0]) {
    hole3m();
  }
  translate([-BASE_W/4, BASE_D/4, 0]) {
    hole3m();
  }
  translate([-BASE_W/4, -BASE_D/4, 0]) {
    hole3m();
  }
  translate([BASE_W/4, 0, 0]) {
    hole3m();
  }
  translate([-BASE_W/4, 0, 0]) {
    hole3m();
  }
}

translate([-(BASE_W/2+0.5), 0, 0]) {
  rotate([0, 0, 90]) {
    pin_lock();
  }
}

translate([(BASE_W/2+0.5), 0, 0]) {
  rotate([0, 0, 270]) {
    pin_lock();
  }
}

translate([0, (BASE_D/2+0.5), 0]) {
  rotate([0, 0, 0]) {
    pin_lock();
  }
}

translate([0, -(BASE_D/2+0.5), 0]) {
  rotate([0, 0, 180]) {
    pin_lock();
  }
}

