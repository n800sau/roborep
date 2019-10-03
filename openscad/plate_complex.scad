$fn = 50;
PIN_W = 3;
PIN_D = 2;
PIN_H = 11;
BASE_W = 24 + PIN_W;
BASE_D = 19 + PIN_W;
BASE_H = 2;
LOCK_R = 2;
LOCK_Z = PIN_H - LOCK_R;
STAND_W = 3;
STAND_D = 3;
STAND_H = 2;

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

rotate([0, 0, $t*200]) {
  
difference() {
  translate([0, 0, 1]) {
    union() {
      cube([BASE_W, BASE_D, BASE_H], center=true);
      translate([BASE_W/2-STAND_W-PIN_W, BASE_D/2-STAND_D-PIN_W, 1]) {
        stand();
      }
      translate([-BASE_W/2+PIN_W, BASE_D/2-STAND_D-PIN_W, 1]) {
        stand();
      }
      translate([-BASE_W/2+PIN_W, -BASE_D/2+PIN_W, 1]) {
        stand();
      }
      translate([BASE_W/2-STAND_W-PIN_W, -BASE_D/2+PIN_W, 1]) {
        stand();
      }
    }
  }
  translate([BASE_W/7, BASE_D/4, 0]) {
    hole3m();
  }
  translate([BASE_W/7, -BASE_D/4, 0]) {
    hole3m();
  }
  translate([-BASE_W/7, BASE_D/4, 0]) {
    hole3m();
  }
  translate([-BASE_W/7, -BASE_D/4, 0]) {
    hole3m();
  }
  translate([BASE_W/4, 0, 0]) {
    hole3m();
  }
  translate([-BASE_W/4, 0, 0]) {
    hole3m();
  }
  translate([0, 0, 0]) {
    hole3m();
  }
}

translate([-(BASE_W/2+2-PIN_W), 0, 0]) {
  rotate([0, 0, 90]) {
    pin_lock();
  }
}

translate([(BASE_W/2+2-PIN_W), 0, 0]) {
  rotate([0, 0, 270]) {
    pin_lock();
  }
}

translate([0, (BASE_D/2+2-PIN_W), 0]) {
  rotate([0, 0, 0]) {
    pin_lock();
  }
}

translate([0, -(BASE_D/2+2-PIN_W), 0]) {
  rotate([0, 0, 180]) {
    pin_lock();
  }
}

}
