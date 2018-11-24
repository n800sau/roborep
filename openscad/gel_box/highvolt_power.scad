use <MCAD/boxes.scad>

pad = 0.5;
pcb_x = 45.2 + 2*pad;
pcb_y = 37.2 + 2*pad;
pcb_z = 1.6 + 2*pad;
hole_off_y = 23 + pad;
hole_off_x = 2 + pad;
hole_dist_x = 41;
wall = 2;

vm_x = 37.7 + pad;
vm_outer_x = 40.5 + pad;
vm_h = 20.7 + pad;

banana_hole_d = 4.6;
banana_outer_hole_d = 11.6;
banana_outer_border_d = 12.4;
banana_hole_z = wall + pad + max(banana_outer_border_d/2, vm_h/2);

inppad_x = 14;
box_x = wall + inppad_x + pad + pcb_x + pad + vm_outer_x
  + pad + banana_outer_border_d + pad + wall;
box_y = wall + pcb_y + wall;

elem_h = 14;
under_pcb_h = 2.5;

height = wall+max(under_pcb_h+pcb_z+elem_h, vm_h);
echo("height:", height);
full_height = height+2+wall;
echo("full height:", full_height);

$fn = 50;

module switch_holes() {
  cube([9.5, 50, 5], center=true);
  translate([9.5, 0, 0]) {
    rotate([90, 0, 0]) {
      cylinder(d=2.5, h = 50, center=true);
    }
  }
  translate([-9.5, 0, 0]) {
    rotate([90, 0, 0]) {
      cylinder(d=2.5, h = 50, center=true);
    }
  }
}

module banana_jaycar_female_hole() {
  h = 20;
  rotate([90, 0, 0]) {
    difference() {
      cylinder(d=banana_hole_d, h=h);
      translate([-10, banana_hole_d/2-0.5, 0]) {
        cube([20, 5, h]);
      }
      translate([-10, -banana_hole_d/2-5+0.5, 0]) {
        cube([20, 5, h]);
      }
    }
  }
}

module banana_female_hole(border=false) {
  h = 20;
  rotate([90, 0, 0]) {
    union() {
      if(border) {
        translate([0, 0, h/2]) {
          difference() {
            cylinder(d=banana_outer_border_d + 4, h=2);
            cylinder(d=banana_outer_border_d, h=2);
          }
        }
      } else {
        cylinder(d=banana_hole_d, h=h);
        translate([0, 0, h/2]) {
          cylinder(d=banana_outer_hole_d, h=h);
        }
      }
    }
  }
}

module pcb_holes(border=false, h=20) {
    translate([hole_off_x, hole_off_y, -h/2]) {
      d = 1.9 + (border ? 4 : 0);
      color("blue") {
      cylinder(d=d, h=h);
      translate([hole_dist_x, 0, 0]) {
        cylinder(d=d, h=h);
      }
      }
    }
}

module high_volt_box() {
  difference() {
    union() {
      difference() {
        cube([box_x, box_y, full_height]);
        translate([wall, wall, wall]) {
          cube([box_x-wall*2, box_y-wall*2, 50]);
        }
      }
      translate([wall+inppad_x, wall, wall+under_pcb_h/2]) {
        pcb_holes(border=true, h=under_pcb_h);
      }
    }
    translate([wall+inppad_x, wall, wall]) {
      pcb_holes();
    }
  }
  translate([wall+inppad_x, wall, wall]) {
    cube([pcb_x, (box_y-pcb_y)/2, under_pcb_h]);
    translate([0, box_y-wall*2-(box_y-pcb_y)/2, 0]) {
      cube([pcb_x, (box_y-pcb_y)/2, under_pcb_h]);
    }
  }
}

module banana_holes(border=false) {
  translate([wall+pad+(inppad_x)/2, 10, banana_hole_z]) {
    banana_female_hole(border=border);
  }
  translate([wall+pad+inppad_x+pad+pcb_x+pad+vm_outer_x+pad+
      banana_outer_border_d/2, 0, 0]) {
    translate([0, 10, banana_hole_z]) {
      banana_female_hole(border=border);
    }
  }
}

module vmeter_hole() {
  cube([vm_x, 20, vm_h]);
}

module high_volt_enclosure() {
  difference() {
    union() {
      high_volt_box();
      banana_holes(border=true);
    }
    // lid_holes
    translate([-10, 0, full_height-5]) {
      rotate([0, 90, 0]) {
        translate([0, 10, 0]) {
          cylinder(d=3.2, h=box_x+20);
        }
        translate([0, box_y-10, 0]) {
          cylinder(d=3.2, h=box_x+20);
        }
      }
    }
    // power hole
    translate([wall + inppad_x/2, box_y, banana_hole_z]) {
      rotate([90, 90, 0]) {
        cylinder(d=8.8, h=10);
      }
    }
    banana_holes(border=false);
    // power switch hole
    translate([0, pcb_x/2, banana_hole_z]) {
      rotate([0, 0, 90]) {
        switch_holes();
      }
    }
    // volt meter
    translate([wall+pad+inppad_x+pad+pcb_x, 0, wall+pad]) {
      vmeter_hole();
    }
  }
}

high_volt_enclosure();
