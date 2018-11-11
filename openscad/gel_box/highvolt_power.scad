pcb_x = 45;
pcb_y = 37;
pcb_z = 2;
pcb_pad = 1;
wall = 2;

elem_h = 14;
under_pcb_h = 2.5;

height = wall+under_pcb_h+pcb_z+elem_h;
echo("height:", height);
full_height = height+20;
echo("full height:", full_height);

banana_hole_z = wall+under_pcb_h+pcb_z+elem_h+10;


banana_hole_d = 12;

volt_meter_x = 48;
volt_meter_y = 29;
volt_meter_h = 25;

$fn = 50;

module switch_holes() {
  cube([9, 50, 5], center=true);
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

module banana_female_hole() {
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

module high_volt_box() {
  difference() {
    translate([-wall, -wall, -wall]) {
      cube([pcb_x+pcb_pad*2+wall*2, pcb_y+pcb_pad*2+wall*2, full_height]);
    }
    translate([0, 0, under_pcb_h]) {
      cube([pcb_x+pcb_pad*2, pcb_y+pcb_pad*2, 50]);
    }
    translate([0, 2, 0]) {
      cube([pcb_x+pcb_pad*2, pcb_y+pcb_pad*2-4, under_pcb_h]);
    }
  }
}

difference() {
  high_volt_box();
  translate([12, 10, banana_hole_z]) {
    banana_female_hole();
  }
  translate([pcb_x-12+wall, 10, banana_hole_z]) {
    banana_female_hole();
  }
  // power hole
  translate([pcb_x/2, pcb_y+8, banana_hole_z]) {
    rotate([90, 90, 0]) {
      cylinder(d=9, h=10);
    }
  }
  // power switch hole
  translate([0, pcb_x/2, banana_hole_z]) {
    rotate([0, 0, 90]) {
      switch_holes();
    }
  }
}
