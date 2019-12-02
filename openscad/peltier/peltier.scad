$fn = 50;

wall = 3;

block_sz_x = 40;
block_sz_y = 40;
block_sz_z = 4;

hole_dist = 72;

cover_sz_x = 80;
cover_sz_y = 80;

hole_d = 4;

heat_block_sz_x = 20;
heat_block_sz_y = 16;
heat_block_sz_z = 12;

heat_block_cover_off_z = heat_block_sz_z - block_sz_z;

fan_bracket_sz_x = 16;
fan_bracket_sz_y = 4;
fan_bracket_sz_z = 77;

fan_bracket_leave_sz_x = fan_bracket_sz_x;
fan_bracket_leave_sz_y = 15;
fan_bracket_leave_sz_z = fan_bracket_sz_y;

fan_bracket_hole_offset = 5;

module pcr_tube() {
  tube_top_d = 9.6;
  tube_top_h = 17;
  tube_middle_d = 7.5;
  tube_middle_h = 12.1;
  tube_bottom_d = 4;
  translate([0, 0, tube_middle_h]) {
    cylinder(d2=tube_top_d, d1=tube_middle_d, h=tube_top_h);
  }
  cylinder(d2=tube_middle_d, d1=tube_bottom_d, h=tube_middle_h);
}

module holes_block(holes=true) {
    for(angle=[0, 90, 180, 270]) {
      rotate([0, 0, angle]) {
        translate([(block_sz_x-wall)/2-4, (block_sz_y-wall)/2-8, 0]) {
          if(holes) {
            cylinder(d=8, h=wall, center=true);
          } else {
            difference() {
              translate([0, 0, 9]) {
                cylinder(d=12, h=20, center=true);
              }
              translate([0, 0, -9]) {
                pcr_tube();
              }
            }
          }
        }
        if(holes) {
          translate([(block_sz_x-wall)/2-17, (block_sz_y-wall)/2-8.5, 0]) {
            cylinder(d=8, h=wall, center=true);
          }
        }
      }
    }
}

module spec_cross(make_hole=true) {
  difference() {
    translate([0, 0, -block_sz_z/2]) {
      difference() {
        union() {
          cube([block_sz_x+14, block_sz_y+14, wall], center=true);
          for(angl=[0,1]) {
              rotate([0, 0, 45+angl*90]) {
                cube([cover_sz_x+20, 15, wall], center=true);
                for(xsgn=[-1, 1]) {
                  translate([xsgn*(cover_sz_x+20)/2, 0, 0]) {
                    cylinder(d=15, h=wall, center=true);
                  }
                }
              }
          }
          holes_block(holes=false);
        }
        if(make_hole) {
          cube([block_sz_x-wall, block_sz_y-wall, wall], center=true);
        } else {
          holes_block();
        }
        for(xsgn=[-1,1]) {
          for(ysgn=[-1,1]) {
            translate([xsgn*hole_dist/2, ysgn*hole_dist/2, 0]) {
              cylinder(d=hole_d, h=50, center=true);
            }
          }
        }
      }
    }
  }
}

module peltier_fan_holder() {
  difference() {
    cube([block_sz_x+2*wall, block_sz_y+2*wall, block_sz_z], center=true);
    cube([block_sz_x, block_sz_y, block_sz_z], center=true);
    for(xsgn=[-1,1]) {
      for(ysgn=[-1,1]) {
        translate([xsgn*block_sz_x/2, ysgn*block_sz_y/2, 0]) {
          cube([10, 10, 10], center=true);
        }
      }
    }
  }
  spec_cross();
}

module heater_holder() {
  spec_cross(false);
}

module stand() {
    difference() {
      cylinder(d=hole_d+wall*2, h=heat_block_cover_off_z, center=true);
      cylinder(d=hole_d, h=heat_block_cover_off_z, center=true);
    }
}

module fan_bracket() {
  cube([fan_bracket_sz_x, fan_bracket_sz_y, fan_bracket_sz_z], center=true);
  for(zsgn=[-1,1]) {
    translate([0, fan_bracket_leave_sz_y/2, zsgn*fan_bracket_sz_z/2]) {
      difference() {
        cube([fan_bracket_leave_sz_x, fan_bracket_leave_sz_y+fan_bracket_sz_y, fan_bracket_leave_sz_z], center=true);
        translate([0, (fan_bracket_leave_sz_y+fan_bracket_sz_y)/2-fan_bracket_hole_offset, 0]) {
          cylinder(d=hole_d, h=20, center=true);
        }
      }
    }
  }
}
 
rotate([180, 0, 0]) {
  translate([0, 0, (block_sz_z+wall)/2]) {
    //peltier_fan_holder();
  }
}
translate([0, 0, heat_block_cover_off_z+(block_sz_z+wall)/2]) {
//	scale([1, 1, 0.3]) {
//    heater_holder();
//  }
}
translate([0, 0, (block_sz_z+wall)/2]) {
  for(xsgn=[-1,1]) {
    for(ysgn=[-1,1]) {
      translate([xsgn*hole_dist/2, ysgn*hole_dist/2, 0]) {
//        stand();
      }
    }
  }
}

translate([0, 0, -fan_bracket_sz_z/2-10]) {
  //fan_bracket_move = fan_bracket_leave_sz_y+2;
  fan_bracket_move = fan_bracket_leave_sz_y+fan_bracket_sz_y-fan_bracket_hole_offset/2;
  for(xsgn=[-1,1]) {
    for(ysgn=[-1,1]) {
      translate([xsgn*hole_dist/2-ysgn*xsgn*fan_bracket_move/2-((ysgn>0) ? ((xsgn<0) ? fan_bracket_move : -fan_bracket_move): 0), ysgn*hole_dist/2-fan_bracket_move/2+((ysgn>0) ? fan_bracket_move: 0), 0]) {
        rotate([0, 0, -45*ysgn*xsgn+((ysgn>0) ? 180 : 0)]) {
          fan_bracket();
        }
      }
    }
  }
}
