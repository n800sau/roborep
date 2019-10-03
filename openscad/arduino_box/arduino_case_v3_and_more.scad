//design and code by emanresu (http://www.thingiverse.com/Emanresu)
//generates a simple tray with standoffs for an arduino to sit in


//CUSTOMIZER VARIABLES

//arduino type
arduino_type = "NanoExt"; //[Uno,Mega,NanoExt]

//thickness of side walls and bottom (mm)
thick = 1.6;
//height of side walls (mm)
wall_height = 15;
//height of standoff (mm)
standoff_height = 7;
//standoff hole diameter (mm)
standoff_hole_diameter = 2.9;
//standoff thickness (mm)
standoff_thick = 1.6;
//clearance between arduino and sidewalls (mm)
clearance = 1.5;

// make bottom hole to speed up printing (For Uno and Nano only)
bottom_hole = 1;

/* [Hidden] */

//sets openSCAD resolution for nice, round holes
$fa=1;
$fs=0.5;

base_x = (arduino_type == "Mega" ? 101.6 : (arduino_type == "NanoExt" ? 57.3 : 68.58));
base_y = 53.34;

   ///////////////
  //  Modules  //
 ///////////////

module Arduino_standoffs() {
	// body...
	//arranging standoffs
	translate([13.97, 2.54, standoff_height/2]) {
		difference() {
			cylinder(r=standoff_hole_diameter/2+standoff_thick, h=standoff_height, center=true);
			cylinder(r=standoff_hole_diameter/2, h=standoff_height+1, center=true);
		}	
	}
	translate([15.24, 50.8, standoff_height/2]) {
		difference() {
			cylinder(r=standoff_hole_diameter/2+standoff_thick, h=standoff_height, center=true);
			cylinder(r=standoff_hole_diameter/2, h=standoff_height+1, center=true);
		}	
	}
	translate([66.04, 7.62, standoff_height/2]) {
		difference() {
			cylinder(r=standoff_hole_diameter/2+standoff_thick, h=standoff_height, center=true);
			cylinder(r=standoff_hole_diameter/2, h=standoff_height+1, center=true);
		}	
	}
	translate([66.04, 35.56, standoff_height/2]) {
		difference() {
			cylinder(r=standoff_hole_diameter/2+standoff_thick, h=standoff_height, center=true);
			cylinder(r=standoff_hole_diameter/2, h=standoff_height+1, center=true);
		}	
	}
	
	if (arduino_type == "Mega") {
		//adds extra standoffs for Mega, if specified
		translate([90.17, 50.8, standoff_height/2]) {
			difference() {
				cylinder(r=standoff_hole_diameter/2+standoff_thick, h=standoff_height, center=true);
				cylinder(r=standoff_hole_diameter/2, h=standoff_height+1, center=true);
			}	
		}
		translate([96.52, 2.54, standoff_height/2]) {
			difference() {
				cylinder(r=standoff_hole_diameter/2+standoff_thick, h=standoff_height, center=true);
				cylinder(r=standoff_hole_diameter/2, h=standoff_height+1, center=true);
			}	
		}	
	}
}

module Arduino_case_body() {
	// body...
	
	//sets length for type of arduino specified, 101.6 mm for Mega, 68.58 mm for Uno
	
  difference() {
    cube(size=[base_x+2*thick+2*clearance, base_y+2*thick+2*clearance, wall_height+thick], center=false);
    translate([thick, thick, thick]) {
      //main compartment
      cube(size=[base_x+2*clearance, base_y+2*clearance, wall_height+1], center=false);
    }
    translate([thick/2, 7.62+thick+clearance+(arduino_type=="NanoExt" ? 3 : 0), wall_height/2+thick+standoff_height]) {
      //power plug gap
      cube(size=[thick+1, 8.89+2*clearance, wall_height], center=true);
    }
    if(arduino_type != "NanoExt") {
      translate([thick/2, 38.1+thick+clearance,wall_height/2+thick+standoff_height]) {
        //USB plug gap
        cube(size=[thick+1, 11.43+2*clearance, wall_height], center=true);
      }
    }
  }	
}

module arduino_case_attachments() {
  for(ix=[-1, 1]) {
    translate([ix*15, 0, 0]) {
      difference() {
        union() {
          cube([10, base_y+14, thick], center=true);
          for(iy=[-1,1]) {
            translate([0, iy*(base_y+14)/2, 0]) {
              cylinder(d=10, h=thick, center=true);
            }
          }
        }
        for(iy=[-1,1]) {
          translate([0, iy*(base_y+14)/2, 0]) {
            cylinder(d=3.5, h=thick, center=true);
          }
        }
      }
    }
  }
}


   ////////////////////
  //  Control Code  //
 ////////////////////

difference() {
  union() {
    //Joins case body with standoffs
    Arduino_case_body();
    translate([(thick+clearance+base_x)/2, 30, thick/2]) {
      arduino_case_attachments();
    }
    translate([thick+clearance-(arduino_type == "NanoExt" ? 68.58-57.3 : 0), thick+clearance, thick]) {
      Arduino_standoffs();
    }
  }
  if(bottom_hole && arduino_type != "Mega") {
    translate([thick+clearance+3, thick+clearance+7, 0]) {
      cube(size=[base_x-10, base_y-14, wall_height+thick], center=false);
    }
  }
}
