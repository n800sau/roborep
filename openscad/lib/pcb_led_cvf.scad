// for led power module modified

/* bluetooth_mate_holder.scad
 * By John Biehler - November 17, 2011
 * http://www.thingiverse.com/thing:

 * derived from pcb_box_slide_with_tabs.scad
 * By Nicholas C Lewis - Jan 1, 2011
 * http://www.thingiverse.com/thing:5396
 *
 * derived from pcb_box_slide.scad
 *     Parametric, open PCB-box (slide-in) by chlunde
 *     http://www.thingiverse.com/thing:4071
 *
 * CHANGES
 * - adjusted to fit Arduino Mega
 * - added mounting tabs
 * - turned off cutouts
 * - adjusted bottom cutout size
 */

$fn=50;

module pcb_led_cvf(

	pcb_width = 48.2,
	pcb_length =24.1,
	pcb_height =1.1,
	slisse = 0.8,

	pcb_bottom_margin = 1,
	wall_width_thick = 2,
	box_height = 7,

	//tab parameters
	tab_hole_radius=2,
	tab_outer_radius=4,

	inner_margin = 4,

	add_tabs = true

) {
	wall_width = wall_width_thick-slisse;

module tab() {
	difference(){
		union() {
			translate([-tab_outer_radius,-tab_outer_radius,0])
			cube([tab_outer_radius,2*tab_outer_radius,wall_width_thick+1]);
			translate([-tab_outer_radius,0,0])
				cylinder(r=tab_outer_radius, h = wall_width_thick+1);
		}
		translate([-tab_outer_radius,0,0]) {
			cylinder(r= tab_hole_radius, h = wall_width_thick+5,center=true);
		}
	}

}

module box() {
	union() {
		cube([pcb_width + 2*wall_width, pcb_length + 2*wall_width, wall_width]);

		cube([wall_width_thick, pcb_length + 2*wall_width, box_height]);

		cube([pcb_width + 2*wall_width, wall_width_thick, box_height]);

		%translate([0, pcb_length+wall_width, 0])
			cube([pcb_width + 2*wall_width,wall_width,box_height]);

		translate([pcb_width + wall_width - slisse,0,0])
			cube([wall_width_thick, pcb_length + 2*wall_width, box_height]);
	}
	if(add_tabs) {
//comment out this part to remove tabs
		for(i=[(pcb_length + 2*wall_width)/2]){
	translate([0,i,0])tab();
	translate([pcb_width + 2*wall_width,i,0])mirror()tab();
		}
	}

}

module pcb() {
	translate([wall_width, wall_width, wall_width + pcb_bottom_margin])
		cube([pcb_width,pcb_length + wall_width + 0.1,pcb_height]);
	translate([wall_width, wall_width, wall_width + pcb_bottom_margin+pcb_height])rotate([0,45,0])
		cube([pcb_height,pcb_length + wall_width + 0.1,pcb_height]);
	translate([wall_width+pcb_width-2*cos(45)*pcb_height, wall_width, wall_width + pcb_bottom_margin+pcb_height])rotate([0,45,0])
		cube([pcb_height,pcb_length + wall_width + 0.1,pcb_height]);
	translate([wall_width_thick, wall_width, wall_width + pcb_bottom_margin])
		cube([pcb_width-2*slisse,pcb_length + wall_width + 0.1,box_height]);

}



module holes() {
		*translate([wall_width + pcb_width/2, wall_width_thick + 0.1, pcb_width/2+wall_width])
			rotate(a=90,v=[1,0,0])
				cylinder(r=pcb_width/2, h=wall_width_thick + 0.2);

		*translate([-0.1, wall_width + pcb_length/2, pcb_length/2 + wall_width])
			rotate(a=90,v=[0,1,0])
				cylinder(r=pcb_length/2, h=wall_width_thick + 0.2);

		*translate([pcb_width + wall_width - slisse - 0.1, wall_width + pcb_length/2, pcb_length/2 + wall_width])
			rotate(a=90,v=[0,1,0])
				cylinder(r=pcb_length/2, h=wall_width_thick + 0.2);

	 * translate([pcb_width/2 + wall_width, wall_width + pcb_length/2, -0.1])
			rotate(a=0,v=[0,1,0])
				cylinder(r=min(pcb_length,pcb_width)/2 - wall_width/2, h=wall_width_thick + 0.2);

		translate([inner_margin +wall_width, inner_margin+wall_width, -0.1])
			cube([pcb_width-inner_margin*2,pcb_length -inner_margin*2,wall_width + 0.2]);
}

	difference() {
		box();
		pcb();
		holes();
	}
}



pcb_led_cvf();
