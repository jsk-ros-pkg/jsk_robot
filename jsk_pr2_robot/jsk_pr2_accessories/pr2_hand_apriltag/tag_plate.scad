board_width = 63;
board_length = 55;
board_height = 6.0;
hole_diameter = 3.2;
screw_head_diameter_with_margin = 6.3;
screw_head_depth_with_margin = 3.2;

hole_ypos = 15;
hole_interval = 50;
hole1_pos = [(board_width - hole_interval) * 0.5, hole_ypos];
hole2_pos = [(board_width + hole_interval) * 0.5, hole_ypos];

difference() {
    cube([board_width, board_length, board_height]);
    translate([hole1_pos[0], hole1_pos[1], 0]){
        cylinder(h = board_height, d = hole_diameter, $fn = 32);
    }
    translate([hole2_pos[0], hole2_pos[1], 0]){
        cylinder(h = board_height, d = hole_diameter, $fn = 32);
    }
    translate([hole1_pos[0], hole1_pos[1], 0]){
        cylinder(h = screw_head_depth_with_margin, d = screw_head_diameter_with_margin, $fn = 32);
    }
    translate([hole2_pos[0], hole2_pos[1], 0]){
        cylinder(h = screw_head_depth_with_margin, d = screw_head_diameter_with_margin, $fn = 32);
    }

}
