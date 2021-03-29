from random import seed, uniform
from math import cos, sin, tan, atan

LINK_TAG = '<link name="link_name">\nTAGS\n</link>'
COMP_TAG = '\t<pose> pos_x pos_y 0.15 0 0 rot_z </pose>\n\t<type name="the_name">\n\t\t<geometry>\n\t\t\t<box>\n\t\t\t\t<size>size_x 0.05 0.3</size>\n\t\t\t</box>\n\t\t</geometry>\n\t</type>'

def generate_obstacle(left_wall_y, right_wall_y, length, sections, min_width, r_seed):
    seed(r_seed)
    tag = LINK_TAG.replace('TAGS', COMP_TAG.replace('type','visual').replace('the_name','visual') + '\n' + COMP_TAG.replace('type','collision').replace('the_name','collision'))
    model = '<?xml version="1.0"?>\n<sdf version="1.4">\n<model name="obstacle">\n<pose>1 0 0  0 0 0</pose>\n<static>true</static>'
    if(length>0):
        section_length = length / sections
        passage_points = []
        #left side
        y_pos = left_wall_y
        pos_x = 0
        for i in range(sections):
            min_y = 0
            #min_y = right_wall_y + min_width
            link = tag.replace('link_name','obstacle_left_'+str(i))
            min_rot = - atan((y_pos-min_y)/section_length)
            max_rot = atan((left_wall_y-y_pos)/section_length)
            if(i==sections-1):
                min_rot=max_rot
            rot_z = uniform(min_rot,max_rot)
            pos_y = tan(rot_z)*section_length/2
            size_x = section_length / cos(rot_z)
            link = link.replace('pos_x',str(pos_x)).replace('pos_y',str((y_pos + pos_y))).replace('rot_z',str(rot_z)).replace('size_x',str(size_x))
            y_pos += 2*pos_y
            pos_x += section_length
            passage_points.append(y_pos)
            model += link + '\n'
            
        #right side
        y_pos = right_wall_y
        pos_x = 0
        for i in range(sections):
            #max_y = -min_width/2
            max_y = passage_points[i] - min_width
            link = tag.replace('link_name','obstacle_right_'+str(i))
            min_rot = -atan((y_pos-right_wall_y)/section_length)
            max_rot = atan((max_y-y_pos)/section_length)
            if(i==sections-1):
                max_rot=min_rot
            rot_z = uniform(min_rot,max_rot)
            pos_y = tan(rot_z)*section_length/2
            size_x = section_length / cos(rot_z)
            link = link.replace('pos_x',str(pos_x)).replace('pos_y',str((y_pos + pos_y))).replace('rot_z',str(rot_z)).replace('size_x',str(size_x))
            y_pos += 2*pos_y
            pos_x += section_length
            model += link + '\n'  
    model += '</model></sdf>'
    return model

#print(generate_obstacle(0.5,-0.5,3,10,0.4))
