
import bpy

def create_block(name,x,y,z):
    print("test")
    bpy.ops.mesh.primitive_cube_add(enter_editmode=False, align='WORLD', location=(x, y, z), scale=(1, 1, 1))
    bpy.context.object.name = name


## create path

def create_path(name,x,y,z):
    bpy.ops.curve.primitive_nurbs_path_add(enter_editmode=True, align='WORLD', location=(x, y, z), scale=(1, 1, 1))
    bpy.context.object.name = name


    
def extrude_path(name,x,y,z):
    print("obect selected")
    bpy.data.objects[name].select_set(True)
    print("edit mode toggled")
    bpy.ops.object.editmode_toggle()
    print("poitns unselected")
    bpy.ops.curve.select_all()
    print("last point selected")
    bpy.ops.curve.de_select_last()
    bpy.ops.curve.extrude_move(CURVE_OT_extrude={"mode":'TRANSLATION'}, TRANSFORM_OT_translate={"value":(x,y,z), "orient_type":'LOCAL', "orient_matrix":((1, 0, 0), (0, 1, 0), (0, 0, 1)), "orient_matrix_type":'LOCAL', "constraint_axis":(True, True, True), "mirror":False, "use_proportional_edit":False, "proportional_edit_falloff":'SMOOTH',"proportional_size":1, "use_proportional_connected":False, "use_proportional_projected":False, "snap":False, "snap_target":'CLOSEST',"snap_point":(0, 0, 0), "snap_align":False, "snap_normal":(0, 0, 0), "gpencil_strokes":False, "cursor_transform":False, "texture_space":False,"remove_on_cancel":False, "release_confirm":True, "use_accurate":False,"use_automerge_and_split":False})       

def follow_path(Obj_name,path_name):
    ## set path to follow
    bpy.data.objects[Obj_name].select_set(True)
    bpy.context.space_data.context = 'CONSTRAINT'
    bpy.ops.object.constraint_add(type='FOLLOW_PATH')
    bpy.context.object.constraints["Follow Path"].target = bpy.data.objects[path_name]



def delete_obj():
    bpy.ops.outliner.delete()


create_block("block",0,0,0)
print("creation du block")
create_path("path",0,0,0)
print("creation du path")
extrude_path("path",0,10,0)
print("extrusion du path")
