# Usage: blender --background --python convert_dae2stl.py -- folder_path

import bpy
import sys, os
import glob

argv = sys.argv
argv = argv[argv.index("--") + 1:] # get all args after "--"

folder_path = argv[0]
abs_folder_path = os.path.realpath(os.path.sep.join([folder_path, '*.dae']))
daes = glob.glob(abs_folder_path)


for dae_in in daes:
    print(f'Converting "{dae_in}"...')
    bpy.ops.wm.read_homefile(use_empty=True)
    bpy.ops.wm.collada_import(filepath=dae_in)
    # Clear scene from non mesh elements
    for obj in bpy.context.scene.objects:
        if obj.type == 'MESH':
            obj.select_set(False)
        else:
            obj.select_set(True)
    bpy.ops.object.delete()
 
    fpath = dae_in[:-3] + 'stl'
    try:
        bpy.ops.export_mesh.stl(filepath=fpath)
    except e as Exception:
        print(e)
    finally:
        print(f'{fpath} ok!')
        
print('Convertion done!')
