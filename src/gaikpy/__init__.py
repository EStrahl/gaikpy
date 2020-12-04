
import os
import gaikpy

def get_nico_data_path():


    path =os.path.join(os.path.dirname(gaikpy.__file__), 'resources', 'data','nico')
    if not os.path.exists(path+"/nico_right_20_new.p"):
        from pathlib import Path
        path = Path(gaikpy.__file__).parents[2]
        path = path / 'resources' / 'data' / 'nico' 

    return str(path)

def get_nico_urdf_path():
    path= os.path.join(os.path.dirname(gaikpy.__file__), 'resources', 'urdf','nico')
    if not os.path.exists(path+"/complete_modified_bounds.urdf"):
        from pathlib import Path
        path = Path(gaikpy.__file__).parents[2]
        path = path / 'resources' / 'urdf' / 'nico'
    return str(path)