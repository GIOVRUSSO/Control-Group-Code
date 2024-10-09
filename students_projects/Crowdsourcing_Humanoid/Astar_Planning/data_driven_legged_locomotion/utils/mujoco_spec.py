import numpy as np
import pathlib

def attach_spec(dest, source, source_dir, prefix="", body_pos = np.array([0.0, 0.0, 0.0]), body_quat = np.array([1.0, 0.0, 0.0, 0.0])):
    for mesh in source.mesh:
        new_mesh = dest.add_mesh()
        new_mesh.name = prefix + mesh.file.split(".")[0]
        mesh_path = source_dir / pathlib.Path(mesh.file)
        new_mesh.file = str(mesh_path.resolve())

    for texture in source.texture:
        new_texture = dest.add_texture()
        new_texture.name = prefix + texture.name
        new_texture.nchannel = texture.nchannel
        new_texture.type = texture.type
        texture_path = source_dir / pathlib.Path(texture.file)
        new_texture.file = str(texture_path.resolve())
        
    for material in source.material:
        new_material = dest.add_material()
        new_material.name = prefix + material.name
        new_material.rgba = material.rgba
        new_material.specular = material.specular
        new_material.shininess = material.shininess
        new_material.metallic = material.metallic
        new_material.roughness = material.roughness
        new_material.specular = material.specular
        new_material.emission = material.emission
        new_textures = []
        for texture in material.textures:
            if texture != "":
                new_textures.append(prefix + texture)
            else:
                new_textures.append(texture)
        new_material.textures = new_textures

    body = source.worldbody.first_body()
    while body:
        new_body = dest.worldbody.add_body()
        new_body.name = prefix + body.name
        new_body.pos = body_pos
        new_body.quat = body_quat
        geom = body.first_geom()
        while geom:
            new_geom = new_body.add_geom()
            if geom.name != "":
                new_geom.name = prefix + geom.name
            new_geom.type = geom.type
            new_geom.size = geom.size
            new_geom.pos = geom.pos
            new_geom.quat = geom.quat
            if geom.material != "":
                new_geom.material = prefix + geom.material
            if geom.meshname != "":
                new_geom.meshname = prefix + geom.meshname
            new_geom.contype = geom.contype
            new_geom.conaffinity = geom.conaffinity
            new_geom.group = geom.group
            geom = body.next_geom(geom)
        body = source.worldbody.next_body(body)