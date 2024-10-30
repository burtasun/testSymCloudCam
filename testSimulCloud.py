import numpy as np
import open3d as o3d #sino funciona instala el wheel del repo en si: https://github.com/isl-org/Open3D/releases (archivos WHL)

if __name__=='__main__':
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    #read model
    rawMesh = o3d.io.read_triangle_model(r'.\Motorblock.STL')
    for mesh in rawMesh.meshes:
        vis.add_geometry(mesh.mesh)
    #se inicializa visor, se capturara en funcion del tamaño ventana (Resolucion) y de la perspectiva.
    #(se pueden definir programaticamente de mil formas)
    #Aquí
    #   https://www.open3d.org/docs/latest/python_api/open3d.visualization.O3DVisualizer.html
    #   puedes ver algun método del visualizador para definir la matriz intrínseca y extrínseca
    vis.run()

    #ahora se captura la nube parcial en cuestion
    filename="nubeCapturada.ply" #puedes ver las nubes de mil formas, yo uso cloudCompare https://cloudcompare-org.danielgm.net/release/
    #En cloud compare puedes visualizar todos los atributos que quieras de los puntos. 
    #Por cada punto 3D podríamos tener una colección de escalares, que podrían estar asociados a diferentes bandas del espectro por ejemplo.
    vis.capture_depth_point_cloud(filename)
    pcd = o3d.io.read_point_cloud(filename)
    #se le podrian anexar los colores (si los renderizara), leyendo el buffer de la imagen
    # vis.capture_screen_float_buffer(image) #posteriormente habria que anexarlos
    
    #por defecto es una lista plana de puntos, esta libreria tiene tensores/voxeles sparse 3D
    xyz = np.asarray(pcd.points)#y como no, se pueden manipular los datos con numpy!
    xyz = np.nan_to_num(xyz)#el fondo es nan por defecto

    #Bounding box
    mM=[np.min(xyz),np.max(xyz)]
    print(f'{mM}')
#end    