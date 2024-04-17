#include <iostream>
#include <fstream> // Pour l'écriture dans un fichier
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

namespace fs = boost::filesystem;

typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;

int main() {
    // Chemins des dossiers d'entrée et de sortie
    std::string input_directory_upper = "C:/Users/etien/source/repos/segmentation/segmentation/planes/marseille_centre_ville_quart_cluster";
    std::string input_directory_lower = "C:/Users/etien/source/repos/segmentation/segmentation/footprint/marseille_centre_ville_quart_cluster";
    std::string output_directory = "C:/Users/etien/source/repos/segmentation/segmentation/boites/marseille_centre_ville_quart_cluster";


    fs::create_directory(output_directory);

    // Ouverture du fichier pour écrire les surfaces totales
    std::ofstream surface_file(output_directory + "/surfaces.txt");

    if (!surface_file.is_open()) {
        std::cerr << "Unable to open surface file for writing." << std::endl;
        return 1;
    }

    // Itérer simultanément sur tous les fichiers PCD dans les deux dossiers
    auto upper_it = fs::directory_iterator(input_directory_upper);
    auto lower_it = fs::directory_iterator(input_directory_lower);

    while (upper_it != fs::end(upper_it) && lower_it != fs::end(lower_it)) {
        if (upper_it->path().extension() == ".pcd" && lower_it->path().extension() == ".pcd") {
            std::string input_pcd_path_upper = upper_it->path().string();
            std::string input_pcd_path_lower = lower_it->path().string();

            // Charger le nuage de points de la partie supérieure depuis le fichier PCD
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_upper(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_path_upper, *cloud_upper) == -1) {
                std::cerr << "Couldn't read file " << input_pcd_path_upper << std::endl;
                continue;
            }

            // Charger le nuage de points de la partie inférieure depuis le fichier PCD
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lower(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_path_lower, *cloud_lower) == -1) {
                std::cerr << "Couldn't read file " << input_pcd_path_lower << std::endl;
                continue;
            }

            // Identifier les coins supérieurs de la boîte englobante (4 points les plus éloignés)
            pcl::PointXYZ min_pt_upper, max_pt_upper;
            pcl::getMinMax3D(*cloud_upper, min_pt_upper, max_pt_upper);

            // Identifier les coins inférieurs de la boîte englobante (4 points les plus éloignés)
            pcl::PointXYZ min_pt_lower, max_pt_lower;
            pcl::getMinMax3D(*cloud_lower, min_pt_lower, max_pt_lower);

            MyMesh mesh;

            // Ajouter les sommets du cube
            MyMesh::VertexHandle vhandle[8];

            vhandle[0] = mesh.add_vertex(MyMesh::Point(min_pt_lower.x, min_pt_lower.y, min_pt_lower.z));
            vhandle[1] = mesh.add_vertex(MyMesh::Point(max_pt_lower.x, min_pt_lower.y, min_pt_lower.z));
            vhandle[2] = mesh.add_vertex(MyMesh::Point(max_pt_lower.x, max_pt_lower.y, min_pt_lower.z));
            vhandle[3] = mesh.add_vertex(MyMesh::Point(min_pt_lower.x, max_pt_lower.y, min_pt_lower.z));

            vhandle[4] = mesh.add_vertex(MyMesh::Point(min_pt_upper.x, min_pt_upper.y, min_pt_upper.z));
            vhandle[5] = mesh.add_vertex(MyMesh::Point(max_pt_upper.x, min_pt_upper.y, min_pt_upper.z));
            vhandle[6] = mesh.add_vertex(MyMesh::Point(max_pt_upper.x, max_pt_upper.y, min_pt_upper.z));
            vhandle[7] = mesh.add_vertex(MyMesh::Point(min_pt_upper.x, max_pt_upper.y, min_pt_upper.z));

            // Ajouter les faces du cube
            std::vector<MyMesh::VertexHandle> face_vhandles;

            // Face inférieure
            face_vhandles.clear();
            face_vhandles.push_back(vhandle[0]);
            face_vhandles.push_back(vhandle[1]);
            face_vhandles.push_back(vhandle[2]);
            face_vhandles.push_back(vhandle[3]);
            mesh.add_face(face_vhandles);

            // Face supérieure
            face_vhandles.clear();
            face_vhandles.push_back(vhandle[4]);
            face_vhandles.push_back(vhandle[5]);
            face_vhandles.push_back(vhandle[6]);
            face_vhandles.push_back(vhandle[7]);
            mesh.add_face(face_vhandles);

            // Faces latérales
            for (int i = 0; i < 4; ++i) {
                face_vhandles.clear();
                face_vhandles.push_back(vhandle[i]);
                face_vhandles.push_back(vhandle[(i + 1) % 4]);
                face_vhandles.push_back(vhandle[(i + 5) % 4 + 4]);
                face_vhandles.push_back(vhandle[(i + 4) % 4 + 4]);
                mesh.add_face(face_vhandles);
            }

            // Extraire le nom de fichier sans extension
            fs::path input_file_path_upper(input_pcd_path_upper);
            std::string output_filename = input_file_path_upper.stem().string() + "_cube.obj";
            std::string output_obj_path = output_directory + "/" + output_filename;

            // Sauvegarder le maillage au format OBJ
            if (!OpenMesh::IO::write_mesh(mesh, output_obj_path)) {
                std::cerr << "Cannot write mesh to file '" << output_obj_path << "'" << std::endl;
                return 1;
            }

            std::cout << "Mesh exported to '" << output_obj_path << "'" << std::endl;

            // Calculer les dimensions du cube
            double width = max_pt_upper.x - min_pt_upper.x;
            double height = max_pt_upper.y - min_pt_upper.y;
            double depth = max_pt_upper.z - min_pt_upper.z;

            // Calculer la surface totale sans la face inférieure
            double total_surface = 2 * ((width * height) + (height * depth)) + (width * depth);

            // Sauvegarder le maillage au format OBJ
            surface_file << output_filename << ": " << total_surface << " units squared" << std::endl;


            std::cout << "Surface of " << output_filename << " calculated and written to file." << std::endl;
        }
        ++upper_it;
        ++lower_it;
    }

    // Fermer le fichier des surfaces
    surface_file.close();

    return 0;
}
